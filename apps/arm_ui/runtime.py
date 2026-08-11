"""ROS 2 runtime for hand guiding and MoveIt sequence execution.

The web layer never publishes hardware commands. This runtime is the only arm
UI component allowed to switch controllers or submit a MoveIt action.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Sequence

from .models import Routine, SIDES, Waypoint


@dataclass(frozen=True)
class JointSnapshot:
    side: str
    names: tuple[str, ...]
    positions: tuple[float, ...]
    received_at: float


class TrajectoryCancelled(RuntimeError):
    """MoveIt reports PREEMPTED after an operator STOP request."""


class ArmRosRuntime:
    """Owns ROS subscriptions, controller switches, and MoveIt sequence goals."""

    def __init__(
        self,
        *,
        robot_type: str = "fr3",
        state_timeout_sec: float = 1.0,
        service_timeout_sec: float = 5.0,
        action_timeout_sec: float = 300.0,
        status_callback: Callable[[str, str], None] | None = None,
    ) -> None:
        try:
            from action_msgs.srv import CancelGoal
            import rclpy
            from controller_manager_msgs.srv import (
                ConfigureController,
                ListControllers,
                LoadController,
                SwitchController,
            )
            from moveit_msgs.action import MoveGroupSequence
            from moveit_msgs.msg import Constraints, JointConstraint, MotionSequenceItem
            from rclpy.action import ActionClient
            from rclpy.executors import MultiThreadedExecutor
            from sensor_msgs.msg import JointState
        except ImportError as error:
            raise RuntimeError(
                "arm_ui必须在已source ROS 2和工作区的环境中运行"
            ) from error

        self.ros = {
            "rclpy": rclpy,
            "ActionClient": ActionClient,
            "CancelGoal": CancelGoal,
            "ConfigureController": ConfigureController,
            "Constraints": Constraints,
            "JointConstraint": JointConstraint,
            "JointState": JointState,
            "ListControllers": ListControllers,
            "LoadController": LoadController,
            "MotionSequenceItem": MotionSequenceItem,
            "MoveGroupSequence": MoveGroupSequence,
            "MultiThreadedExecutor": MultiThreadedExecutor,
            "SwitchController": SwitchController,
        }
        self.robot_type = str(robot_type)
        self.state_timeout_sec = float(state_timeout_sec)
        self.service_timeout_sec = float(service_timeout_sec)
        self.action_timeout_sec = float(action_timeout_sec)
        self.status_callback = status_callback
        self._snapshots: dict[str, JointSnapshot] = {}
        self._snapshot_lock = threading.RLock()
        self._goal_lock = threading.RLock()
        self._execution_lock = threading.Lock()
        self._stop_requested = threading.Event()
        self._active_goal = None
        self._running_side: str | None = None
        self._controller_mode = {side: "unknown" for side in SIDES}

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("arm_ui")
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self.executor.spin, name="arm-ui-ros", daemon=True
        )

        self._subscriptions = []
        self._manager_clients: dict[str, dict[str, object]] = {}
        for side in SIDES:
            self._subscriptions.append(
                self.node.create_subscription(
                    JointState,
                    f"/{side}/franka/joint_states",
                    lambda message, side=side: self._on_joint_state(side, message),
                    10,
                )
            )
            manager = f"/{side}/controller_manager"
            self._manager_clients[side] = {
                "load": self.node.create_client(
                    LoadController, f"{manager}/load_controller"
                ),
                "configure": self.node.create_client(
                    ConfigureController, f"{manager}/configure_controller"
                ),
                "list": self.node.create_client(
                    ListControllers, f"{manager}/list_controllers"
                ),
                "switch": self.node.create_client(
                    SwitchController, f"{manager}/switch_controller"
                ),
            }
        # Fake hardware uses one root controller manager and a combined state
        # topic; real hardware uses one namespaced manager/topic per arm.
        self._subscriptions.append(
            self.node.create_subscription(
                JointState, "/joint_states", self._on_combined_joint_state, 10
            )
        )
        self._manager_clients["combined"] = {
            "load": self.node.create_client(
                LoadController, "/controller_manager/load_controller"
            ),
            "configure": self.node.create_client(
                ConfigureController, "/controller_manager/configure_controller"
            ),
            "list": self.node.create_client(
                ListControllers, "/controller_manager/list_controllers"
            ),
            "switch": self.node.create_client(
                SwitchController, "/controller_manager/switch_controller"
            ),
        }
        self._trajectory_cancel_clients: dict[str, tuple[object, object]] = {}
        for side in SIDES:
            self._trajectory_cancel_clients[side] = (
                self.node.create_client(
                    CancelGoal,
                    f"/{side}/{side}_arm_controller/"
                    "follow_joint_trajectory/_action/cancel_goal",
                ),
                self.node.create_client(
                    CancelGoal,
                    f"/{side}_arm_controller/"
                    "follow_joint_trajectory/_action/cancel_goal",
                ),
            )

        self.sequence_client = ActionClient(
            self.node, MoveGroupSequence, "/sequence_move_group"
        )
        self._spin_thread.start()

    def _notify(self, side: str, message: str) -> None:
        if self.status_callback is not None:
            self.status_callback(side, message)

    def joint_names(self, side: str) -> tuple[str, ...]:
        return tuple(f"{side}_{self.robot_type}_joint{index}" for index in range(1, 8))

    def _on_joint_state(self, side: str, message: object) -> None:
        names = tuple(str(name) for name in message.name)
        positions = tuple(float(value) for value in message.position)
        index_by_name = {name: index for index, name in enumerate(names)}
        expected = self.joint_names(side)
        if any(name not in index_by_name for name in expected):
            return
        ordered = tuple(positions[index_by_name[name]] for name in expected)
        if not all(math.isfinite(value) for value in ordered):
            return
        with self._snapshot_lock:
            self._snapshots[side] = JointSnapshot(
                side, expected, ordered, time.monotonic()
            )

    def _on_combined_joint_state(self, message: object) -> None:
        for side in SIDES:
            self._on_joint_state(side, message)

    def snapshot(self, side: str, *, require_fresh: bool = True) -> JointSnapshot:
        if side not in SIDES:
            raise ValueError(f"未知机械臂: {side}")
        with self._snapshot_lock:
            snapshot = self._snapshots.get(side)
        if snapshot is None:
            raise RuntimeError(f"尚未收到{side}机械臂关节状态")
        age = time.monotonic() - snapshot.received_at
        if require_fresh and age > self.state_timeout_sec:
            raise RuntimeError(f"{side}机械臂关节状态已过期（{age:.2f}秒）")
        return snapshot

    def controller_mode(self, side: str) -> str:
        return self._controller_mode[side]

    def is_running(self) -> bool:
        with self._goal_lock:
            return self._running_side is not None

    def _call(self, client: object, request: object, operation: str) -> object:
        if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise RuntimeError(f"ROS服务不可用: {operation}")
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(self.service_timeout_sec):
            raise TimeoutError(f"ROS服务超时: {operation}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"ROS服务无响应: {operation}")
        return result

    def _controllers(self, side: str) -> dict[str, object]:
        service = self.ros["ListControllers"]
        clients = self._clients(side)
        response = self._call(
            clients["list"],
            service.Request(),
            f"controller_manager/list_controllers ({side})",
        )
        return {controller.name: controller for controller in response.controller}

    def _graph_controller_mode(self, side: str) -> str:
        controllers = self._controllers(side)
        teach = controllers.get(f"{side}_teach_controller")
        arm = controllers.get(f"{side}_arm_controller")
        if teach is not None and teach.state == "active":
            mode = "teach"
        elif arm is not None and arm.state == "active":
            mode = "trajectory"
        else:
            mode = "inactive"
        self._controller_mode[side] = mode
        return mode

    def _clients(self, side: str) -> dict[str, object]:
        per_arm = self._manager_clients[side]
        if per_arm["list"].wait_for_service(timeout_sec=0.05):
            return per_arm
        combined = self._manager_clients["combined"]
        if combined["list"].wait_for_service(timeout_sec=0.05):
            return combined
        # Preserve the more informative per-arm service name in the eventual
        # timeout when neither stack has started yet.
        return per_arm

    def _ensure_configured(self, side: str, name: str) -> None:
        clients = self._clients(side)
        controllers = self._controllers(side)
        if name not in controllers:
            service = self.ros["LoadController"]
            request = service.Request()
            request.name = name
            response = self._call(
                clients["load"], request, f"load {name}"
            )
            if not response.ok:
                raise RuntimeError(
                    f"无法加载{name}；请确认lychee_teach_controllers已构建"
                )
            controllers = self._controllers(side)
        state = controllers[name].state
        if state == "unconfigured":
            service = self.ros["ConfigureController"]
            request = service.Request()
            request.name = name
            response = self._call(
                clients["configure"],
                request,
                f"configure {name}",
            )
            if not response.ok:
                raise RuntimeError(f"无法配置控制器{name}")
        elif state not in {"inactive", "active"}:
            raise RuntimeError(f"控制器{name}状态异常: {state}")

    def switch_mode(self, side: str, mode: str) -> None:
        if side not in SIDES or mode not in {"teach", "trajectory"}:
            raise ValueError("无效的机械臂或控制模式")
        if self.is_running():
            raise RuntimeError("轨迹正在执行，不能切换控制器")
        other_side = "right" if side == "left" else "left"
        if mode == "teach" and self._graph_controller_mode(other_side) == "teach":
            raise RuntimeError(
                f"{other_side}机械臂仍在拖动模式，请先让它退出拖动并保持"
            )
        self._switch_mode_unchecked(side, mode)

    def _switch_mode_unchecked(self, side: str, mode: str) -> None:
        wanted = f"{side}_{'teach' if mode == 'teach' else 'arm'}_controller"
        conflicting = (
            f"{side}_arm_controller" if mode == "teach" else f"{side}_teach_controller"
        )
        self._ensure_configured(side, wanted)
        clients = self._clients(side)
        controllers = self._controllers(side)
        activate = [] if controllers[wanted].state == "active" else [wanted]
        deactivate = (
            [conflicting]
            if conflicting in controllers and controllers[conflicting].state == "active"
            else []
        )
        if activate or deactivate:
            service = self.ros["SwitchController"]
            request = service.Request()
            request.activate_controllers = activate
            request.deactivate_controllers = deactivate
            request.strictness = service.Request.STRICT
            request.activate_asap = True
            request.timeout.sec = int(self.service_timeout_sec)
            response = self._call(
                clients["switch"],
                request,
                f"switch {side} to {mode}",
            )
            if not response.ok:
                raise RuntimeError(f"{side}机械臂切换到{mode}模式失败")
        self._controller_mode[side] = mode
        self._notify(side, "拖动模式" if mode == "teach" else "保持模式")

    def execute(self, routine: Routine, waypoints: Sequence[Waypoint]) -> None:
        if not waypoints:
            raise ValueError("任务中没有点位")
        if any(point.side != routine.side for point in waypoints):
            raise ValueError("任务包含另一条机械臂的点位")
        if not self._execution_lock.acquire(blocking=False):
            raise RuntimeError("已有轨迹正在执行")
        side = routine.side
        self._stop_requested.clear()
        with self._goal_lock:
            self._running_side = side
        try:
            other_side = "right" if side == "left" else "left"
            if self._graph_controller_mode(other_side) == "teach":
                raise RuntimeError(
                    f"{other_side}机械臂仍在拖动模式，不能开始轨迹"
                )
            _ = self.snapshot(side)
            self._switch_mode_unchecked(side, "trajectory")
            if self._stop_requested.is_set():
                raise TrajectoryCancelled("轨迹已由操作员停止")
            # Re-read after switching so planning starts from the held real state.
            start = self.snapshot(side)
            goal = self._make_sequence_goal(routine, waypoints, start)
            if not self.sequence_client.wait_for_server(
                timeout_sec=self.service_timeout_sec
            ):
                raise RuntimeError("MoveIt sequence_move_group action不可用")

            self._notify(side, "规划中")
            future = self.sequence_client.send_goal_async(goal)
            event = threading.Event()
            future.add_done_callback(lambda _future: event.set())
            if not event.wait(self.service_timeout_sec):
                raise TimeoutError("MoveIt轨迹请求超时")
            handle = future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError("MoveIt拒绝了轨迹任务")
            with self._goal_lock:
                self._active_goal = handle
            self._notify(side, "运行中")
            if self._stop_requested.is_set():
                self.stop()
            result_future = handle.get_result_async()
            result_event = threading.Event()
            result_future.add_done_callback(lambda _future: result_event.set())
            if not result_event.wait(self.action_timeout_sec):
                self.stop()
                raise TimeoutError("轨迹执行超时，已请求停止")
            wrapped = result_future.result()
            if wrapped is None:
                raise RuntimeError("MoveIt轨迹执行无结果")
            error_code = wrapped.result.response.error_code.val
            if int(error_code) == -4:
                raise TrajectoryCancelled("轨迹已由操作员停止")
            if int(error_code) != 1:
                raise RuntimeError(f"MoveIt轨迹失败，错误码: {error_code}")
            self._controller_mode[side] = "trajectory"
            self._notify(side, "完成并保持末点")
        except TrajectoryCancelled:
            self._notify(side, "已停止并保持当前位置")
            raise
        except Exception as error:
            self._notify(side, f"失败：{error}")
            raise
        finally:
            with self._goal_lock:
                self._active_goal = None
                self._running_side = None
            self._stop_requested.clear()
            self._execution_lock.release()

    def _make_sequence_goal(
        self,
        routine: Routine,
        waypoints: Sequence[Waypoint],
        start: JointSnapshot,
    ) -> object:
        goal = self.ros["MoveGroupSequence"].Goal()
        for index, point in enumerate(waypoints):
            item = self.ros["MotionSequenceItem"]()
            item.req.group_name = f"{routine.side}_arm"
            item.req.pipeline_id = "pilz_industrial_motion_planner"
            item.req.planner_id = "PTP"
            item.req.allowed_planning_time = 5.0
            item.req.num_planning_attempts = 1
            item.req.max_velocity_scaling_factor = routine.velocity_scale
            item.req.max_acceleration_scaling_factor = routine.acceleration_scale
            constraints = self.ros["Constraints"]()
            constraints.name = point.name
            for joint_name, position in zip(start.names, point.joints, strict=True):
                constraint = self.ros["JointConstraint"]()
                constraint.joint_name = joint_name
                constraint.position = position
                constraint.tolerance_above = 1e-4
                constraint.tolerance_below = 1e-4
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)
            item.req.goal_constraints = [constraints]
            if index == 0:
                item.req.start_state.is_diff = True
                item.req.start_state.joint_state.name = list(start.names)
                item.req.start_state.joint_state.position = list(start.positions)
            item.blend_radius = (
                routine.blend_radius_m if index < len(waypoints) - 1 else 0.0
            )
            goal.request.items.append(item)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        return goal

    def stop(self) -> bool:
        with self._goal_lock:
            handle = self._active_goal
            side = self._running_side
        if side is None:
            return False
        self._stop_requested.set()
        if handle is None:
            self._notify(side, "将在控制器切换完成后停止并保持")
            return True
        # Ask the top-level planner to unwind, but do not rely on it for timely
        # stopping: Pilz may not answer this request until execution returns.
        _ = handle.cancel_goal_async()

        cancel_client = self._trajectory_cancel_client(side)
        request = self.ros["CancelGoal"].Request()
        # A zero goal ID and zero timestamp cancels every goal on this action
        # server, which is safe because each arm controller has one owner in
        # MoveIt mode.
        response = self._call(
            cancel_client,
            request,
            f"cancel {side} FollowJointTrajectory",
        )
        delivered = response is not None
        if delivered:
            self._notify(side, "正在停止并保持")
        return delivered

    def _trajectory_cancel_client(self, side: str) -> object:
        namespaced, combined = self._trajectory_cancel_clients[side]
        if namespaced.wait_for_service(timeout_sec=0.05):
            return namespaced
        if combined.wait_for_service(timeout_sec=0.05):
            return combined
        return namespaced

    def shutdown(self) -> None:
        self.executor.shutdown(timeout_sec=2.0)
        self._spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        if self.ros["rclpy"].ok():
            self.ros["rclpy"].shutdown()
