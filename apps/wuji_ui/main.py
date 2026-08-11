"""NiceGUI entry point for dual Wuji Hand 2 pose teaching."""

from __future__ import annotations

import argparse
import asyncio
import math
from pathlib import Path

from nicegui import run, ui

from .models import (
    FINGERS,
    HandPose,
    ManusTrigger,
    SIDES,
    WujiPoseRepository,
    summarize_gap_samples,
)
from .runtime import WujiUiRuntime


SIDE_LABELS = {"left": "左手", "right": "右手"}
FINGER_LABELS = {
    "index": "拇指—食指",
    "middle": "拇指—中指",
    "ring": "拇指—无名指",
    "pinky": "拇指—小指",
}
MODE_LABELS = {
    "manual": "手动（电机失能）",
    "teleop": "MANUS 遥操",
    "pose": "移动到保存姿态",
    "hold": "保持姿态",
}


def _degrees(value: float) -> float:
    return round(math.degrees(float(value)), 5)


def _radians(value: object) -> float:
    return math.radians(float(value))


class WujiUiApplication:
    def __init__(self, repository: WujiPoseRepository, runtime: WujiUiRuntime) -> None:
        self.repository = repository
        self.runtime = runtime

    def build_page(self) -> None:
        states = {
            side: {"selected": None, "busy": False, "fields": [], "labels": []}
            for side in SIDES
        }
        status_labels = {}
        mode_labels = {}
        name_inputs = {}
        pose_refreshers = {}

        def _select_pose(side: str, pose: HandPose, refresh) -> None:
            states[side]["selected"] = pose.name
            name_inputs[side].value = pose.name
            for field, value in zip(
                states[side]["fields"], pose.qpos, strict=True
            ):
                field.value = _degrees(value)
            refresh()

        ui.add_head_html(
            """
            <style>
              body { background: #f4f6f8; }
              .wuji-card { border: 1px solid #d8e0e7; border-radius: 12px; }
              .joint-grid { display: grid; grid-template-columns: 180px 95px 340px;
                            gap: 7px 12px; align-items: center; }
              .joint-value input { text-align: right; font-family: monospace; }
              .joint-editor { display: flex; align-items: center; gap: 8px; }
              .status-pill { border-radius: 999px; padding: 5px 12px; }
            </style>
            """
        )
        with ui.header().classes("items-center bg-slate-800"):
            ui.label("Wuji Hand 2 双手姿态记录").classes("text-xl font-semibold")
            ui.space()
            ui.label("实际反馈 · rad 存储 · device joint order").classes(
                "text-sm text-slate-300"
            )
            disable_all_button = ui.button(
                "双手失能", icon="power_settings_new", color="negative"
            )

        with ui.column().classes("w-full max-w-[1780px] mx-auto p-5 gap-5"):
            ui.label(
                "手动模式下电机失能，可直接摆手；遥操和姿态回放会使能电机。"
                "记录始终读取电机实际位置。"
            ).classes("rounded-lg bg-blue-50 text-blue-900 p-3 w-full")
            with ui.row().classes("w-full items-start gap-5 flex-nowrap"):
                for side in SIDES:
                    with ui.card().classes("wuji-card grow basis-1/2 p-5"):
                        with ui.row().classes("w-full items-center"):
                            ui.label(SIDE_LABELS[side]).classes("text-xl font-semibold")
                            status_labels[side] = ui.label("等待反馈").classes(
                                "status-pill bg-amber-100 text-amber-900"
                            )
                            ui.space()
                            mode_labels[side] = ui.label("手动（电机失能）")
                        with ui.row().classes("w-full gap-2"):
                            manual_button = ui.button(
                                "手动 / 失能", icon="pan_tool"
                            ).props("outline")
                            teleop_button = ui.button(
                                "启动 MANUS 遥操", icon="back_hand", color="positive"
                            )

                        ui.separator()
                        with ui.row().classes("w-full items-end"):
                            name_inputs[side] = ui.input("姿态名称").classes("grow")
                            record_button = ui.button(
                                "读取当前位置", icon="add_location"
                            )
                            save_button = ui.button("保存", icon="save", color="primary")

                        with ui.element("div").classes("joint-grid w-full"):
                            ui.label("关节").classes("font-medium")
                            ui.label("当前角度").classes("font-medium text-right")
                            ui.label("待保存角度").classes("font-medium text-right")
                            for joint_name, (lower, upper) in zip(
                                self.runtime.joint_names[side],
                                self.runtime.joint_limits[side],
                                strict=True,
                            ):
                                ui.label(joint_name).classes("font-mono text-xs")
                                current_label = ui.label("--").classes(
                                    "font-mono text-right"
                                )
                                with ui.row().classes(
                                    "joint-editor w-full flex-nowrap"
                                ):
                                    field = ui.number(
                                        value=0.0,
                                        min=_degrees(lower),
                                        max=_degrees(upper),
                                        step=0.01,
                                        format="%.5f",
                                    ).props(
                                        "dense outlined suffix=°"
                                    ).classes("joint-value w-[105px] shrink-0")
                                    ui.slider(
                                        min=_degrees(lower),
                                        max=_degrees(upper),
                                        step=0.1,
                                        value=0.0,
                                    ).props("label").classes("grow").bind_value(field)
                                states[side]["labels"].append(current_label)
                                states[side]["fields"].append(field)

                        ui.separator()
                        with ui.row().classes("w-full items-center"):
                            ui.label("已保存姿态").classes("text-lg font-semibold")
                            ui.space()
                            move_button = ui.button(
                                "移动到选中姿态", icon="play_arrow", color="warning"
                            )
                            delete_button = ui.button(
                                "删除", icon="delete", color="negative"
                            ).props("flat")

                        @ui.refreshable
                        def pose_list(side: str = side) -> None:
                            poses = self.repository.poses(side)
                            if not poses:
                                ui.label("尚未保存姿态").classes(
                                    "text-slate-400 py-3"
                                )
                                return
                            with ui.list().props("bordered separator").classes("w-full"):
                                for pose in poses:
                                    selected = states[side]["selected"] == pose.name
                                    with ui.item(
                                        on_click=lambda _event, side=side, pose=pose,
                                        refreshers=pose_refreshers: _select_pose(
                                            side, pose, refreshers[side]
                                        )
                                    ).classes(
                                        "cursor-pointer "
                                        + ("bg-blue-50" if selected else "")
                                    ):
                                        with ui.item_section():
                                            ui.item_label(pose.name)
                                            ui.item_label(
                                                ", ".join(
                                                    f"{_degrees(value):.1f}°"
                                                    for value in pose.qpos
                                                )
                                            ).props("caption lines=2").classes("font-mono")

                        pose_list()
                        refresh_pose_list = pose_list.refresh
                        pose_refreshers[side] = refresh_pose_list

                        async def _manual(side: str = side) -> None:
                            if states[side]["busy"]:
                                return
                            states[side]["busy"] = True
                            try:
                                await run.io_bound(self.runtime.set_manual, side)
                                ui.notify(f"{SIDE_LABELS[side]}电机已失能，可手动摆姿态")
                            except Exception as error:
                                ui.notify(str(error), color="negative", timeout=8)
                            finally:
                                states[side]["busy"] = False

                        async def _teleop(side: str = side) -> None:
                            if states[side]["busy"]:
                                return
                            states[side]["busy"] = True
                            try:
                                await run.io_bound(self.runtime.start_teleop, side)
                                ui.notify(
                                    f"{SIDE_LABELS[side]} MANUS遥操已启动",
                                    color="positive",
                                )
                            except Exception as error:
                                ui.notify(str(error), color="negative", timeout=8)
                            finally:
                                states[side]["busy"] = False

                        def _record(
                            side: str = side, refresh=refresh_pose_list
                        ) -> None:
                            try:
                                snapshot = self.runtime.snapshot(side)
                                if not name_inputs[side].value:
                                    name_inputs[side].value = (
                                        f"pose_{len(self.repository.poses(side)) + 1}"
                                    )
                                states[side]["selected"] = None
                                for field, value in zip(
                                    states[side]["fields"], snapshot.qpos, strict=True
                                ):
                                    field.value = _degrees(value)
                                refresh()
                                ui.notify("已读取实际关节反馈；请确认名称后保存")
                            except Exception as error:
                                ui.notify(str(error), color="negative", timeout=8)

                        def _save(
                            side: str = side, refresh=refresh_pose_list
                        ) -> None:
                            try:
                                pose = HandPose.create(
                                    name_inputs[side].value,
                                    side,
                                    [_radians(field.value) for field in states[side]["fields"]],
                                )
                                self.repository.save(
                                    pose, previous_name=states[side]["selected"]
                                )
                                states[side]["selected"] = pose.name
                                refresh()
                                ui.notify(f"姿态“{pose.name}”已保存", color="positive")
                            except Exception as error:
                                ui.notify(str(error), color="negative", timeout=8)

                        def _delete(
                            side: str = side, refresh=refresh_pose_list
                        ) -> None:
                            try:
                                name = states[side]["selected"]
                                if not name:
                                    raise ValueError("请先选中要删除的姿态")
                                self.repository.delete(side, name)
                                states[side]["selected"] = None
                                name_inputs[side].value = ""
                                refresh()
                                ui.notify(f"姿态“{name}”已删除")
                            except Exception as error:
                                ui.notify(str(error), color="negative")

                        async def _move(side: str = side) -> None:
                            if states[side]["busy"]:
                                return
                            try:
                                name = states[side]["selected"]
                                if not name:
                                    raise ValueError("请先选中要移动到的姿态")
                                pose = self.repository.pose(side, name)
                                states[side]["busy"] = True
                                await run.io_bound(
                                    self.runtime.move_to_pose, side, pose.qpos
                                )
                                ui.notify(
                                    f"{SIDE_LABELS[side]}开始平滑移动到“{name}”",
                                    color="warning",
                                )
                            except Exception as error:
                                ui.notify(str(error), color="negative", timeout=8)
                            finally:
                                states[side]["busy"] = False

                        with ui.dialog() as teleop_dialog, ui.card().classes("max-w-lg"):
                            ui.label(f"确认启动{SIDE_LABELS[side]} MANUS遥操").classes(
                                "text-lg font-semibold"
                            )
                            ui.label("电机将使能并跟随手套。请确认手周围无障碍物。")
                            with ui.row().classes("w-full justify-end"):
                                ui.button("取消", on_click=teleop_dialog.close).props("flat")
                                ui.button(
                                    "确认启动",
                                    color="positive",
                                    on_click=lambda dialog=teleop_dialog, action=_teleop: (
                                        dialog.close(), asyncio.create_task(action())
                                    ),
                                )

                        with ui.dialog() as move_dialog, ui.card().classes("max-w-lg"):
                            ui.label(f"确认移动{SIDE_LABELS[side]}").classes(
                                "text-lg font-semibold"
                            )
                            ui.label(
                                "电机将使能并以受限速度移动到选中姿态。"
                                "请确认手周围无障碍物。"
                            )
                            with ui.row().classes("w-full justify-end"):
                                ui.button("取消", on_click=move_dialog.close).props("flat")
                                ui.button(
                                    "确认移动",
                                    color="warning",
                                    on_click=lambda dialog=move_dialog, action=_move: (
                                        dialog.close(), asyncio.create_task(action())
                                    ),
                                )

                        manual_button.on(
                            "click", lambda action=_manual: asyncio.create_task(action())
                        )
                        teleop_button.on("click", teleop_dialog.open)
                        record_button.on("click", _record)
                        save_button.on("click", _save)
                        delete_button.on("click", _delete)
                        move_button.on("click", move_dialog.open)

            trigger_state = {
                "sampling": False,
                "samples": [],
                "last_sequence": None,
                "stats": None,
                "selected": None,
            }
            with ui.card().classes("wuji-card w-full p-5"):
                with ui.row().classes("w-full items-center"):
                    ui.label("MANUS 距离阈值采集").classes("text-xl font-semibold")
                    ui.label(
                        "采集意图触发时的拇指尖—目标指尖距离，并映射到已保存姿态。"
                    ).classes("text-sm text-slate-500")
                with ui.row().classes("w-full items-end gap-3"):
                    trigger_side = ui.toggle(SIDE_LABELS, value="left").props(
                        "no-caps"
                    )
                    trigger_finger = ui.select(
                        FINGER_LABELS, value="index", label="距离类型"
                    ).classes("w-48")
                    trigger_pose = ui.select(
                        [pose.name for pose in self.repository.poses("left")],
                        label="对应保存姿态",
                    ).classes("w-56")
                    trigger_name = ui.input("触发器名称").classes("grow")
                    start_capture = ui.button(
                        "开始采集", icon="fiber_manual_record", color="positive"
                    )
                    stop_capture = ui.button("停止并计算", icon="stop")
                    clear_capture = ui.button("清空", icon="restart_alt").props("flat")

                with ui.row().classes("w-full items-center gap-5"):
                    live_gap = ui.label("实时距离：-- mm").classes(
                        "text-lg font-mono"
                    )
                    capture_status = ui.label("尚未采集").classes(
                        "rounded bg-slate-100 px-3 py-2"
                    )
                    sample_stats = ui.label(
                        "min / p05 / median / p95 / max：--"
                    ).classes("font-mono text-sm")

                with ui.row().classes("w-full items-end gap-3"):
                    enter_threshold = ui.number(
                        "进入阈值",
                        value=25.0,
                        min=0.1,
                        max=300.0,
                        step=0.1,
                        suffix="mm",
                        format="%.2f",
                    ).classes("w-40")
                    exit_threshold = ui.number(
                        "退出阈值",
                        value=32.0,
                        min=0.1,
                        max=300.0,
                        step=0.1,
                        suffix="mm",
                        format="%.2f",
                    ).classes("w-40")
                    dwell_seconds = ui.number(
                        "停留时间",
                        value=0.15,
                        min=0.01,
                        max=2.0,
                        step=0.01,
                        suffix="s",
                        format="%.2f",
                    ).classes("w-40")
                    ui.space()
                    save_trigger = ui.button(
                        "保存阈值映射", icon="save", color="primary"
                    )
                    delete_trigger = ui.button(
                        "删除触发器", icon="delete", color="negative"
                    ).props("flat")

                @ui.refreshable
                def trigger_list() -> None:
                    triggers = self.repository.triggers(trigger_side.value)
                    if not triggers:
                        ui.label("当前手侧尚未保存阈值映射").classes(
                            "text-slate-400 py-2"
                        )
                        return
                    with ui.list().props("bordered separator").classes("w-full"):
                        for trigger in triggers:
                            selected = trigger_state["selected"] == trigger.name
                            with ui.item(
                                on_click=lambda _event, trigger=trigger: _load_trigger(
                                    trigger
                                )
                            ).classes(
                                "cursor-pointer "
                                + ("bg-blue-50" if selected else "")
                            ):
                                with ui.item_section():
                                    ui.item_label(trigger.name)
                                    ui.item_label(
                                        f"{FINGER_LABELS[trigger.finger]} → "
                                        f"{trigger.pose_name}｜进入≤"
                                        f"{trigger.enter_max_m * 1000.0:.1f} mm｜"
                                        f"退出≥{trigger.exit_min_m * 1000.0:.1f} mm｜"
                                        f"dwell {trigger.dwell_seconds:.2f} s"
                                    ).props("caption")

                trigger_list()

                def _reset_capture() -> None:
                    trigger_state["sampling"] = False
                    trigger_state["samples"] = []
                    trigger_state["last_sequence"] = None
                    trigger_state["stats"] = None
                    capture_status.text = "尚未采集"
                    sample_stats.text = "min / p05 / median / p95 / max：--"

                def _update_trigger_pose_options() -> None:
                    side = trigger_side.value
                    options = [pose.name for pose in self.repository.poses(side)]
                    if list(trigger_pose.options) == options:
                        return
                    trigger_pose.options = options
                    if trigger_pose.value not in options:
                        trigger_pose.value = options[0] if options else None
                    trigger_pose.update()

                def _start_capture() -> None:
                    try:
                        snapshot = self.runtime.manus_gaps(trigger_side.value)
                        finger = trigger_finger.value
                        if finger not in FINGERS:
                            raise ValueError("请选择要采集的目标手指")
                        trigger_state["samples"] = []
                        trigger_state["last_sequence"] = snapshot.sequence
                        trigger_state["stats"] = None
                        trigger_state["sampling"] = True
                        capture_status.text = (
                            f"正在采集{SIDE_LABELS[trigger_side.value]}"
                            f"{FINGER_LABELS[finger]}，请重复目标动作…"
                        )
                    except Exception as error:
                        ui.notify(str(error), color="negative", timeout=8)

                def _stop_capture() -> None:
                    trigger_state["sampling"] = False
                    try:
                        stats = summarize_gap_samples(trigger_state["samples"])
                        trigger_state["stats"] = stats
                        enter_threshold.value = round(
                            float(stats["recommended_enter_max_m"]) * 1000.0, 2
                        )
                        exit_threshold.value = round(
                            float(stats["recommended_exit_min_m"]) * 1000.0, 2
                        )
                        sample_stats.text = (
                            "min / p05 / median / p95 / max："
                            f"{float(stats['min_m']) * 1000.0:.1f} / "
                            f"{float(stats['p05_m']) * 1000.0:.1f} / "
                            f"{float(stats['median_m']) * 1000.0:.1f} / "
                            f"{float(stats['p95_m']) * 1000.0:.1f} / "
                            f"{float(stats['max_m']) * 1000.0:.1f} mm"
                        )
                        capture_status.text = (
                            f"采集完成：{stats['sample_count']}个独立帧；"
                            "推荐值可手动修改"
                        )
                        if not trigger_name.value and trigger_pose.value:
                            trigger_name.value = (
                                f"{trigger_finger.value}_{trigger_pose.value}"
                            )
                    except Exception as error:
                        capture_status.text = str(error)
                        ui.notify(str(error), color="negative")

                def _save_trigger() -> None:
                    try:
                        if trigger_state["stats"] is None:
                            raise ValueError("请先完成一次MANUS阈值采集")
                        trigger = ManusTrigger.create(
                            trigger_name.value,
                            trigger_side.value,
                            trigger_finger.value,
                            trigger_pose.value,
                            enter_max_m=float(enter_threshold.value) / 1000.0,
                            exit_min_m=float(exit_threshold.value) / 1000.0,
                            dwell_seconds=float(dwell_seconds.value),
                            calibration=trigger_state["stats"],
                        )
                        self.repository.save_trigger(
                            trigger, previous_name=trigger_state["selected"]
                        )
                        trigger_state["selected"] = trigger.name
                        trigger_list.refresh()
                        ui.notify(
                            f"阈值“{trigger.name}”已映射到姿态“"
                            f"{trigger.pose_name}”",
                            color="positive",
                        )
                    except Exception as error:
                        ui.notify(str(error), color="negative", timeout=8)

                def _load_trigger(trigger: ManusTrigger) -> None:
                    trigger_state["sampling"] = False
                    trigger_state["selected"] = trigger.name
                    trigger_side.value = trigger.side
                    _update_trigger_pose_options()
                    trigger_finger.value = trigger.finger
                    trigger_pose.value = trigger.pose_name
                    trigger_name.value = trigger.name
                    enter_threshold.value = trigger.enter_max_m * 1000.0
                    exit_threshold.value = trigger.exit_min_m * 1000.0
                    dwell_seconds.value = trigger.dwell_seconds
                    trigger_state["stats"] = dict(trigger.calibration)
                    trigger_list.refresh()

                def _delete_trigger() -> None:
                    try:
                        name = trigger_state["selected"]
                        if not name:
                            raise ValueError("请先选择要删除的触发器")
                        self.repository.delete_trigger(trigger_side.value, name)
                        trigger_state["selected"] = None
                        trigger_name.value = ""
                        trigger_list.refresh()
                        ui.notify(f"触发器“{name}”已删除")
                    except Exception as error:
                        ui.notify(str(error), color="negative")

                def _change_trigger_side() -> None:
                    _reset_capture()
                    trigger_state["selected"] = None
                    _update_trigger_pose_options()
                    trigger_list.refresh()

                def _refresh_manus_threshold() -> None:
                    _update_trigger_pose_options()
                    try:
                        snapshot = self.runtime.manus_gaps(trigger_side.value)
                        finger = trigger_finger.value
                        gap_m = float(snapshot.gaps_m[finger])
                        live_gap.text = f"实时距离：{gap_m * 1000.0:.2f} mm"
                        if (
                            trigger_state["sampling"]
                            and snapshot.sequence != trigger_state["last_sequence"]
                        ):
                            trigger_state["last_sequence"] = snapshot.sequence
                            trigger_state["samples"].append(gap_m)
                            capture_status.text = (
                                f"正在采集：{len(trigger_state['samples'])}个独立帧"
                            )
                    except Exception as error:
                        live_gap.text = f"实时距离：{error}"

                trigger_side.on_value_change(lambda _event: _change_trigger_side())
                trigger_finger.on_value_change(lambda _event: _reset_capture())
                start_capture.on("click", _start_capture)
                stop_capture.on("click", _stop_capture)
                clear_capture.on("click", _reset_capture)
                save_trigger.on("click", _save_trigger)
                delete_trigger.on("click", _delete_trigger)
                _update_trigger_pose_options()
                ui.timer(0.1, _refresh_manus_threshold)

            with ui.row().classes("w-full items-center"):
                ui.label(f"保存文件：{self.repository.path}").classes(
                    "text-sm text-slate-500"
                )
                ui.space()
                ui.button(
                    "下载 JSON",
                    icon="download",
                    on_click=lambda: ui.download(self.repository.path)
                    if self.repository.path.exists()
                    else ui.notify("尚未保存姿态", color="warning"),
                ).props("flat")

        def _refresh_feedback() -> None:
            for side in SIDES:
                mode_labels[side].text = f"模式：{MODE_LABELS[self.runtime.mode(side)]}"
                try:
                    snapshot = self.runtime.snapshot(side)
                    for label, value in zip(
                        states[side]["labels"], snapshot.qpos, strict=True
                    ):
                        label.text = f"{_degrees(value):.2f}°"
                    detail = self.runtime.error(side)
                    if detail:
                        status_labels[side].text = detail
                        status_labels[side].classes(
                            remove=(
                                "bg-amber-100 text-amber-900 "
                                "bg-green-100 text-green-900"
                            ),
                            add="bg-red-100 text-red-900",
                        )
                    else:
                        status_labels[side].text = "反馈正常"
                        status_labels[side].classes(
                            remove=(
                                "bg-amber-100 text-amber-900 "
                                "bg-red-100 text-red-900"
                            ),
                            add="bg-green-100 text-green-900",
                        )
                except Exception as error:
                    detail = self.runtime.error(side) or str(error)
                    status_labels[side].text = detail
                    status_labels[side].classes(
                        remove="bg-amber-100 text-amber-900 bg-green-100 text-green-900",
                        add="bg-red-100 text-red-900",
                    )

        async def _disable_all() -> None:
            try:
                for side in SIDES:
                    await run.io_bound(self.runtime.set_manual, side)
                ui.notify("左右手电机均已失能", color="positive")
            except Exception as error:
                ui.notify(str(error), color="negative", timeout=8)

        disable_all_button.on(
            "click", lambda: asyncio.create_task(_disable_all())
        )
        ui.timer(0.2, _refresh_feedback)


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wuji-left-address", required=True, metavar="IP:PORT")
    parser.add_argument("--wuji-right-address", required=True, metavar="IP:PORT")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("config/calibration/wuji_hand_2_poses.json"),
    )
    parser.add_argument("--wuji-kp", type=float, default=4.0)
    parser.add_argument("--wuji-kd", type=float, default=0.1)
    parser.add_argument("--wuji-current-limit", type=float, default=1.0)
    parser.add_argument("--wuji-rate", type=float, default=30.0)
    parser.add_argument("--wuji-stale-timeout", type=float, default=0.25)
    parser.add_argument("--pose-speed", type=float, default=1.0, metavar="RAD_S")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args(argv)
    if args.port <= 0 or args.port > 65535:
        parser.error("--port必须在1..65535")
    if args.wuji_kp < 0.0 or args.wuji_kd < 0.0:
        parser.error("--wuji-kp和--wuji-kd不能为负")
    if args.wuji_current_limit <= 0.0 or args.pose_speed <= 0.0:
        parser.error("电流限制和姿态速度必须为正数")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    runtime = WujiUiRuntime(
        addresses={
            "left": args.wuji_left_address,
            "right": args.wuji_right_address,
        },
        kp=args.wuji_kp,
        kd=args.wuji_kd,
        current_limit=args.wuji_current_limit,
        rate=args.wuji_rate,
        stale_timeout=args.wuji_stale_timeout,
        pose_speed_rad_s=args.pose_speed,
    )
    repository = WujiPoseRepository(args.output, runtime.joint_names)
    application = WujiUiApplication(repository, runtime)
    ui.page("/")(application.build_page)
    try:
        ui.run(
            host=args.host,
            port=args.port,
            title="Wuji Hand 2 Pose UI",
            reload=False,
            show=not args.no_browser,
        )
    finally:
        runtime.close()
    return 0


if __name__ in {"__main__", "__mp_main__"}:
    raise SystemExit(main())
