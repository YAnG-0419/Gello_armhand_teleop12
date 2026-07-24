#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from franka_msgs.srv import SetFullCollisionBehavior


class CollisionBehaviorSetter(Node):
    def __init__(self):
        super().__init__('collision_behavior_setter')
        self.left_client = self.create_client(
            SetFullCollisionBehavior,
            '/left/service_server/set_full_collision_behavior')
        self.right_client = self.create_client(
            SetFullCollisionBehavior,
            '/right/service_server/set_full_collision_behavior')
        self.set_behaviors()

    def call_service(self, client, arm_name):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{arm_name} collision-behavior service is unavailable.')

        request = SetFullCollisionBehavior.Request()
        request.lower_torque_thresholds_nominal = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0]
        request.upper_torque_thresholds_nominal = [60.0, 60.0, 50.0, 50.0, 45.0, 40.0, 35.0]
        request.lower_force_thresholds_nominal = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
        request.upper_force_thresholds_nominal = [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]

        self.get_logger().info(f'Setting collision thresholds for the {arm_name} arm.')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Setting {arm_name} collision thresholds failed.')

    def set_behaviors(self):
        self.call_service(self.left_client, 'left')
        self.call_service(self.right_client, 'right')


def main():
    rclpy.init()
    node = CollisionBehaviorSetter()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
