import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    robot_config = LaunchConfiguration("robot_config")
    set_collision_behavior = LaunchConfiguration('set_collision_behavior')
    ld.add_action(DeclareLaunchArgument(
        "robot_config",
        description="Absolute dual-FR3 workcell configuration path.",
    ))
    ld.add_action(DeclareLaunchArgument(
        'set_collision_behavior', default_value='false',
        description='Explicitly allow automatic collision-threshold changes.'))

    # Core hardware controllers: per-arm ros2_control at 1 kHz.
    franka_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('franka_fr3_arm_controllers'),
                'launch',
                'franka_fr3_arm_controllers.launch.py'
            )
        ),
        launch_arguments={"robot_config_file": robot_config}.items(),
    )

    set_bi_collision_behavior_node = Node(
        package='franka_fr3_arm_controllers',
        executable='set_bi_collision_behavior.py',
        name='set_bi_collision_behavior_node',
        output='screen',
        condition=IfCondition(set_collision_behavior)
    )

    ld.add_action(franka_controllers_launch)
    ld.add_action(set_bi_collision_behavior_node)
    return ld
