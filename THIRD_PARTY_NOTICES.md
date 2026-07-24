# Third-party notices

The realtime FR3 controller is adapted from Franka Robotics ROS 2 controller
code and retains its Apache-2.0 license and NOTICE files in
`ros_ws/src/franka_fr3_arm_controllers`.

The MuJoCo FR3 model includes Franka Robotics description assets. Their
license and notice are retained in
`teleop_sources/pico/assets/dual_fr3/FRANKA_DESCRIPTION_LICENSE` and
`FRANKA_DESCRIPTION_NOTICE`.

The PICO adapter includes the XRoboToolkit PC Service Python binding and its
prebuilt Linux SDK library under `third_party/xrobotoolkit_sdk`. Its upstream
license is retained alongside the source. The included native library targets
x86-64 Linux; rebuild it from upstream for another architecture.

The container downloads pinned upstream releases of libfranka,
franka_ros2, and franka_description during the build. Those components remain
under their respective upstream licenses.
