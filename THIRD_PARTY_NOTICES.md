# Third-party notices

The realtime FR3 controller is adapted from Franka Robotics ROS 2 controller code and retains its Apache-2.0 license and NOTICE files in `ros_ws/src/franka_fr3_arm_controllers`.

The MuJoCo FR3 model includes Franka Robotics description assets. Their license and notice are retained in `teleop_sources/pico/assets/dual_fr3/FRANKA_DESCRIPTION_LICENSE` and `FRANKA_DESCRIPTION_NOTICE`.

The PICO adapter includes the XRoboToolkit PC Service Python binding and its prebuilt Linux SDK library under `third_party/xrobotoolkit_sdk`. Its upstream license is retained alongside the source. The included native library targets x86-64 Linux; rebuild it from upstream for another architecture.

The MANUS hand adapter includes MANUS CoreSDK 3.1.1 headers and its integrated x86-64 Linux library under `third_party/manus_sdk`. These proprietary files remain governed by the vendor agreement retained as `LICENSE.vendor`. The adapter source adds the attribution required for derivatives of vendor sample code.

The hand retargeting assets under `assets/linkerhand_l20` are LinkerHand L20 URDFs and meshes distributed by the vendor under Apache-2.0. Their upstream license is retained alongside the models.

The hand retargeting optimizer in `teleop_sources/pico/src/pico_bimanual_franka_teleop/hand_retarget.py` and the G20 projection and slew limiter in `ros_ws/src/linker_hand_bridge/linker_hand_bridge/core.py` are adapted from the sibling WiLoR repository, which drove the same hand from monocular MANO reconstructions. They were copied rather than imported because sibling repositories are read-only references.

The container downloads pinned upstream releases of libfranka, franka_ros2, and franka_description during the build. Those components remain under their respective upstream licenses.
