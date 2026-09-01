from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[4]


def test_deployment_config_declares_three_serial_identity_and_semantic_slots():
    path = REPO_ROOT / "data_collection/config/cameras.yaml"
    cameras = yaml.safe_load(path.read_text(encoding="utf-8"))["camera_bringup"]
    assert {"cam0", "cam1", "cam2"} <= set(cameras)
    for name in ("cam0", "cam1", "cam2"):
        assert set(cameras[name]) >= {
            "serial_number",
            "semantic",
            "config_file",
            "start_delay_sec",
        }


def test_build_and_runtime_each_require_explicit_sdk_license_acceptance():
    cmake = (REPO_ROOT / "ros_ws/src/orbbec_camera/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    launch = (
        REPO_ROOT
        / "ros_ws/src/teleop_camera_bringup/launch/triple_camera.launch.py"
    ).read_text(encoding="utf-8")
    assert "LIA_ENABLE_LICENSE_GATED_ORBBEC_RUNTIME" in cmake
    assert "ORBBEC_SDK_LICENSE_ACCEPTED" in launch
    assert "!= \"YES\"" in launch


def test_original_license_notice_and_sdk_eula_are_retained():
    required = [
        "ros_ws/src/orbbec_camera/LICENSE",
        "ros_ws/src/orbbec_camera/NOTICE",
        "ros_ws/src/orbbec_camera/SDK/LICENSE.txt",
        "ros_ws/src/orbbec_camera/SDK/End User License Agreement.txt",
        "ros_ws/src/orbbec_camera_msgs/LICENSE",
        "ros_ws/src/orbbec_camera_msgs/NOTICE",
    ]
    assert all((REPO_ROOT / path).is_file() for path in required)


def test_head_camera_preserves_native_raw_depth_without_pointcloud_or_registration():
    path = REPO_ROOT / "ros_ws/src/teleop_camera_bringup/config/camera_primary_params.yaml"
    params = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert params["enable_depth"] is True
    assert (params["depth_width"], params["depth_height"], params["depth_fps"]) == (
        1280,
        800,
        10,
    )
    assert params["depth_format"] == "Y16"
    assert params["depth_registration"] is False
    assert params["enable_depth_undistortion"] is False
    assert params["enable_point_cloud"] is False
    assert params["enable_colored_point_cloud"] is False
