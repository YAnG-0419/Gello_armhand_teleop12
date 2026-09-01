import json
from pathlib import Path

import pytest


rosbag2_py = pytest.importorskip("rosbag2_py")
pytest.importorskip("pyarrow")

from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, JointState
from teleop_core.contract import (
    DATA_LEFT_ARM_JOINT_NAMES,
    DATA_RIGHT_ARM_JOINT_NAMES,
    LEFT_COMMAND_JOINT_NAMES,
    RIGHT_COMMAND_JOINT_NAMES,
    VALIDATED_COMMAND_TOPIC,
    WUJI_COMMAND_TOPIC,
    WUJI_LEFT_JOINT_NAMES,
    WUJI_RIGHT_JOINT_NAMES,
    WUJI_STATE_TOPIC,
)
from teleop_data_collector.rosbag_to_lerobot import (
    _source_tree_snapshot,
    main,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
CONFIG = REPO_ROOT / "data_collection/config/convert_gello_lerobot_v2.yaml"


def _joint_state(
    names: tuple[str, ...] | list[str],
    positions: list[float],
    velocities: list[float] | None = None,
) -> JointState:
    message = JointState()
    message.name = list(names)
    message.position = positions
    message.velocity = velocities if velocities is not None else [0.01] * len(names)
    return message


def _image(value: int) -> Image:
    message = Image()
    message.height = 24
    message.width = 32
    message.encoding = "rgb8"
    message.step = message.width * 3
    message.data = bytes([value % 255]) * (message.height * message.step)
    return message


def _depth_image(value: int) -> Image:
    message = Image()
    message.height = 24
    message.width = 32
    message.encoding = "16UC1"
    message.step = message.width * 2
    pixel = int(value).to_bytes(2, byteorder="little", signed=False)
    message.data = pixel * (message.height * message.width)
    return message


def _write_synthetic_bag(path: Path) -> None:
    # Deliberately put the validated command sides in right/left source order.
    # Conversion must select by name and emit the fixed left/right contract order.
    messages = {
        VALIDATED_COMMAND_TOPIC: _joint_state(
            [*RIGHT_COMMAND_JOINT_NAMES, *LEFT_COMMAND_JOINT_NAMES],
            [*map(float, range(10, 17)), *map(float, range(0, 7))],
        ),
        WUJI_COMMAND_TOPIC.format(side="left"): _joint_state(
            WUJI_LEFT_JOINT_NAMES, list(map(float, range(20, 40)))
        ),
        WUJI_COMMAND_TOPIC.format(side="right"): _joint_state(
            WUJI_RIGHT_JOINT_NAMES, list(map(float, range(40, 60)))
        ),
        "/left/franka/joint_states": _joint_state(
            DATA_LEFT_ARM_JOINT_NAMES,
            list(map(float, range(100, 107))),
            list(map(float, range(110, 117))),
        ),
        "/right/franka/joint_states": _joint_state(
            DATA_RIGHT_ARM_JOINT_NAMES,
            list(map(float, range(120, 127))),
            list(map(float, range(130, 137))),
        ),
        WUJI_STATE_TOPIC.format(side="left"): _joint_state(
            WUJI_LEFT_JOINT_NAMES,
            list(map(float, range(140, 160))),
            list(map(float, range(160, 180))),
        ),
        WUJI_STATE_TOPIC.format(side="right"): _joint_state(
            WUJI_RIGHT_JOINT_NAMES,
            list(map(float, range(180, 200))),
            list(map(float, range(200, 220))),
        ),
        "/cam0/color/image_raw": _image(32),
        "/cam0/depth/image_raw": _depth_image(1000),
        "/cam1/color/image_raw": _image(64),
        "/cam2/color/image_raw": _image(96),
    }

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {}
    for topic, message in messages.items():
        type_name = (
            "sensor_msgs/msg/Image"
            if isinstance(message, Image)
            else "sensor_msgs/msg/JointState"
        )
        topic_types[topic] = type_name
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic,
                type=type_name,
                serialization_format="cdr",
            )
        )
    start_ns = 1_000_000_000
    for frame in range(8):
        timestamp_ns = start_ns + frame * 33_333_333
        for topic, message in messages.items():
            writer.write(topic, serialize_message(message), timestamp_ns)
    del writer

    state = {
        "bag_contract_version": 1,
        "state": "finalized",
        "finalized": True,
        "teleoperator": "gello",
        "source_bag": str(path),
        "workcell_id": "synthetic-dual-fr3-wuji",
        "workcell_config_hash": "synthetic-workcell-sha256",
        "control_config_id": "synthetic-gello-v1",
        "calibration_ids": {"arms": "synthetic", "hands": "synthetic"},
        "device_identities": {"workcell": "synthetic"},
        "source_topic_contract": topic_types,
        "timestamp_policy": "source_header_and_bag_receive_time",
    }
    (path / "collection_state.json").write_text(
        json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def test_synthetic_finalized_bag_converts_atomically_to_exact_lerobot_v2(
    tmp_path: Path,
) -> None:
    import pyarrow.parquet as pq

    bag = tmp_path / "episode_000000"
    output = tmp_path / "lerobot_dataset"
    _write_synthetic_bag(bag)
    source_before = _source_tree_snapshot([bag])

    main(["--config", str(CONFIG), str(bag), "--output", str(output)])

    assert _source_tree_snapshot([bag]) == source_before
    info = json.loads((output / "meta/info.json").read_text(encoding="utf-8"))
    metadata = json.loads(
        (output / "meta/conversion_metadata.json").read_text(encoding="utf-8")
    )
    assert info["codebase_version"] == "v2.0"
    assert info["teleoperator"] == "gello"
    assert info["features"]["action"]["shape"] == [54]
    assert info["features"]["observation.state"]["shape"] == [108]
    assert set(metadata["episodes"][0]["video_paths"]) == {
        "observation.images.cam0",
        "observation.images.cam1",
        "observation.images.cam2",
    }
    assert set(metadata["episodes"][0]["depth_image_storage"]) == {
        "observation.depths.cam0",
    }
    depth_feature = info["features"]["observation.depths.cam0"]
    assert depth_feature["dtype"] == "image"
    assert depth_feature["info"]["image.format"] == "raw16"
    assert depth_feature["info"]["image.fps"] == 20
    assert depth_feature["info"]["image.is_depth_map"] is True
    assert "unmatched_by_stream" in metadata["episodes"][0]

    parquet_path = next((output / "data").rglob("*.parquet"))
    table = pq.read_table(
        parquet_path,
        columns=["action", "observation.state", "observation.depths.cam0"],
    )
    action = table["action"][0].as_py()
    state = table["observation.state"][0].as_py()
    depth = table["observation.depths.cam0"][0].as_py()
    assert depth["encoding"] == "16UC1"
    assert depth["format"] == "raw16"
    assert len(depth["data"]) == 24 * 32 * 2
    assert action == [
        *map(float, range(0, 7)),
        *map(float, range(10, 17)),
        *map(float, range(20, 40)),
        *map(float, range(40, 60)),
    ]
    assert state == [
        *map(float, range(100, 107)),
        *map(float, range(110, 117)),
        *map(float, range(120, 127)),
        *map(float, range(130, 137)),
        *map(float, range(140, 160)),
        *map(float, range(160, 180)),
        *map(float, range(180, 200)),
        *map(float, range(200, 220)),
    ]


def test_failed_conversion_does_not_publish_partial_output(tmp_path: Path) -> None:
    bag = tmp_path / "episode_000000"
    output = tmp_path / "failed_dataset"
    _write_synthetic_bag(bag)
    state_path = bag / "collection_state.json"
    state = json.loads(state_path.read_text(encoding="utf-8"))
    state["state"] = "incomplete"
    state["finalized"] = False
    state_path.write_text(json.dumps(state), encoding="utf-8")
    source_before = _source_tree_snapshot([bag])

    with pytest.raises(SystemExit, match="not finalized"):
        main(["--config", str(CONFIG), str(bag), "--output", str(output)])

    assert _source_tree_snapshot([bag]) == source_before
    assert not output.exists()
