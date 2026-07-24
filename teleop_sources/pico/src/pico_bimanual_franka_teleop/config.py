from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


def _exact_mapping(value, fields: set[str], label: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    actual = set(value)
    if actual != fields:
        missing = sorted(fields - actual)
        unknown = sorted(actual - fields)
        raise ValueError(f"{label} fields differ: missing={missing}, unknown={unknown}")
    return value


def _positive(value, label: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result <= 0:
        raise ValueError(f"{label} must be a positive finite number")
    return result


@dataclass(frozen=True)
class UdpConfig:
    command_host: str
    command_port: int
    state_host: str
    state_port: int
    state_timeout: float


@dataclass(frozen=True)
class HostConfig:
    translation_scale: float
    rotation_scale: float
    control_rate: float
    max_joint_speed: float
    robot_state_wait_timeout: float


@dataclass(frozen=True)
class MotionTrackerConfig:
    serials: dict[str, str]
    tracker_to_control: dict[str, dict]
    ready_timeout: float
    stale_timeout: float
    keyboard_device: str


@dataclass(frozen=True)
class PicoConfig:
    udp: UdpConfig
    host: HostConfig
    input: MotionTrackerConfig


def load_config(path, require_tracker_serials: bool) -> PicoConfig:
    config_path = Path(path)
    with config_path.open(encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    root = _exact_mapping(root, {"udp", "host", "input"}, str(config_path))

    udp = _exact_mapping(
        root["udp"],
        {
            "command_host",
            "command_port",
            "state_host",
            "state_port",
            "state_timeout",
        },
        "udp",
    )
    command_port = int(udp["command_port"])
    state_port = int(udp["state_port"])
    if not 1 <= command_port <= 65535 or not 1 <= state_port <= 65535:
        raise ValueError("UDP ports must be between 1 and 65535")
    command_host = str(udp["command_host"]).strip()
    state_host = str(udp["state_host"]).strip()
    if not command_host or not state_host:
        raise ValueError("UDP hosts must be non-empty")

    host = _exact_mapping(
        root["host"],
        {
            "translation_scale",
            "rotation_scale",
            "control_rate",
            "max_joint_speed",
            "robot_state_wait_timeout",
        },
        "host",
    )

    input_config = _exact_mapping(
        root["input"],
        {
            "type",
            "serials",
            "ready_timeout",
            "stale_timeout",
            "activation",
            "tracker_to_control",
        },
        "input",
    )
    if input_config["type"] != "motion_trackers":
        raise ValueError("input.type must be motion_trackers")
    serials = _exact_mapping(
        input_config["serials"], {"left", "right"}, "input.serials"
    )
    serials = {side: str(serials[side]).strip() for side in ("left", "right")}
    if any(not serial for serial in serials.values()):
        raise ValueError("Both input tracker serials must be non-empty")
    if serials["left"] == serials["right"]:
        raise ValueError("Left and right tracker serials must differ")
    if require_tracker_serials and any(
        serial.startswith("REPLACE_WITH_") for serial in serials.values()
    ):
        raise ValueError(
            "Motion tracker serials are not configured; run "
            "scripts/list_pico_trackers.sh and edit config/pico.yaml"
        )

    activation = _exact_mapping(
        input_config["activation"], {"type", "device"}, "input.activation"
    )
    if activation["type"] != "keyboard":
        raise ValueError("input.activation.type must be keyboard")
    keyboard_device = str(activation["device"]).strip()
    if not keyboard_device:
        raise ValueError("input.activation.device must be non-empty")

    transforms = _exact_mapping(
        input_config["tracker_to_control"],
        {"left", "right"},
        "input.tracker_to_control",
    )
    for side in ("left", "right"):
        transform = _exact_mapping(
            transforms[side],
            {"translation_xyz", "quaternion_xyzw"},
            f"input.tracker_to_control.{side}",
        )
        translation = np.asarray(transform["translation_xyz"], dtype=float)
        quaternion = np.asarray(transform["quaternion_xyzw"], dtype=float)
        if translation.shape != (3,) or quaternion.shape != (4,):
            raise ValueError(f"{side} tracker transform dimensions must be 3 and 4")
        if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(quaternion)):
            raise ValueError(f"{side} tracker transform must be finite")
        if np.linalg.norm(quaternion) <= 1e-8:
            raise ValueError(f"{side} tracker quaternion must be non-zero")

    return PicoConfig(
        udp=UdpConfig(
            command_host=command_host,
            command_port=command_port,
            state_host=state_host,
            state_port=state_port,
            state_timeout=_positive(udp["state_timeout"], "udp.state_timeout"),
        ),
        host=HostConfig(
            translation_scale=_positive(
                host["translation_scale"], "host.translation_scale"
            ),
            rotation_scale=_positive(host["rotation_scale"], "host.rotation_scale"),
            control_rate=_positive(host["control_rate"], "host.control_rate"),
            max_joint_speed=_positive(
                host["max_joint_speed"], "host.max_joint_speed"
            ),
            robot_state_wait_timeout=_positive(
                host["robot_state_wait_timeout"],
                "host.robot_state_wait_timeout",
            ),
        ),
        input=MotionTrackerConfig(
            serials=serials,
            tracker_to_control=transforms,
            ready_timeout=_positive(
                input_config["ready_timeout"], "input.ready_timeout"
            ),
            stale_timeout=_positive(
                input_config["stale_timeout"], "input.stale_timeout"
            ),
            keyboard_device=keyboard_device,
        ),
    )
