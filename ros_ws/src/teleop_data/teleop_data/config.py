from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class TopicSpec:
    name: str
    topic: str
    type_name: str
    required: bool


@dataclass(frozen=True)
class RecordingConfig:
    schema_version: str
    data_root: Path
    storage_id: str
    conversion_fps: int
    replay_speed: float
    replay_preposition_speed: float
    replay_hand_preposition_speed: float
    replay_rate: float
    replay_state_timeout: float
    replay_discovery_timeout: float
    topics: tuple[TopicSpec, ...]


def load_config(path):
    config_path = Path(path).expanduser()
    with config_path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError("Recording config must be a mapping.")
    required_root = {
        "schema_version",
        "data_root",
        "storage_id",
        "conversion",
        "replay",
        "topics",
    }
    root_fields = set(raw)
    missing_root = sorted(required_root - root_fields)
    unknown_root = sorted(root_fields - required_root)
    if missing_root or unknown_root:
        raise ValueError(
            f"Recording config fields differ: missing={missing_root}, "
            f"unknown={unknown_root}."
        )
    schema = str(raw["schema_version"]).strip()
    data_root = str(raw["data_root"]).strip()
    storage = str(raw["storage_id"]).strip()
    conversion = raw["conversion"]
    replay = raw["replay"]
    topics_raw = raw["topics"]
    if not schema or not data_root or not storage:
        raise ValueError("schema_version, data_root, and storage_id are required.")
    if not isinstance(topics_raw, dict) or not topics_raw:
        raise ValueError("Recording topics must be a non-empty mapping.")
    if not isinstance(conversion, dict) or set(conversion) != {"fps"}:
        raise ValueError("conversion must contain exactly: fps.")
    replay_fields = {
        "speed",
        "preposition_speed",
        "hand_preposition_speed",
        "rate",
        "state_timeout",
        "discovery_timeout",
    }
    if not isinstance(replay, dict) or set(replay) != replay_fields:
        raise ValueError(
            "replay must contain exactly: discovery_timeout, preposition_speed, "
            "hand_preposition_speed, rate, speed, state_timeout."
        )
    conversion_fps = int(conversion["fps"])
    replay_values = {key: float(replay[key]) for key in replay_fields}
    if conversion_fps <= 0 or any(value <= 0 for value in replay_values.values()):
        raise ValueError("Conversion and replay numeric values must be positive.")
    topics = []
    seen = set()
    for name, fields in topics_raw.items():
        if not isinstance(fields, dict):
            raise ValueError(f"Topic {name} must be a mapping.")
        if set(fields) != {"topic", "type", "required"}:
            raise ValueError(
                f"Topic {name} must contain exactly: topic, type, required."
            )
        if "required" not in fields or not isinstance(fields["required"], bool):
            raise ValueError(f"Topic {name} requires a boolean 'required' field.")
        topic = str(fields["topic"]).strip()
        type_name = str(fields["type"]).strip()
        if not topic.startswith("/") or not type_name:
            raise ValueError(f"Topic {name} needs an absolute name and type.")
        if topic in seen:
            raise ValueError(f"Duplicate recording topic: {topic}")
        seen.add(topic)
        topics.append(
            TopicSpec(
                str(name),
                topic,
                type_name,
                fields["required"],
            )
        )
    return RecordingConfig(
        schema,
        Path(data_root).expanduser(),
        storage,
        conversion_fps,
        replay_values["speed"],
        replay_values["preposition_speed"],
        replay_values["hand_preposition_speed"],
        replay_values["rate"],
        replay_values["state_timeout"],
        replay_values["discovery_timeout"],
        tuple(topics),
    )


def validate_topics(specs, discovered):
    errors = []
    warnings = []
    for spec in specs:
        actual = discovered.get(spec.topic, [])
        if not actual:
            target = errors if spec.required else warnings
            target.append(f"missing {spec.topic} [{spec.type_name}]")
        elif spec.type_name not in actual:
            errors.append(
                f"{spec.topic} expected {spec.type_name}, found {', '.join(actual)}"
            )
    return errors, warnings
