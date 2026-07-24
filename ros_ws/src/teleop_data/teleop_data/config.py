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
    topics: tuple[TopicSpec, ...]


def load_config(path):
    config_path = Path(path).expanduser()
    with config_path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError("Recording config must be a mapping.")
    schema = str(raw.get("schema_version", "")).strip()
    data_root = str(raw.get("data_root", "")).strip()
    storage = str(raw.get("storage_id", "")).strip()
    topics_raw = raw.get("topics")
    if not schema or not data_root or not storage:
        raise ValueError("schema_version, data_root, and storage_id are required.")
    if not isinstance(topics_raw, dict) or not topics_raw:
        raise ValueError("Recording topics must be a non-empty mapping.")
    topics = []
    seen = set()
    for name, fields in topics_raw.items():
        if not isinstance(fields, dict):
            raise ValueError(f"Topic {name} must be a mapping.")
        topic = str(fields.get("topic", "")).strip()
        type_name = str(fields.get("type", "")).strip()
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
                bool(fields.get("required", True)),
            )
        )
    return RecordingConfig(
        schema,
        Path(data_root).expanduser(),
        storage,
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
