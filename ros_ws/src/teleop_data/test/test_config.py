from pathlib import Path

import pytest

from teleop_data.config import load_config, validate_topics


def test_recording_config_loads():
    path = Path(__file__).parents[1] / "config" / "recording.yaml"
    config = load_config(path)
    assert config.schema_version == "franka.teleop.raw.v1"
    assert len(config.topics) >= 4


def test_missing_optional_topic_is_warning():
    path = Path(__file__).parents[1] / "config" / "recording.yaml"
    config = load_config(path)
    discovered = {
        spec.topic: [spec.type_name] for spec in config.topics if spec.required
    }
    errors, warnings = validate_topics(config.topics, discovered)
    assert not errors
    assert warnings
