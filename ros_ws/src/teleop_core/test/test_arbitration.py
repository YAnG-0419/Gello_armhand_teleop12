import pytest

from teleop_core.arbitration import SourceArbiter


def test_one_source_owns_channel_until_timeout():
    arbiter = SourceArbiter(("pico", "replay"), timeout=0.25)
    assert arbiter.accept("pico", "a", 0, 1.0)
    assert not arbiter.accept("pico", "a", 1, 1.1)
    with pytest.raises(ValueError, match="still owns"):
        arbiter.accept("replay", "b", 0, 1.2)
    assert arbiter.accept("replay", "b", 0, 1.36)


def test_stale_sequences_are_rejected():
    arbiter = SourceArbiter(("pico",), timeout=0.25)
    arbiter.accept("pico", "a", 2, 1.0)
    with pytest.raises(ValueError, match="stale"):
        arbiter.accept("pico", "a", 2, 1.1)
