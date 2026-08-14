from pathlib import Path

import pytest

from operator_hand_poses import HandHomeStore


def test_hand_home_store_records_each_task_and_side_atomically(tmp_path: Path):
    store = HandHomeStore(tmp_path)

    with pytest.raises(RuntimeError, match="not recorded"):
        store.load_side("assembly", "left")

    store.save_side("assembly", "left", range(20))

    assert store.load_side("assembly", "left") == tuple(float(i) for i in range(20))
    assert not tuple(store.path.parent.glob(".*.tmp"))
