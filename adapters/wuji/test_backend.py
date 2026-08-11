from types import SimpleNamespace

import numpy as np
import pytest

from adapters.wuji.backend import _hand2_feedback_positions


def frame(*, missing_node=None):
    node_ids = [finger * 5 + joint for finger in range(5) for joint in range(1, 5)]
    entries = [
        SimpleNamespace(nid=node_id, position=float(index))
        for index, node_id in enumerate(node_ids)
        if node_id != missing_node
    ]
    return SimpleNamespace(num_joints=len(entries), joints=list(reversed(entries)))


def test_hand2_feedback_is_reordered_by_node_id() -> None:
    np.testing.assert_array_equal(_hand2_feedback_positions(frame()), np.arange(20))


def test_hand2_feedback_requires_all_twenty_joints() -> None:
    with pytest.raises(RuntimeError, match="missing joints"):
        _hand2_feedback_positions(frame(missing_node=7))
