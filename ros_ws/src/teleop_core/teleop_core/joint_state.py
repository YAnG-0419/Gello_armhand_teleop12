import math
import re

import numpy as np


def ordered_joint_positions(names, positions, required_names):
    if len(names) != len(positions) or len(names) != len(set(names)):
        return None
    values = dict(zip(names, positions))
    if any(name not in values for name in required_names):
        return None
    ordered = [values[name] for name in required_names]
    if not all(math.isfinite(value) for value in ordered):
        return None
    return ordered


def ordered_arm_positions(names, positions, side):
    found = {}
    for name, position in zip(names, positions):
        lowered = name.lower()
        if side not in lowered:
            continue
        match = re.search(r"joint([1-7])$", lowered)
        if match:
            found[int(match.group(1))] = float(position)
    if len(found) != 7:
        raise ValueError(f"Expected seven arm joints for {side}; found {sorted(found)}.")
    result = np.asarray([found[index] for index in range(1, 8)], dtype=np.float64)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"Arm state for {side} contains non-finite positions.")
    return result
