from dataclasses import dataclass

import numpy as np

from .contract import COMMAND_JOINT_NAMES, SIDES, command_names


LOWER_LIMITS = np.array(
    [-2.9007400167, -1.8360900167, -2.9007400167, -3.0770200167,
     -2.87630335, 0.43982265, -3.05083335] * 2
)
UPPER_LIMITS = np.array(
    [2.9007400167, 1.8360900167, 2.9007400167, -0.1169370833,
     2.87630335, 4.62163335, 3.05083335] * 2
)


@dataclass(frozen=True)
class ValidatedCommand:
    names: tuple[str, ...]
    positions: tuple[float, ...]


class CommandSafetyGate:
    def __init__(self, max_joint_speed, max_initial_delta, nominal_dt):
        if max_joint_speed <= 0 or max_initial_delta <= 0 or nominal_dt <= 0:
            raise ValueError("Safety limits must be positive.")
        self.max_joint_speed = float(max_joint_speed)
        self.max_initial_delta = float(max_initial_delta)
        self.nominal_dt = float(nominal_dt)
        self.active = {side: False for side in SIDES}
        self.last_output = {side: None for side in SIDES}
        self.last_time = {side: None for side in SIDES}

    def reset(self):
        for side in SIDES:
            self.active[side] = False
            self.last_output[side] = None
            self.last_time[side] = None

    def validate(self, active_sides, names, positions, measured_q, now):
        active_sides = tuple(active_sides)
        names = tuple(names)
        positions = tuple(float(value) for value in positions)
        measured = np.asarray(measured_q, dtype=float)
        if measured.shape != (14,) or not np.all(np.isfinite(measured)):
            raise ValueError("Measured state must contain 14 finite positions.")
        if names != command_names(active_sides) or len(positions) != len(names):
            raise ValueError("Command names do not match active sides.")
        values = dict(zip(names, positions))
        output_names = []
        output_positions = []
        for side in SIDES:
            offset = 0 if side == "left" else 7
            if side not in active_sides:
                self.active[side] = False
                self.last_output[side] = None
                self.last_time[side] = None
                continue
            side_names = COMMAND_JOINT_NAMES[offset:offset + 7]
            target = np.asarray([values[name] for name in side_names], dtype=float)
            if not np.all(np.isfinite(target)):
                raise ValueError(f"{side} command contains non-finite positions.")
            if np.any(target < LOWER_LIMITS[offset:offset + 7]) or np.any(
                target > UPPER_LIMITS[offset:offset + 7]
            ):
                raise ValueError(f"{side} command exceeds FR3 joint limits.")
            if not self.active[side]:
                current = measured[offset:offset + 7]
                if np.max(np.abs(target - current)) > self.max_initial_delta:
                    raise ValueError(f"{side} first target is too far from measured state.")
                self.last_output[side] = current.copy()
                self.last_time[side] = now - self.nominal_dt
                self.active[side] = True
            dt = min(0.1, max(self.nominal_dt, now - self.last_time[side]))
            max_step = self.max_joint_speed * dt
            command = self.last_output[side] + np.clip(
                target - self.last_output[side], -max_step, max_step
            )
            self.last_output[side] = command
            self.last_time[side] = now
            output_names.extend(side_names)
            output_positions.extend(command)
        if not output_names:
            return None
        return ValidatedCommand(tuple(output_names), tuple(float(v) for v in output_positions))
