"""Dataset-facing hand layouts.

These profiles describe recorded vendor state/action vectors. They are kept
separate from the live bridge profiles so offline conversion remains ROS- and
hardware-driver-independent.
"""

from __future__ import annotations

from dataclasses import dataclass


G20_JOINT_NAMES = (
    "Thumb Base",
    "Index Finger Base",
    "Middle Finger Base",
    "Ring Finger Base",
    "Pinky Finger Base",
    "Thumb Abduction",
    "Index Finger Abduction",
    "Middle Finger Abduction",
    "Ring Finger Abduction",
    "Pinky Finger Abduction",
    "Thumb Horizontal Abduction",
    "Reserved 1",
    "Reserved 2",
    "Reserved 3",
    "Reserved 4",
    "Thumb Tip",
    "Index Finger Tip",
    "Middle Finger Tip",
    "Ring Finger Tip",
    "Pinky Finger Tip",
)

O30I_JOINT_NAMES = (
    "thumb_cmc_roll",
    "thumb_cmc_yaw",
    "thumb_mcp",
    "thumb_ip",
    "index_mcp_roll",
    "index_mcp_pitch",
    "index_pip",
    "index_dip",
    "middle_mcp_roll",
    "middle_mcp_pitch",
    "middle_pip",
    "middle_dip",
    "ring_mcp_roll",
    "ring_mcp_pitch",
    "ring_pip",
    "ring_dip",
    "pinky_mcp_roll",
    "pinky_mcp_pitch",
    "pinky_pip",
    "pinky_dip",
)
O30I_RIGHT_LOWER = (
    0.0, 0.0, 0.0, 0.0,
    -0.4, 0.0, 0.0, 0.0,
    -0.38, 0.0, 0.0, 0.0,
    -0.28, 0.0, 0.0, 0.0,
    -0.28, 0.0, 0.0, 0.0,
)
O30I_RIGHT_UPPER = (
    0.6108, 2.094, 1.7134, 1.733,
    0.03711, 1.72918, 1.63, 1.66112,
    0.05418, 1.884, 1.7071, 1.5863,
    0.18823, 1.9626, 1.6621, 1.6749,
    0.28103, 1.8497, 1.7026, 1.7331,
)


@dataclass(frozen=True)
class HandDataProfile:
    model: str
    joint_names: tuple[str, ...]
    lower_bounds: tuple[float, ...]
    upper_bounds: tuple[float, ...]

    @property
    def width(self) -> int:
        return len(self.joint_names)


def create_hand_data_profile(model: str, *, side: str = "right") -> HandDataProfile:
    normalized = str(model).strip().lower()
    if normalized not in {"g20", "o30i"}:
        raise ValueError(
            f"unsupported recorded hand model {model!r}; "
            "registered models: ['g20', 'o30i']"
        )
    if normalized == "o30i":
        if side != "right":
            raise ValueError("the checked-in O30i data profile currently supports right only")
        return HandDataProfile(
            model="o30i",
            joint_names=O30I_JOINT_NAMES,
            lower_bounds=O30I_RIGHT_LOWER,
            upper_bounds=O30I_RIGHT_UPPER,
        )
    return HandDataProfile(
        model="g20",
        joint_names=G20_JOINT_NAMES,
        lower_bounds=(0.0,) * len(G20_JOINT_NAMES),
        upper_bounds=(255.0,) * len(G20_JOINT_NAMES),
    )


def default_hand_profiles() -> dict[str, HandDataProfile]:
    return {
        side: create_hand_data_profile("g20", side=side)
        for side in ("left", "right")
    }
