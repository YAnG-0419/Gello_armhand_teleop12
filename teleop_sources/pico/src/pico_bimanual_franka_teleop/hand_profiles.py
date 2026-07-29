"""Host-side retargeting profiles selected independently for each hand."""

from __future__ import annotations

from pathlib import Path


REGISTERED_RETARGETING_MODELS = ("g20",)


def create_hand_retargeter(
    model: str,
    *,
    side: str,
    assets_root: Path,
    max_iterations: int,
):
    """Create the kinematic retargeter for one configured device model."""
    normalized = str(model).strip().lower()
    if normalized != "g20":
        raise ValueError(
            f"no retargeting profile registered for {model!r}; "
            f"registered models: {list(REGISTERED_RETARGETING_MODELS)}"
        )

    # Import lazily so arm-only runs never pay for pinocchio.
    from .hand_retarget import L20Retargeter, THUMB_OPPOSITION_YAW_ROLL

    urdf = (
        Path(assets_root)
        / "linkerhand_l20"
        / side
        / f"linkerhand_l20_{side}.urdf"
    )
    if not urdf.is_file():
        raise FileNotFoundError(f"Hand URDF not found: {urdf}")
    return L20Retargeter(
        urdf,
        side,
        max_iterations=max_iterations,
        thumb_opposition_fixed=THUMB_OPPOSITION_YAW_ROLL[side],
    )
