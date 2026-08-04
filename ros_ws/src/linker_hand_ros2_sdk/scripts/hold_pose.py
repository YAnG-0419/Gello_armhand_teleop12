"""Hold the hand at an exact tick vector so it can be photographed beside the model.

    # the bridge must be stopped: this needs exclusive CAN access
    python3 hold_pose.py --device 1 --ticks 89,178,118,13,178,178,120,13,96,96,96,96,96,96,96,96,96,96,96,96
    python3 hold_pose.py --device 1 --joint index_pip --tick 128   # one joint, rest straight

THIS MOVES THE HAND. It goes to the pose, holds it until you press Enter, then
returns to straight.

It prints the same vector in the form `pose_view.py --ticks` expects, so the
model and the metal can be put in provably identical configurations. That
matters more than it sounds: comparing a rendered pose against the hand in
whatever state it happened to be in proves nothing, and it is easy to do by
accident.

The joint order is the URDF's throughout -- thumb, index, middle, ring, pinky,
each roll/pitch/pip/dip -- and the vendor's own order is different, so the
reordering happens here, by name, once.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from linker_hand_ros2_sdk.LinkerHand import o30i_control  # noqa: E402
from linker_hand_ros2_sdk.o30i_contract import (  # noqa: E402
    O30I_DRIVER_TO_URDF, O30I_RIGHT_LOWER, O30I_RIGHT_UPPER,
    O30I_URDF_JOINT_NAMES)
from linker_hand_ros2_sdk.o30i_transport import (  # noqa: E402
    bundled_libcanbus_path, make_libcanbus_communication)

o30i_control.CANFDCommunication = make_libcanbus_communication(
    o30i_control, bundled_libcanbus_path())


def driver_vector(ticks: dict[str, int]) -> list[int]:
    return [int(ticks[O30I_DRIVER_TO_URDF[d]])
            for d in list(o30i_control.JOINT_NAMES)]


def urdf_degrees(name: str, tick: int) -> float:
    import math
    i = O30I_URDF_JOINT_NAMES.index(name)
    lower, upper = O30I_RIGHT_LOWER[i], O30I_RIGHT_UPPER[i]
    return math.degrees(lower + (tick / 255.0) * (upper - lower))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", type=int, default=1)
    parser.add_argument("--channel", type=int, default=0)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--ticks", help="20 values 0..255 in URDF joint order")
    group.add_argument("--joint", choices=sorted(O30I_URDF_JOINT_NAMES),
                       help="hold one joint, everything else straight")
    parser.add_argument("--tick", type=int, default=128,
                        help="tick for --joint (default 128)")
    args = parser.parse_args()

    if args.ticks:
        parts = [p for p in args.ticks.replace(",", " ").split() if p]
        if len(parts) != len(O30I_URDF_JOINT_NAMES):
            raise SystemExit(f"--ticks needs {len(O30I_URDF_JOINT_NAMES)} values, "
                             f"got {len(parts)}")
        values = [int(float(p)) for p in parts]
        if any(not 0 <= v <= 255 for v in values):
            raise SystemExit("--ticks must all be within 0..255")
        pose = dict(zip(O30I_URDF_JOINT_NAMES, values))
    else:
        if not 0 <= args.tick <= 255:
            raise SystemExit("--tick must be within 0..255")
        pose = {name: 0 for name in O30I_URDF_JOINT_NAMES}
        pose[args.joint] = args.tick

    ordered = [pose[n] for n in O30I_URDF_JOINT_NAMES]
    print("\n  the identical configuration for the model:\n")
    print("    python pose_view.py --hand o30i_right --ticks "
          + ",".join(str(v) for v in ordered) + "\n")
    print(f"  {'joint':<20}{'tick':>6}{'URDF deg':>10}")
    for name in O30I_URDF_JOINT_NAMES:
        print(f"  {name:<20}{pose[name]:>6}{urdf_degrees(name, pose[name]):>10.1f}")

    hand = o30i_control.LinkerHandO30IController(
        hand_type="right", canfd_device=args.device, channel=args.channel,
        comm_type="libcanbus")
    if not hand.is_connected:
        print("\n  no transport; is the bridge still holding the bus?")
        return 1
    model = str(hand.get_product_model() or "")
    if "O30" not in model.upper():
        print(f"\n  index {args.device} is not an O30i ({model!r})")
        hand.close()
        return 1
    straight = {name: 0 for name in O30I_URDF_JOINT_NAMES}
    try:
        hand.setup()
        time.sleep(0.2)
        hand.set_target_position(driver_vector(straight))
        time.sleep(1.2)
        input("\n  hand is straight; press Enter to move to the pose ")
        hand.set_target_position(driver_vector(pose))
        time.sleep(1.5)
        measured = hand.get_current_position()
        if measured is not None:
            by_urdf = {O30I_DRIVER_TO_URDF[d]: v
                       for d, v in zip(list(o30i_control.JOINT_NAMES), measured)}
            worst = max(O30I_URDF_JOINT_NAMES,
                        key=lambda n: abs(by_urdf[n] - pose[n]))
            print(f"  holding. largest tick error: {worst} "
                  f"commanded {pose[worst]}, reports {by_urdf[worst]}")
        input("  compare with the model now; press Enter to release ")
        return 0
    except KeyboardInterrupt:
        print("\n  interrupted")
        return 130
    finally:
        try:
            hand.set_target_position(driver_vector(straight))
            time.sleep(1.0)
        finally:
            hand.close()


if __name__ == "__main__":
    sys.exit(main())
