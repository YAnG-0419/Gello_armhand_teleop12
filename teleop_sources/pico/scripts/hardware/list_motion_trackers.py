#!/usr/bin/env python3
import argparse
import time

import numpy as np

from pico_bimanual_franka_teleop.config import load_config


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = load_config(args.config, allow_unconfigured_trackers=True)

    import xrobotoolkit_sdk as xrt

    xrt.init()
    deadline = time.monotonic() + config.input.motion_trackers.ready_timeout
    try:
        print("Move one tracker at a time to identify its serial; Ctrl-C exits.")
        while True:
            serials = list(xrt.get_motion_tracker_serial_numbers())
            poses = list(xrt.get_motion_tracker_pose())
            if serials and len(serials) == len(poses):
                parts = []
                for serial, pose in zip(serials, poses):
                    position = np.asarray(pose, dtype=float)[:3]
                    parts.append(
                        f"{serial}: [{position[0]:+.3f}, "
                        f"{position[1]:+.3f}, {position[2]:+.3f}]"
                    )
                print("\r" + " | ".join(parts) + " " * 8, end="", flush=True)
                deadline = (
                    time.monotonic()
                    + config.input.motion_trackers.ready_timeout
                )
            elif time.monotonic() >= deadline:
                raise TimeoutError("No PICO motion tracker data was received")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()
    finally:
        xrt.close()


if __name__ == "__main__":
    main()
