#!/usr/bin/env python3
import argparse

from pico_bimanual_franka_teleop.simulation import DualFr3Simulation


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock-xr", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--translation-scale", type=float, default=1.0)
    parser.add_argument("--control-rate", type=float, default=100.0)
    args = parser.parse_args()
    simulation = DualFr3Simulation(
        mock_xr=args.mock_xr,
        translation_scale=args.translation_scale,
        control_rate=args.control_rate,
    )
    simulation.run(duration=args.duration, headless=args.headless)


if __name__ == "__main__":
    main()
