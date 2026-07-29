#!/usr/bin/env python3
"""Record external joint torques and joint states for the tau_ext probe.

Run inside the tools container with both arms up and teleoperation
disengaged:

    docker compose run --rm tools python3 \
        /workspace/franka_upper_body_teleop/scripts/record_tau_ext_probe.py \
        --output /data/diagnostics/tau_probe.jsonl --duration 60

Protocol: keep hands off both arms for the first five seconds (baseline),
then push and pull each arm gently in varied directions - end effector and
mid links, forward, backward, and sideways. Analyze offline with
scripts/analyze_tau_ext_probe.py; no human judgement of the printout is
needed.
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

SIDES = ("left", "right")
TOPICS = {
    "tau": "/{side}/franka_robot_state_broadcaster/external_joint_torques",
    "q": "/{side}/franka/joint_states",
}


class ProbeRecorder(Node):
    def __init__(self, stream):
        super().__init__("tau_ext_probe_recorder")
        self.stream = stream
        self.counts = {}
        for side in SIDES:
            for kind, pattern in TOPICS.items():
                self.create_subscription(
                    JointState,
                    pattern.format(side=side),
                    lambda msg, side=side, kind=kind: self._row(side, kind, msg),
                    qos_profile_sensor_data,
                )

    def _row(self, side, kind, message):
        values = message.effort if kind == "tau" else message.position
        self.stream.write(
            json.dumps(
                {
                    "t": round(time.monotonic(), 5),
                    "side": side,
                    "kind": kind,
                    "name": list(message.name),
                    "value": [round(float(v), 6) for v in values],
                },
                separators=(",", ":"),
            )
            + "\n"
        )
        self.counts[(side, kind)] = self.counts.get((side, kind), 0) + 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    with open(args.output, "w", encoding="utf-8") as stream:
        stream.write(
            json.dumps(
                {
                    "schema": "tau-ext-probe.v1",
                    "written_at": time.time(),
                    "protocol": "hands off 5 s, then gentle varied pushes",
                }
            )
            + "\n"
        )
        node = ProbeRecorder(stream)
        deadline = time.monotonic() + args.duration
        print(f"recording {args.duration:.0f} s to {args.output} ...", flush=True)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        for side in SIDES:
            for kind in TOPICS:
                count = node.counts.get((side, kind), 0)
                status = "" if count else "   <- MISSING, topic never arrived"
                print(f"{side} {kind}: {count} messages{status}")
        node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
