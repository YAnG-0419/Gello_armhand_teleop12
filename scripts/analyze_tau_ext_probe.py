#!/usr/bin/env python3
"""Decide the tau_ext sign convention from a probe recording, no eyeballs.

Reads a scripts/record_tau_ext_probe.py recording and prints, per joint:

- the sign verdict against the convention compiled into the safety
  gateway's contact gating. Measured 2026-07-29 (right arm, six joints,
  corr <= -0.93): tau_ext_hat_filtered carries the sign OPPOSITE to the
  external push, and the gate expects exactly that. MATCH means the
  recording confirms it; INVERTED means the gate's comparison in
  teleop_core/safety.py must be flipped before any contact trial.
- the quiet-baseline noise and a suggested contact_torque_threshold.

Ground truth needs no reported push direction: with the impedance
controller holding the disengaged arm, an external push deflects each
joint in the push direction (a spring), so the sign of the q deviation
identifies the true external torque direction sample by sample.
"""

import argparse
import json
import re

import numpy as np

SIDES = ("left", "right")
ACTIVE_TORQUE = 1.0  # Nm of |tau deviation| that counts as a push sample
MIN_DEFLECTION = 0.002  # rad of |q deviation| required for a valid sign
MIN_SAMPLES = 50


def load(path):
    rows = []
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if "schema" in row:
                continue
            rows.append(row)
    return rows


def series(rows, side, kind):
    times, matrix = [], []
    for row in rows:
        if row["side"] != side or row["kind"] != kind:
            continue
        found = {}
        names = row["name"]
        values = row["value"]
        if names and len(names) == len(values):
            for name, value in zip(names, values):
                match = re.search(r"joint([1-7])$", name.lower())
                if match:
                    found[int(match.group(1))] = float(value)
        elif len(values) == 7:
            found = {index + 1: float(v) for index, v in enumerate(values)}
        if len(found) != 7:
            continue
        times.append(row["t"])
        matrix.append([found[index] for index in range(1, 8)])
    return np.asarray(times), np.asarray(matrix)


def analyze(path, baseline_seconds):
    rows = load(path)
    verdicts = {}
    for side in SIDES:
        tau_t, tau = series(rows, side, "tau")
        q_t, q = series(rows, side, "q")
        if len(tau_t) < MIN_SAMPLES or len(q_t) < MIN_SAMPLES:
            print(f"=== {side}: not enough data "
                  f"(tau {len(tau_t)}, q {len(q_t)} rows) ===")
            continue
        start = tau_t[0]
        quiet = tau_t < start + baseline_seconds
        if quiet.sum() < 10:
            print(f"=== {side}: baseline window too short ===")
            continue
        tau_base = np.median(tau[quiet], axis=0)
        q_interp = np.column_stack(
            [np.interp(tau_t, q_t, q[:, j]) for j in range(7)]
        )
        q_base = np.median(q_interp[quiet], axis=0)
        dtau = tau - tau_base
        dq = q_interp - q_base
        quiet_p99 = np.percentile(np.abs(dtau[quiet]), 99, axis=0)

        print(f"=== {side} ===")
        side_votes = []
        for j in range(7):
            valid = (np.abs(dtau[:, j]) > ACTIVE_TORQUE) & (
                np.abs(dq[:, j]) > MIN_DEFLECTION
            )
            n = int(valid.sum())
            suggested = max(1.0, float(np.ceil(quiet_p99[j] * 6) / 2))
            if n < MIN_SAMPLES:
                print(
                    f"  j{j + 1}: not enough push samples ({n}); "
                    f"quiet p99 {quiet_p99[j]:.2f} Nm, "
                    f"suggested threshold {suggested:.1f} Nm"
                )
                continue
            # The gate expects the reported torque sign OPPOSITE to the push
            # (measured 2026-07-29), so gateway-agreement means
            # sign(dtau) == -sign(dq).
            agree = float(
                np.mean(np.sign(dtau[valid, j]) == -np.sign(dq[valid, j]))
            )
            corr = float(np.corrcoef(dtau[valid, j], dq[valid, j])[0, 1])
            if agree > 0.8 and corr < -0.5:
                verdict = "MATCH"
            elif agree < 0.2 and corr > 0.5:
                verdict = "INVERTED"
            else:
                verdict = "AMBIGUOUS"
            side_votes.append(verdict)
            print(
                f"  j{j + 1}: {verdict}  (agree {agree * 100:.0f}%, "
                f"corr {corr:+.2f}, {n} samples); quiet p99 "
                f"{quiet_p99[j]:.2f} Nm, suggested threshold {suggested:.1f} Nm"
            )
        decided = [v for v in side_votes if v != "AMBIGUOUS"]
        if decided and all(v == "MATCH" for v in decided):
            verdicts[side] = "MATCH"
        elif decided and all(v == "INVERTED" for v in decided):
            verdicts[side] = "INVERTED"
        elif decided:
            verdicts[side] = "MIXED"

    print()
    if not verdicts:
        print("VERDICT: no usable data - re-run the probe with firmer pushes.")
        return 1
    if all(v == "MATCH" for v in verdicts.values()):
        print("VERDICT: tau_ext sign MATCHES the gateway's compiled "
              "convention; no code change needed.")
        return 0
    if all(v == "INVERTED" for v in verdicts.values()):
        print("VERDICT: tau_ext sign is INVERTED versus the gateway's "
              "compiled convention - flip the comparison in "
              "teleop_core/safety.py before any contact trial.")
        return 0
    print(f"VERDICT: inconsistent across sides/joints ({verdicts}) - "
          "re-run with slower, firmer single-direction pushes.")
    return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--baseline-seconds", type=float, default=5.0)
    args = parser.parse_args()
    return analyze(args.log, args.baseline_seconds)


if __name__ == "__main__":
    raise SystemExit(main())
