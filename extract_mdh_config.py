#!/usr/bin/env python3
"""Extract only calibrated MDH parameters from a local read-only JSON snapshot."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def extract(snapshot_path: Path, arm: str, output: Path) -> None:
    snapshot: dict[str, Any] = json.loads(snapshot_path.read_text(encoding="utf-8"))
    if snapshot.get("target") != arm:
        raise ValueError(f"snapshot target must be {arm!r}; found {snapshot.get('target')!r}")
    if "read_only" not in snapshot.get("note", ""):
        raise ValueError("input snapshot must be marked read_only")
    rows = snapshot.get("get_calib_DH")
    if not isinstance(rows, list) or len(rows) != 6 or any(not isinstance(row, list) or len(row) < 4 for row in rows):
        raise ValueError("snapshot must contain six get_calib_DH rows with at least four values")
    config = {
        "arm": arm,
        "length_unit": "m",
        "joint_angle_unit": "rad",
        "pose_orientation_convention": "fixed XYZ RPY; R=Rz(yaw)Ry(pitch)Rx(roll)",
        "transform_convention": "A_i=Tx(a_i)Rx(alpha_i)Rz(q_i+theta0_i)Tz(d_i)",
        "parameter_source_file": snapshot_path.name,
        "get_calib_DH": [list(map(float, row[:5])) for row in rows],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path, help="local controller snapshot created by the read-only collector")
    parser.add_argument("--arm", choices=("right_main_arm", "left_slave_arm"), required=True)
    parser.add_argument("--output", type=Path, help="defaults to config/local_<arm>.json")
    args = parser.parse_args()
    output = args.output or Path("config") / f"local_{args.arm}.json"
    try:
        extract(args.snapshot, args.arm, output)
    except (OSError, json.JSONDecodeError, KeyError, ValueError) as exc:
        parser.error(str(exc))
    print(f"Wrote local model config: {output}")
    print("The output contains MDH parameters only; controller address, serial and joint telemetry are omitted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
