#!/usr/bin/env python3
"""Read-only multi-pose MDH forward-kinematics comparison for GCR3-618.

The robot is never connected to and no motion API is imported or called.
Input files are the JSON snapshots produced by the existing get_* collector.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Iterable


def identity() -> list[list[float]]:
    return [[1.0 if r == c else 0.0 for c in range(4)] for r in range(4)]


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def tx(x: float) -> list[list[float]]:
    t = identity(); t[0][3] = x; return t


def tz(z: float) -> list[list[float]]:
    t = identity(); t[2][3] = z; return t


def rx(a: float) -> list[list[float]]:
    c, s = math.cos(a), math.sin(a)
    t = identity(); t[1][1], t[1][2], t[2][1], t[2][2] = c, -s, s, c
    return t


def rz(a: float) -> list[list[float]]:
    c, s = math.cos(a), math.sin(a)
    t = identity(); t[0][0], t[0][1], t[1][0], t[1][1] = c, -s, s, c
    return t


def fk_mdh(rows: list[list[float]], q: list[float]) -> list[list[float]]:
    """Apply A_i=Tx(a_i) Rx(alpha_i) Rz(q_i+theta0_i) Tz(d_i), SI units."""
    if len(rows) != 6 or len(q) != 6:
        raise ValueError("MDH model and joint vector must each contain six rows/values")
    t = identity()
    for i, (row, qi) in enumerate(zip(rows, q), 1):
        if len(row) < 4:
            raise ValueError(f"MDH row {i} needs a, alpha, d, theta0")
        a, alpha, d, theta0 = (float(v) for v in row[:4])
        t = matmul(t, tx(a))
        t = matmul(t, rx(alpha))
        t = matmul(t, rz(float(qi) + theta0))
        t = matmul(t, tz(d))
    return t


def rpy_xyz_matrix(rpy: list[float]) -> list[list[float]]:
    """Fixed XYZ roll-pitch-yaw convention: R=Rz(yaw) Ry(pitch) Rx(roll)."""
    if len(rpy) != 3:
        raise ValueError("pose orientation must contain three RPY values")
    roll, pitch, yaw = map(float, rpy)
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rotation_error_deg(a: list[list[float]], b: list[list[float]]) -> float:
    # trace(Ra^T Rb) is the rotation-angle invariant; clamp floating point drift.
    cosine = (sum(a[r][c] * b[r][c] for r in range(3) for c in range(3)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def load_config(path: Path) -> dict[str, Any]:
    config = json.loads(path.read_text(encoding="utf-8"))
    if config.get("joint_angle_unit") != "rad" or config.get("length_unit") != "m":
        raise ValueError("config must declare joint_angle_unit=rad and length_unit=m")
    # Validate model shape and values early.
    fk_mdh(config["get_calib_DH"], [0.0] * 6)
    return config


def sample_paths(inputs: Iterable[Path]) -> list[Path]:
    found: list[Path] = []
    for path in inputs:
        if path.is_dir():
            found.extend(sorted(p for p in path.rglob("*.json") if p.is_file()))
        elif path.is_file() and path.suffix.lower() == ".json":
            found.append(path)
        else:
            raise FileNotFoundError(f"not a JSON file or directory: {path}")
    # De-duplicate paths while preserving order.
    return list(dict.fromkeys(p.resolve() for p in found))


def validate_sample(path: Path, config: dict[str, Any], arm: str) -> dict[str, Any]:
    sample = json.loads(path.read_text(encoding="utf-8"))
    q = sample["get_actual_joints_position"]
    pose = sample["get_flange_pose"]
    if len(q) != 6 or len(pose) != 6:
        raise ValueError("sample must contain six joint values and six flange-pose values")
    if sample.get("target") not in (None, arm):
        raise ValueError(f"sample target {sample.get('target')!r} does not match --arm {arm!r}")
    if sample.get("note", "").find("read_only") < 0:
        raise ValueError("sample is not marked read_only; refusing non-snapshot input")

    calculated = fk_mdh(config["get_calib_DH"], list(map(float, q)))
    sdk_r = rpy_xyz_matrix(list(map(float, pose[3:6])))
    model_r = [row[:3] for row in calculated[:3]]
    delta_mm = [(calculated[i][3] - float(pose[i])) * 1000.0 for i in range(3)]
    return {
        "sample": str(path),
        "timestamp_utc": sample.get("collected_at_utc", ""),
        "arm": arm,
        "joints_deg": [math.degrees(float(v)) for v in q],
        "sdk_x_mm": float(pose[0]) * 1000,
        "sdk_y_mm": float(pose[1]) * 1000,
        "sdk_z_mm": float(pose[2]) * 1000,
        "model_x_mm": calculated[0][3] * 1000,
        "model_y_mm": calculated[1][3] * 1000,
        "model_z_mm": calculated[2][3] * 1000,
        "dx_mm": delta_mm[0], "dy_mm": delta_mm[1], "dz_mm": delta_mm[2],
        "position_error_mm": math.sqrt(sum(v * v for v in delta_mm)),
        "orientation_error_deg": rotation_error_deg(sdk_r, model_r),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("samples", nargs="+", type=Path, help="read-only sample JSON files and/or directories")
    parser.add_argument("--arm", choices=("right_main_arm", "left_slave_arm"), required=True)
    parser.add_argument("--config", type=Path, help="defaults to config/<arm>.json")
    parser.add_argument("--output", type=Path, default=Path("results"))
    args = parser.parse_args()
    config_path = args.config or Path("config") / f"{args.arm}.json"
    config = load_config(config_path)
    paths = sample_paths(args.samples)
    if not paths:
        parser.error("no JSON sample files found")
    results = [validate_sample(p, config, args.arm) for p in paths]
    args.output.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / f"{args.arm}_errors.csv"
    json_path = args.output / f"{args.arm}_errors.json"
    fields = list(results[0].keys())
    with csv_path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(results)
    summary = {
        "arm": args.arm,
        "model_config": str(config_path),
        "sample_count": len(results),
        "position_error_max_mm": max(r["position_error_mm"] for r in results),
        "position_error_rms_mm": math.sqrt(sum(r["position_error_mm"] ** 2 for r in results) / len(results)),
        "orientation_error_max_deg": max(r["orientation_error_deg"] for r in results),
        "orientation_error_rms_deg": math.sqrt(sum(r["orientation_error_deg"] ** 2 for r in results) / len(results)),
        "results_csv": str(csv_path),
        "interpretation": "Descriptive errors only; define acceptance limits from the experiment plan and measurement resolution.",
        "samples": results,
    }
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Samples: {len(results)}")
    print(f"Position error: max={summary['position_error_max_mm']:.6f} mm, RMS={summary['position_error_rms_mm']:.6f} mm")
    print(f"Orientation error: max={summary['orientation_error_max_deg']:.6f} deg, RMS={summary['orientation_error_rms_deg']:.6f} deg")
    print(f"Wrote {csv_path} and {json_path}")
    print(summary["interpretation"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
