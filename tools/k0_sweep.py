#!/usr/bin/env python3
"""
Sweep SIM_K0_OVERRIDE against test_balance across equilibrium keyframes.

Produces:
  1) per-run CSV (one row per (K0, keyframe))
  2) aggregate CSV (one row per K0)
  3) console summary with best K0 by worst-case |final_x|
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence


SUMMARY_RE = re.compile(r"^SUMMARY\s+(.+)$", re.MULTILINE)


@dataclass
class RunResult:
    k0: float
    key: str
    rc: int
    summary: Dict[str, float]


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Sweep K0 via SIM_K0_OVERRIDE and run test_balance.")
    ap.add_argument("--test-balance", default="build/simulate_app/test_balance",
                    help="Path to test_balance executable")
    ap.add_argument("--model", default="myRobot/scene.xml", help="Model path for test_balance")
    ap.add_argument("--duration", type=float, default=240.0, help="Simulation duration per run (s)")
    ap.add_argument("--print-dt", type=float, default=None,
                    help="Print cadence passed to test_balance (default: duration)")
    ap.add_argument("--k0-values", default="-0.1,-0.2,-0.3,-0.4,-0.6,-0.8",
                    help="Comma-separated K0 values")
    ap.add_argument("--lut", default="lqr_lut.csv",
                    help="LUT CSV to derive hip/keyframe list when --keys omitted")
    ap.add_argument("--keys", default=None,
                    help="Comma-separated keyframe names (default: derive from --lut hip column)")
    ap.add_argument("--run-csv", default="k0_sweep_runs.csv", help="Per-run output CSV")
    ap.add_argument("--agg-csv", default="k0_sweep_agg.csv", help="Aggregate output CSV")
    ap.add_argument("--sim-use-ekf", default="0", choices=["0", "1"],
                    help="Set SIM_USE_EKF for runs (default: 0, ground-truth state)")
    ap.add_argument("--sim-mc-debug", default="0", choices=["0", "1"],
                    help="Set SIM_MC_DEBUG for runs (default: 0)")
    return ap.parse_args()


def parse_float_list(csv_values: str) -> List[float]:
    vals: List[float] = []
    for tok in csv_values.split(","):
        tok = tok.strip()
        if not tok:
            continue
        vals.append(float(tok))
    if not vals:
        raise ValueError("empty float list")
    return vals


def key_name_from_hip(hip: float) -> str:
    sign = "m" if hip < 0.0 else "p"
    mag = int(round(abs(hip) * 10000.0))
    return f"eq_hip_{sign}{mag:04d}"


def keys_from_lut(lut_path: Path) -> List[str]:
    keys: List[str] = []
    with lut_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if "hip" not in (reader.fieldnames or []):
            raise ValueError(f"{lut_path} is missing 'hip' column")
        for row in reader:
            hip = float(row["hip"])
            keys.append(key_name_from_hip(hip))
    # preserve order and deduplicate
    dedup: List[str] = []
    seen = set()
    for k in keys:
        if k not in seen:
            seen.add(k)
            dedup.append(k)
    return dedup


def parse_summary(stdout: str) -> Dict[str, float]:
    m = SUMMARY_RE.search(stdout)
    if not m:
        return {}
    fields = {}
    for token in m.group(1).split():
        if "=" not in token:
            continue
        k, v = token.split("=", 1)
        try:
            fields[k] = float(v)
        except ValueError:
            pass
    return fields


def run_one(test_balance: Path, model: Path, duration: float, print_dt: float,
            key: str, k0: float, env_base: Dict[str, str]) -> RunResult:
    cmd = [
        str(test_balance),
        str(model),
        f"{duration:g}",
        "--print-dt",
        f"{print_dt:g}",
        "--key",
        key,
    ]
    env = dict(os.environ)
    env.update(env_base)
    env["SIM_K0_OVERRIDE"] = f"{k0:.9g}"
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    out = p.stdout + ("\n" + p.stderr if p.stderr else "")
    summary = parse_summary(out)
    return RunResult(k0=k0, key=key, rc=p.returncode, summary=summary)


def worst(results: Iterable[RunResult], field: str, abs_value: bool = False) -> float:
    vals: List[float] = []
    for r in results:
        if field not in r.summary:
            continue
        v = r.summary[field]
        vals.append(abs(v) if abs_value else v)
    return max(vals) if vals else math.inf


def main() -> int:
    args = parse_args()
    test_balance = Path(args.test_balance)
    model = Path(args.model)
    lut = Path(args.lut)
    run_csv = Path(args.run_csv)
    agg_csv = Path(args.agg_csv)
    print_dt = args.print_dt if args.print_dt is not None else args.duration

    if not test_balance.exists():
        raise SystemExit(f"test_balance not found: {test_balance}")
    if not model.exists():
        raise SystemExit(f"model not found: {model}")
    if args.keys:
        keys = [k.strip() for k in args.keys.split(",") if k.strip()]
    else:
        if not lut.exists():
            raise SystemExit(f"lut file not found (needed for key derivation): {lut}")
        keys = keys_from_lut(lut)
    if not keys:
        raise SystemExit("no keyframes specified or derived")

    k0_values = parse_float_list(args.k0_values)
    env_base: Dict[str, str] = {
        "SIM_MC_DEBUG": args.sim_mc_debug,
        "SIM_USE_EKF": args.sim_use_ekf,
    }

    print("K0 sweep configuration:")
    print(f"  test_balance: {test_balance}")
    print(f"  model:        {model}")
    print(f"  duration:     {args.duration:g} s")
    print(f"  keys:         {', '.join(keys)}")
    print(f"  k0 values:    {', '.join(f'{x:g}' for x in k0_values)}")
    print(f"  SIM_USE_EKF:  {args.sim_use_ekf} ({'EKF' if args.sim_use_ekf == '1' else 'ground-truth'})")
    print(f"  SIM_MC_DEBUG: {args.sim_mc_debug}")
    print("")

    run_results: List[RunResult] = []
    total = len(k0_values) * len(keys)
    idx = 0
    for k0 in k0_values:
        for key in keys:
            idx += 1
            print(f"[{idx:03d}/{total:03d}] K0={k0:+.6f} key={key} ...", flush=True)
            rr = run_one(test_balance, model, args.duration, print_dt, key, k0, env_base)
            run_results.append(rr)
            if rr.summary:
                fx = rr.summary.get("final_x", float("nan"))
                mx = rr.summary.get("max_x", float("nan"))
                th = rr.summary.get("max_theta_deg", float("nan"))
                dv = int(rr.summary.get("diverged", 1))
                ps = int(rr.summary.get("passed", 0))
                print(f"  rc={rr.rc} diverged={dv} passed={ps} final_x={fx:+.4f} max_x={mx:.4f} max_theta_deg={th:.3f}")
            else:
                print(f"  rc={rr.rc} (missing SUMMARY)")

    # Write per-run CSV
    run_fields = [
        "k0", "key", "rc",
        "max_theta_deg", "max_x", "max_y", "max_xdot", "max_ydot",
        "final_x", "final_y", "final_vx", "final_vy",
        "diverged", "passed",
    ]
    with run_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=run_fields)
        w.writeheader()
        for rr in run_results:
            row = {"k0": rr.k0, "key": rr.key, "rc": rr.rc}
            for k in run_fields:
                if k in ("k0", "key", "rc"):
                    continue
                row[k] = rr.summary.get(k, "")
            w.writerow(row)

    # Aggregate per K0
    agg_rows = []
    for k0 in k0_values:
        group = [r for r in run_results if r.k0 == k0]
        fail_count = 0
        for r in group:
            diverged = int(r.summary.get("diverged", 1))
            passed = int(r.summary.get("passed", 0))
            if r.rc != 0 or diverged != 0 or passed != 1:
                fail_count += 1
        row = {
            "k0": k0,
            "runs": len(group),
            "fail_count": fail_count,
            "worst_abs_final_x": worst(group, "final_x", abs_value=True),
            "worst_max_x": worst(group, "max_x", abs_value=False),
            "worst_max_theta_deg": worst(group, "max_theta_deg", abs_value=False),
            "worst_abs_final_y": worst(group, "final_y", abs_value=True),
        }
        agg_rows.append(row)

    with agg_csv.open("w", newline="") as f:
        fields = [
            "k0", "runs", "fail_count",
            "worst_abs_final_x", "worst_max_x", "worst_max_theta_deg", "worst_abs_final_y",
        ]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in agg_rows:
            w.writerow(row)

    # Rank: fail_count, worst_abs_final_x, worst_max_x, worst_max_theta_deg
    agg_rows_sorted = sorted(
        agg_rows,
        key=lambda r: (r["fail_count"], r["worst_abs_final_x"], r["worst_max_x"], r["worst_max_theta_deg"]),
    )
    best = agg_rows_sorted[0]

    print("\nAggregate ranking (top 5):")
    for row in agg_rows_sorted[:5]:
        print(
            f"  K0={row['k0']:+.6f}  fail={row['fail_count']}/{row['runs']}  "
            f"worst|final_x|={row['worst_abs_final_x']:.4f}  "
            f"worst max_x={row['worst_max_x']:.4f}  "
            f"worst max_theta_deg={row['worst_max_theta_deg']:.3f}"
        )

    print(
        f"\nBEST_K0 {best['k0']:+.6f} "
        f"fail_count={best['fail_count']} "
        f"worst_abs_final_x={best['worst_abs_final_x']:.6f} "
        f"worst_max_x={best['worst_max_x']:.6f} "
        f"worst_max_theta_deg={best['worst_max_theta_deg']:.6f}"
    )
    print(f"wrote: {run_csv}")
    print(f"wrote: {agg_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
