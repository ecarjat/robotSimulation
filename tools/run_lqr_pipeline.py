#!/usr/bin/env python3
"""
Run the full simulation LQR pipeline:
  1) linearize_hip (with equilibrium/keyframe export)
  2) lqr_sweep (state-dim=3, include eq, no sign flip)
  3) per-hip K0 search via k0_sweep:
       coarse seeds + adaptive refinement rounds
  4) rewrite lqr_lut.csv K0 column from per-hip selected K0
  5) regenerate lqr_lut_data.h
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Sequence


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run linearize->lqr_sweep->per-hip k0_sweep->header pipeline.")
    p.add_argument("--linearize-bin", default="build/lqr_harness/linearize_hip",
                   help="Path to linearize_hip binary")
    p.add_argument("--model", default="myRobot/scene.xml", help="MuJoCo model path")
    p.add_argument("--write-keyframes", default="myRobot/keyframes_eq.xml",
                   help="Output equilibrium keyframes XML")
    p.add_argument("--out-dir", default="linearize_out", help="Linearization output directory")
    p.add_argument("--lut", default="lqr_lut.csv", help="LQR LUT CSV path")
    p.add_argument("--state-dim", type=int, default=3, choices=[3, 4],
                   help="State dimension passed to lqr_sweep")
    p.add_argument("--k0-duration", type=float, default=2.0,
                   help="Duration per per-hip K0 sweep run")
    p.add_argument("--k0-values", default="-100,-50,-1",
                   help="Comma-separated seed K0 candidates for per-hip search")
    p.add_argument("--k0-print-dt", type=float, default=2.0,
                   help="Print cadence for per-hip K0 sweep runs")
    p.add_argument("--k0-sim-use-ekf", choices=["0", "1"], default="0",
                   help="SIM_USE_EKF for k0_sweep (default: 0, ground-truth)")
    p.add_argument("--k0-runs-dir", default="linearize_out/k0_sweeps",
                   help="Directory to store per-hip k0_sweep CSV outputs")
    p.add_argument("--k0-adaptive", dest="k0_adaptive", action="store_true", default=True,
                   help="Enable adaptive K0 refinement around best coarse value (default: on)")
    p.add_argument("--no-k0-adaptive", dest="k0_adaptive", action="store_false",
                   help="Disable adaptive K0 refinement (single coarse sweep only)")
    p.add_argument("--k0-refine-rounds", type=int, default=2,
                   help="Adaptive refinement rounds per hip (default: 2)")
    p.add_argument("--k0-max-new-per-round", type=int, default=4,
                   help="Max number of newly proposed K0 candidates per refine round")
    p.add_argument("--k0-min", type=float, default=-200.0,
                   help="Clamp K0 proposals to this minimum value")
    p.add_argument("--k0-max", type=float, default=-0.01,
                   help="Clamp K0 proposals to this maximum value")
    p.add_argument("--k0-dup-eps", type=float, default=1e-6,
                   help="Duplicate tolerance for K0 values")
    p.add_argument("--header-out",
                   default="../stm32Controller/firmware/app/control/lqr_lut_data.h",
                   help="Output header path for lqr_lut_to_header.py")
    p.add_argument("--python", default=sys.executable,
                   help="Python executable used for sub-tools")
    p.add_argument("--no-sign-flip", action="store_true", default=True,
                   help="Pass --no-sign-flip to lqr_sweep (default: enabled)")
    p.add_argument("--sign-gains", default=None,
                   help="Optional --sign-gains value for lqr_sweep")
    p.add_argument("--low-hip-min", type=float, default=-0.270,
                   help="lqr_sweep low-hip shaping full-strength threshold")
    p.add_argument("--low-hip-max", type=float, default=-0.055,
                   help="lqr_sweep low-hip shaping taper-to-1.0 threshold")
    p.add_argument("--low-hip-k2-boost", type=float, default=2.0,
                   help="lqr_sweep low-hip K2 multiplier at full shaping weight")
    p.add_argument("--low-hip-k3-boost", type=float, default=1.5,
                   help="lqr_sweep low-hip K3 multiplier at full shaping weight")
    p.add_argument("--dry-run", action="store_true",
                   help="Print commands only; do not execute")
    return p.parse_args()


def run_cmd(cmd: List[str], dry_run: bool = False) -> str:
    printable = " ".join(cmd)
    print(f"$ {printable}")
    if dry_run:
        return ""
    proc = subprocess.run(cmd, text=True, capture_output=True)
    if proc.stdout:
        print(proc.stdout, end="")
    if proc.returncode != 0:
        if proc.stderr:
            print(proc.stderr, end="", file=sys.stderr)
        raise RuntimeError(f"Command failed ({proc.returncode}): {printable}")
    return proc.stdout


def parse_float_list(csv_values: str) -> List[float]:
    out: List[float] = []
    for tok in csv_values.split(","):
        tok = tok.strip()
        if not tok:
            continue
        out.append(float(tok))
    if not out:
        raise RuntimeError("empty float list")
    return out


def key_name_from_hip(hip: float) -> str:
    sign = "m" if hip < 0.0 else "p"
    mag = int(round(abs(hip) * 10000.0))
    return f"eq_hip_{sign}{mag:04d}"


def load_lut_rows(lut_path: Path) -> List[Dict[str, str]]:
    with lut_path.open("r", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"LUT is empty: {lut_path}")
    required = {"hip", "K0", "K1", "K2", "K3"}
    fields = set(rows[0].keys() or [])
    missing = sorted(required - fields)
    if missing:
        raise RuntimeError(f"LUT missing columns {missing}: {lut_path}")
    return rows


def load_run_csv_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="") as f:
        return list(csv.DictReader(f))


def _to_float(row: Dict[str, str], key: str, default: float) -> float:
    raw = row.get(key, "")
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _to_int(row: Dict[str, str], key: str, default: int) -> int:
    raw = row.get(key, "")
    if raw is None or raw == "":
        return default
    try:
        return int(float(raw))
    except ValueError:
        return default


def score_row(row: Dict[str, str]) -> tuple:
    rc = _to_int(row, "rc", 1)
    diverged = _to_int(row, "diverged", 1)
    passed = _to_int(row, "passed", 0)
    fail = 1 if (rc != 0 or diverged != 0 or passed != 1) else 0
    abs_final_x = abs(_to_float(row, "final_x", 1e9))
    max_x = abs(_to_float(row, "max_x", 1e9))
    max_theta_deg = abs(_to_float(row, "max_theta_deg", 1e9))
    abs_final_y = abs(_to_float(row, "final_y", 1e9))
    abs_final_vx = abs(_to_float(row, "final_vx", 1e9))
    return (fail, abs_final_x, max_x, max_theta_deg, abs_final_y, abs_final_vx)


def aggregate_k0_rows(rows: Sequence[Dict[str, str]]) -> List[Dict[str, float]]:
    grouped: Dict[float, Dict[str, float]] = {}
    for row in rows:
        k0 = _to_float(row, "k0", float("nan"))
        if not math.isfinite(k0):
            continue
        sc = score_row(row)
        entry = grouped.get(k0)
        if entry is None:
            entry = {
                "k0": k0,
                "runs": 0.0,
                "fail_count": 0.0,
                "worst_abs_final_x": 0.0,
                "worst_max_x": 0.0,
                "worst_max_theta_deg": 0.0,
                "worst_abs_final_y": 0.0,
                "worst_abs_final_vx": 0.0,
            }
            grouped[k0] = entry
        entry["runs"] += 1.0
        entry["fail_count"] += float(sc[0])
        entry["worst_abs_final_x"] = max(entry["worst_abs_final_x"], sc[1])
        entry["worst_max_x"] = max(entry["worst_max_x"], sc[2])
        entry["worst_max_theta_deg"] = max(entry["worst_max_theta_deg"], sc[3])
        entry["worst_abs_final_y"] = max(entry["worst_abs_final_y"], sc[4])
        entry["worst_abs_final_vx"] = max(entry["worst_abs_final_vx"], sc[5])
    return list(grouped.values())


def rank_k0_summary(summary: Dict[str, float]) -> tuple:
    return (
        int(summary["fail_count"]),
        summary["worst_abs_final_x"],
        summary["worst_max_x"],
        summary["worst_max_theta_deg"],
        summary["worst_abs_final_y"],
        summary["worst_abs_final_vx"],
        abs(summary["k0"]),
    )


def select_best_k0(rows: Sequence[Dict[str, str]]) -> Dict[str, float]:
    summaries = aggregate_k0_rows(rows)
    if not summaries:
        raise RuntimeError("No valid K0 rows available for selection")
    best = min(summaries, key=rank_k0_summary)
    has_passing = any(int(s["fail_count"]) == 0 for s in summaries)
    if has_passing and int(best["fail_count"]) != 0:
        raise RuntimeError(
            "Internal K0 selection error: selected a failing K0 while passing K0 candidates exist"
        )
    return best


def same_k0(a: float, b: float, eps: float) -> bool:
    scale = max(1.0, abs(a), abs(b))
    return abs(a - b) <= eps * scale


def dedup_k0(values: Sequence[float], eps: float) -> List[float]:
    out: List[float] = []
    for v in values:
        if any(same_k0(v, x, eps) for x in out):
            continue
        out.append(v)
    return out


def clamp_k0(v: float, lo: float, hi: float) -> float:
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def propose_refine_candidates(best_k0: float,
                              tested: Sequence[float],
                              lo: float,
                              hi: float,
                              eps: float,
                              max_new: int) -> List[float]:
    tested_sorted = sorted(tested)
    if not tested_sorted:
        return []

    best_idx = min(range(len(tested_sorted)), key=lambda i: abs(tested_sorted[i] - best_k0))
    best = tested_sorted[best_idx]
    left = tested_sorted[best_idx - 1] if best_idx > 0 else None
    right = tested_sorted[best_idx + 1] if best_idx + 1 < len(tested_sorted) else None

    proposals: List[float] = []

    def add_between(a: float, b: float) -> None:
        mid_lin = 0.5 * (a + b)
        proposals.append(mid_lin)
        if a * b > 0.0:
            mid_geo = math.copysign(math.sqrt(abs(a * b)), a)
            proposals.append(mid_geo)

    if left is not None:
        add_between(left, best)
    else:
        proposals.append(best * 2.0)
        proposals.append(best * 5.0)

    if right is not None:
        add_between(best, right)
    else:
        proposals.append(best * 0.5)
        proposals.append(best * 0.2)

    # Keep proposals near the best first, then deduplicate and clamp.
    proposals = sorted(proposals, key=lambda v: abs(v - best))
    clamped = [clamp_k0(v, lo, hi) for v in proposals]
    clamped = dedup_k0(clamped, eps)
    out: List[float] = []
    for v in clamped:
        if any(same_k0(v, t, eps) for t in tested):
            continue
        out.append(v)
        if len(out) >= max_new:
            break
    return out


def write_k0_selection_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    fields = [
        "hip", "key", "best_k0", "eval_count", "runs", "fail", "fail_count", "abs_final_x", "max_x",
        "max_theta_deg", "abs_final_y", "abs_final_vx",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_updated_lut(lut_path: Path, rows: List[Dict[str, str]], best_by_key: Dict[str, float]) -> None:
    fieldnames = list(rows[0].keys())
    for row in rows:
        hip = float(row["hip"])
        key = key_name_from_hip(hip)
        if key not in best_by_key:
            raise RuntimeError(f"Missing selected K0 for key '{key}' (hip={hip})")
        row["K0"] = f"{best_by_key[key]:.9g}"
    with lut_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    args = parse_args()
    py = args.python
    linearize_bin = Path(args.linearize_bin)
    model = Path(args.model)
    write_keyframes = Path(args.write_keyframes)
    out_dir = Path(args.out_dir)
    lut = Path(args.lut)
    k0_runs_dir = Path(args.k0_runs_dir)
    header_out = Path(args.header_out)

    if not args.dry_run and not linearize_bin.exists():
        raise RuntimeError(f"linearize_hip binary not found: {linearize_bin}")
    if not args.dry_run and not model.exists():
        raise RuntimeError(f"model not found: {model}")

    k0_min = min(args.k0_min, args.k0_max)
    k0_max = max(args.k0_min, args.k0_max)
    seed_k0_values = [clamp_k0(v, k0_min, k0_max)
                      for v in parse_float_list(args.k0_values)]
    seed_k0_values = dedup_k0(seed_k0_values, args.k0_dup_eps)
    if not seed_k0_values:
        raise RuntimeError("no usable seed K0 values after clamping/dedup")

    k0_runs_dir.mkdir(parents=True, exist_ok=True)

    # 1) linearize_hip
    linearize_cmd = [
        str(linearize_bin),
        "--model", str(model),
        "--equilibrium-wheels",
        "--write-keyframes", str(write_keyframes),
        "--out", str(out_dir),
        "--reduced-no-x",
    ]
    run_cmd(linearize_cmd, dry_run=args.dry_run)

    # 2) lqr_sweep
    lqr_sweep_cmd = [
        py, "tools/lqr_sweep.py",
        "--dir", str(out_dir),
        "--state-dim", str(args.state_dim),
        "--include-eq",
        "--low-hip-min", f"{args.low_hip_min:g}",
        "--low-hip-max", f"{args.low_hip_max:g}",
        "--low-hip-k2-boost", f"{args.low_hip_k2_boost:g}",
        "--low-hip-k3-boost", f"{args.low_hip_k3_boost:g}",
        "--lut-out", str(lut),
    ]
    if args.no_sign_flip:
        lqr_sweep_cmd.append("--no-sign-flip")
    if args.sign_gains:
        lqr_sweep_cmd.extend(["--sign-gains", args.sign_gains])
    run_cmd(lqr_sweep_cmd, dry_run=args.dry_run)

    if args.dry_run:
        print("Dry-run complete.")
        return 0

    # 3) per-hip k0 sweep
    rows = load_lut_rows(lut)
    best_by_key: Dict[str, float] = {}
    selection_rows: List[Dict[str, object]] = []
    for row in rows:
        hip = float(row["hip"])
        key = key_name_from_hip(hip)
        print(f"\n=== K0 search for {key} (hip={hip:+.6f}) ===")
        tested_rows: List[Dict[str, str]] = []
        tested_k0: List[float] = []
        pending = list(seed_k0_values)
        round_idx = 0
        max_rounds = max(0, int(args.k0_refine_rounds))

        while pending:
            pending = dedup_k0([clamp_k0(v, k0_min, k0_max) for v in pending], args.k0_dup_eps)
            pending = [v for v in pending if not any(same_k0(v, t, args.k0_dup_eps) for t in tested_k0)]
            if not pending:
                break

            k0_values_arg = ",".join(f"{v:.12g}" for v in pending)
            run_csv = k0_runs_dir / f"k0_sweep_runs_{key}_r{round_idx}.csv"
            agg_csv = k0_runs_dir / f"k0_sweep_agg_{key}_r{round_idx}.csv"
            cmd = [
                py, "tools/k0_sweep.py",
                "--duration", f"{args.k0_duration:g}",
                "--print-dt", f"{args.k0_print_dt:g}",
                f"--k0-values={k0_values_arg}",
                "--keys", key,
                "--run-csv", str(run_csv),
                "--agg-csv", str(agg_csv),
                "--sim-use-ekf", args.k0_sim_use_ekf,
            ]
            run_cmd(cmd, dry_run=False)
            stage_rows = load_run_csv_rows(run_csv)
            if not stage_rows:
                raise RuntimeError(f"k0_sweep returned no rows for {key} round {round_idx}")
            tested_rows.extend(stage_rows)
            for sr in stage_rows:
                tested_k0.append(_to_float(sr, "k0", float("nan")))

            best_summary = select_best_k0(tested_rows)
            best_k0 = best_summary["k0"]
            print(
                f"Round {round_idx}: tested={len(stage_rows)} total={len(tested_rows)} "
                f"best_k0={best_k0:+.6f} "
                f"fail_count={int(best_summary['fail_count'])}/{int(best_summary['runs'])} "
                f"score={(int(best_summary['fail_count']), best_summary['worst_abs_final_x'], best_summary['worst_max_x'], best_summary['worst_max_theta_deg'], best_summary['worst_abs_final_y'], best_summary['worst_abs_final_vx'])}"
            )

            if not args.k0_adaptive or round_idx >= max_rounds:
                break

            pending = propose_refine_candidates(
                best_k0=best_k0,
                tested=tested_k0,
                lo=k0_min,
                hi=k0_max,
                eps=args.k0_dup_eps,
                max_new=max(1, int(args.k0_max_new_per_round)),
            )
            round_idx += 1

        if not tested_rows:
            raise RuntimeError(f"No K0 evaluation rows collected for {key}")

        best_summary = select_best_k0(tested_rows)
        best_k0 = best_summary["k0"]
        best_by_key[key] = best_k0
        selection_rows.append({
            "hip": hip,
            "key": key,
            "best_k0": best_k0,
            "eval_count": len(tested_rows),
            "runs": int(best_summary["runs"]),
            "fail": 1 if int(best_summary["fail_count"]) != 0 else 0,
            "fail_count": int(best_summary["fail_count"]),
            "abs_final_x": best_summary["worst_abs_final_x"],
            "max_x": best_summary["worst_max_x"],
            "max_theta_deg": best_summary["worst_max_theta_deg"],
            "abs_final_y": best_summary["worst_abs_final_y"],
            "abs_final_vx": best_summary["worst_abs_final_vx"],
        })
        if int(best_summary["fail_count"]) != 0:
            print(
                f"Selected K0 for {key}: {best_k0:+.6f} after {len(tested_rows)} evaluations "
                f"(WARNING: no passing candidate found)"
            )
        else:
            print(f"Selected K0 for {key}: {best_k0:+.6f} after {len(tested_rows)} evaluations")

    # 4) rewrite LUT K0 values
    write_updated_lut(lut, rows, best_by_key)
    print(f"Updated K0 column in {lut}")
    selection_csv = k0_runs_dir / "k0_selected.csv"
    write_k0_selection_csv(selection_csv, selection_rows)
    print(f"Wrote K0 selection summary: {selection_csv}")

    # 5) generate firmware header
    header_cmd = [
        py, "tools/lqr_lut_to_header.py",
        "--out", str(header_out),
        str(lut),
    ]
    run_cmd(header_cmd, dry_run=False)

    print("Pipeline complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
