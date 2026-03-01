#!/usr/bin/env python3
"""
Sweep disturbance recovery torque across hip keyframes using test_balance.

Runs test_balance with disturbance/recovery flags for each keyframe and parses:
  - SUMMARY
  - RECOVERY_SUMMARY

Outputs:
  - Per-key CSV with recovery metrics
  - Console summary including global max torque across all swept hip poses
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


SUMMARY_RE = re.compile(r"^SUMMARY\s+(.+)$", re.MULTILINE)
RECOVERY_RE = re.compile(r"^RECOVERY_SUMMARY\s+(.+)$", re.MULTILINE)
KEY_HIP_RE = re.compile(r"^eq_hip_([mp])(\d+)$")


@dataclass
class KeySpec:
    name: str
    hip: Optional[float] = None


@dataclass
class RunResult:
    key: KeySpec
    rc: int
    summary: Dict[str, float]
    recovery: Dict[str, float]


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Sweep disturbance recovery torque across hip keyframes."
    )
    ap.add_argument(
        "--test-balance",
        default="build/simulate_app/test_balance",
        help="Path to test_balance executable",
    )
    ap.add_argument("--model", default="myRobot/scene.xml", help="Model path for test_balance")
    ap.add_argument(
        "--keys",
        default=None,
        help="Comma-separated keyframe names (default: derive from --lut hip column)",
    )
    ap.add_argument(
        "--lut",
        default="lqr_lut.csv",
        help="LUT CSV used to derive keyframes when --keys is omitted",
    )
    ap.add_argument("--duration", type=float, default=8.0, help="Simulation duration per key (s)")
    ap.add_argument(
        "--print-dt",
        type=float,
        default=None,
        help="Print cadence for test_balance (default: duration)",
    )
    ap.add_argument("--theta-disturb-deg", type=float, default=5.0, help="Pitch disturbance (deg)")
    ap.add_argument("--disturb-time", type=float, default=0.5, help="Disturbance time (s)")
    ap.add_argument(
        "--wheel-limit-nm",
        type=float,
        default=35.0,
        help="Wheel and LQR torque limit override in simulation (Nm)",
    )
    ap.add_argument(
        "--settle-theta-err-deg",
        type=float,
        default=1.0,
        help="Settle threshold on |theta_err| (deg)",
    )
    ap.add_argument(
        "--settle-theta-dot",
        type=float,
        default=0.5,
        help="Settle threshold on |theta_dot| (rad/s)",
    )
    ap.add_argument("--settle-hold", type=float, default=0.5, help="Required hold time at settle criteria (s)")
    ap.add_argument(
        "--sim-use-ekf",
        default="0",
        choices=["0", "1"],
        help="Set SIM_USE_EKF for runs (default: 0, ground-truth path)",
    )
    ap.add_argument(
        "--sim-mc-debug",
        default="0",
        choices=["0", "1"],
        help="Set SIM_MC_DEBUG for runs (default: 0)",
    )
    ap.add_argument(
        "--csv-out",
        default="torque_sweep_results.csv",
        help="Per-key sweep results CSV",
    )
    ap.add_argument(
        "--fail-on-errors",
        action="store_true",
        help="Exit non-zero if any run fails or missing RECOVERY_SUMMARY",
    )
    return ap.parse_args()


def key_name_from_hip(hip: float) -> str:
    sign = "m" if hip < 0.0 else "p"
    mag = int(round(abs(hip) * 10000.0))
    return f"eq_hip_{sign}{mag:04d}"


def hip_from_key_name(key: str) -> Optional[float]:
    m = KEY_HIP_RE.match(key)
    if not m:
        return None
    sign, mag_str = m.groups()
    hip = float(mag_str) / 10000.0
    if sign == "m":
        hip = -hip
    return hip


def keys_from_lut(lut_path: Path) -> List[KeySpec]:
    keys: List[KeySpec] = []
    with lut_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if "hip" not in (reader.fieldnames or []):
            raise ValueError(f"{lut_path} is missing 'hip' column")
        for row in reader:
            hip = float(row["hip"])
            keys.append(KeySpec(name=key_name_from_hip(hip), hip=hip))
    dedup: List[KeySpec] = []
    seen = set()
    for k in keys:
        if k.name in seen:
            continue
        seen.add(k.name)
        dedup.append(k)
    return dedup


def parse_fields_from_output(regex: re.Pattern[str], output: str) -> Dict[str, float]:
    m = regex.search(output)
    if not m:
        return {}
    fields: Dict[str, float] = {}
    for token in m.group(1).split():
        if "=" not in token:
            continue
        k, v = token.split("=", 1)
        try:
            fields[k] = float(v)
        except ValueError:
            pass
    return fields


def run_one(
    test_balance: Path,
    model: Path,
    key: KeySpec,
    args: argparse.Namespace,
    env_base: Dict[str, str],
) -> RunResult:
    print_dt = args.print_dt if args.print_dt is not None else args.duration
    cmd = [
        str(test_balance),
        str(model),
        "--duration",
        f"{args.duration:g}",
        "--print-dt",
        f"{print_dt:g}",
        "--key",
        key.name,
        "--theta-disturb-deg",
        f"{args.theta_disturb_deg:g}",
        "--disturb-time",
        f"{args.disturb_time:g}",
        "--wheel-limit-nm",
        f"{args.wheel_limit_nm:g}",
        "--settle-theta-err-deg",
        f"{args.settle_theta_err_deg:g}",
        "--settle-theta-dot",
        f"{args.settle_theta_dot:g}",
        "--settle-hold",
        f"{args.settle_hold:g}",
    ]
    env = dict(os.environ)
    env.update(env_base)
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    output = p.stdout + ("\n" + p.stderr if p.stderr else "")
    summary = parse_fields_from_output(SUMMARY_RE, output)
    recovery = parse_fields_from_output(RECOVERY_RE, output)
    return RunResult(key=key, rc=p.returncode, summary=summary, recovery=recovery)


def to_float(v: object, fallback: float = math.nan) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return fallback


def best_by_metric(results: Sequence[RunResult], metric: str) -> Optional[Tuple[RunResult, float]]:
    best: Optional[Tuple[RunResult, float]] = None
    for r in results:
        val = to_float(r.recovery.get(metric, math.nan))
        if not math.isfinite(val):
            continue
        if best is None or val > best[1]:
            best = (r, val)
    return best


def count_failures(results: Iterable[RunResult]) -> int:
    failures = 0
    for r in results:
        if r.rc != 0 or not r.recovery:
            failures += 1
    return failures


def build_keys(args: argparse.Namespace) -> List[KeySpec]:
    if args.keys:
        out: List[KeySpec] = []
        for tok in args.keys.split(","):
            key = tok.strip()
            if not key:
                continue
            out.append(KeySpec(name=key, hip=hip_from_key_name(key)))
        return out

    lut_path = Path(args.lut)
    if not lut_path.exists():
        raise SystemExit(f"lut file not found (required when --keys omitted): {lut_path}")
    return keys_from_lut(lut_path)


def main() -> int:
    args = parse_args()
    test_balance = Path(args.test_balance)
    model = Path(args.model)
    csv_out = Path(args.csv_out)

    if not test_balance.exists():
        raise SystemExit(f"test_balance not found: {test_balance}")
    if not model.exists():
        raise SystemExit(f"model not found: {model}")

    keys = build_keys(args)
    if not keys:
        raise SystemExit("no keyframes to run")

    env_base: Dict[str, str] = {
        "SIM_USE_EKF": args.sim_use_ekf,
        "SIM_MC_DEBUG": args.sim_mc_debug,
    }

    print("Torque sweep configuration:")
    print(f"  test_balance:       {test_balance}")
    print(f"  model:              {model}")
    print(f"  keys:               {', '.join(k.name for k in keys)}")
    print(f"  duration:           {args.duration:g} s")
    print(f"  theta_disturb_deg:  {args.theta_disturb_deg:g}")
    print(f"  disturb_time:       {args.disturb_time:g} s")
    print(f"  wheel_limit_nm:     {args.wheel_limit_nm:g}")
    print(f"  settle_theta_err:   {args.settle_theta_err_deg:g} deg")
    print(f"  settle_theta_dot:   {args.settle_theta_dot:g} rad/s")
    print(f"  settle_hold:        {args.settle_hold:g} s")
    print(f"  SIM_USE_EKF:        {args.sim_use_ekf}")
    print(f"  SIM_MC_DEBUG:       {args.sim_mc_debug}")
    print("")

    results: List[RunResult] = []
    total = len(keys)
    for idx, key in enumerate(keys, start=1):
        hip_txt = f"{key.hip:+.4f}" if key.hip is not None else "n/a"
        print(f"[{idx:02d}/{total:02d}] key={key.name} hip={hip_txt} ...", flush=True)
        rr = run_one(test_balance, model, key, args, env_base)
        results.append(rr)

        if rr.recovery:
            stabilized = int(rr.recovery.get("stabilized", 0))
            max_tau = to_float(rr.recovery.get("max_tau"), math.nan)
            max_tau_settle = to_float(rr.recovery.get("max_tau_to_settle"), math.nan)
            settle_delay = to_float(rr.recovery.get("settle_delay"), math.nan)
            print(
                f"  rc={rr.rc} stabilized={stabilized} "
                f"max_tau={max_tau:.4f} max_tau_to_settle={max_tau_settle:.4f} "
                f"settle_delay={settle_delay:.4f}"
            )
        else:
            print(f"  rc={rr.rc} missing RECOVERY_SUMMARY")

    fields = [
        "key",
        "hip",
        "rc",
        "passed",
        "diverged",
        "max_theta_err_deg",
        "stabilized",
        "settle_time",
        "settle_delay",
        "max_tau_l",
        "max_tau_r",
        "max_tau",
        "max_tau_to_settle",
    ]
    with csv_out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for rr in results:
            row = {
                "key": rr.key.name,
                "hip": "" if rr.key.hip is None else f"{rr.key.hip:.6f}",
                "rc": rr.rc,
                "passed": rr.summary.get("passed", ""),
                "diverged": rr.summary.get("diverged", ""),
                "max_theta_err_deg": rr.summary.get("max_theta_err_deg", ""),
                "stabilized": rr.recovery.get("stabilized", ""),
                "settle_time": rr.recovery.get("settle_time", ""),
                "settle_delay": rr.recovery.get("settle_delay", ""),
                "max_tau_l": rr.recovery.get("max_tau_l", ""),
                "max_tau_r": rr.recovery.get("max_tau_r", ""),
                "max_tau": rr.recovery.get("max_tau", ""),
                "max_tau_to_settle": rr.recovery.get("max_tau_to_settle", ""),
            }
            writer.writerow(row)

    max_tau = best_by_metric(results, "max_tau")
    max_tau_to_settle = best_by_metric(results, "max_tau_to_settle")
    failures = count_failures(results)
    stabilized_count = sum(
        1 for r in results if int(to_float(r.recovery.get("stabilized"), 0.0)) == 1
    )

    if max_tau is not None:
        r, v = max_tau
        hip_txt = f"{r.key.hip:+.6f}" if r.key.hip is not None else "nan"
        print(f"MAX_TAU key={r.key.name} hip={hip_txt} max_tau={v:.6f}")
    else:
        print("MAX_TAU unavailable (no RECOVERY_SUMMARY values found)")

    if max_tau_to_settle is not None:
        r, v = max_tau_to_settle
        hip_txt = f"{r.key.hip:+.6f}" if r.key.hip is not None else "nan"
        print(f"MAX_TAU_TO_SETTLE key={r.key.name} hip={hip_txt} max_tau_to_settle={v:.6f}")
    else:
        print("MAX_TAU_TO_SETTLE unavailable (no RECOVERY_SUMMARY values found)")

    print(
        f"SWEEP_SUMMARY runs={len(results)} failures={failures} "
        f"stabilized={stabilized_count} csv={csv_out}"
    )

    if args.fail_on_errors and failures > 0:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

