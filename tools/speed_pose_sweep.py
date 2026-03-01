#!/usr/bin/env python3
"""
Sweep hip keyframes and speed targets to find the pose that stabilizes at the highest speed.

Runs test_balance in speed-target mode, parses SUMMARY + SPEED_SUMMARY, and writes:
  - per-run CSV (one row per keyframe/speed target run)
  - per-key CSV (best stabilized speed per keyframe)

Console output includes a BEST_POSE line for quick consumption.
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
SPEED_RE = re.compile(r"^SPEED_SUMMARY\s+(.+)$", re.MULTILINE)
KEY_HIP_RE = re.compile(r"^eq_hip_([mp])(\d+)$")


@dataclass
class KeySpec:
    name: str
    hip: Optional[float] = None


@dataclass
class RunResult:
    key: KeySpec
    target_speed_mps: float
    rc: int
    summary: Dict[str, float]
    speed: Dict[str, float]

    def stabilized(self) -> bool:
        stabilized = int(round(self.speed.get("stabilized", 0.0)))
        diverged = int(round(self.summary.get("diverged", 0.0)))
        return self.rc == 0 and stabilized == 1 and diverged == 0

    def sustained_stable(self, duration_s: float, post_settle_hold_s: float) -> bool:
        if not self.stabilized():
            return False
        settle_time = to_float(self.speed.get("settle_time"), math.nan)
        if not math.isfinite(settle_time) or settle_time < 0.0:
            return False
        return (duration_s - settle_time) >= post_settle_hold_s


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Sweep keyframes/speeds and report best pose by max stabilized speed."
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

    speed = ap.add_argument_group("speed sweep")
    speed.add_argument(
        "--speed-values",
        default=None,
        help="Comma-separated speed targets (m/s). If set, --speed-min/max/step are ignored.",
    )
    speed.add_argument("--speed-min", type=float, default=0.2, help="Min target speed (m/s)")
    speed.add_argument("--speed-max", type=float, default=3.0, help="Max target speed (m/s)")
    speed.add_argument("--speed-step", type=float, default=0.2, help="Speed increment (m/s)")
    speed.add_argument(
        "--break-on-first-unstable",
        action="store_true",
        help="For each keyframe, stop after first unstable speed (expects ascending speed targets).",
    )

    run = ap.add_argument_group("test run settings")
    run.add_argument("--duration", type=float, default=8.0, help="Simulation duration per run (s)")
    run.add_argument(
        "--print-dt",
        type=float,
        default=None,
        help="Print cadence for test_balance (default: duration)",
    )
    run.add_argument("--speed-start-time", type=float, default=0.5, help="Speed command start time (s)")
    run.add_argument(
        "--speed-ramp-mps2",
        type=float,
        default=1.0,
        help="Speed command ramp (m/s^2). 0 means step command.",
    )
    run.add_argument(
        "--speed-settle-err-mps",
        type=float,
        default=0.05,
        help="Settle threshold on |vx-v_ref| (m/s)",
    )
    run.add_argument(
        "--speed-settle-hold",
        type=float,
        default=0.5,
        help="Required hold time at settle criteria (s)",
    )
    run.add_argument(
        "--post-settle-hold",
        type=float,
        default=2.0,
        help="Extra no-divergence hold required after settle to count speed as stable (s).",
    )
    run.add_argument(
        "--wheel-limit-nm",
        type=float,
        default=35.0,
        help="Wheel and LQR torque limit override in simulation (Nm)",
    )
    run.add_argument(
        "--lqr-v-ref-limit",
        type=float,
        default=None,
        help="LQR v_ref clamp (m/s). Default: auto-set from max tested speed (+0.25 m/s).",
    )
    run.add_argument(
        "--sim-use-ekf",
        default="0",
        choices=["0", "1"],
        help="Set SIM_USE_EKF for runs (default: 0, ground-truth path)",
    )
    run.add_argument(
        "--sim-mc-debug",
        default="0",
        choices=["0", "1"],
        help="Set SIM_MC_DEBUG for runs (default: 0)",
    )

    out = ap.add_argument_group("outputs")
    out.add_argument("--run-csv", default="speed_pose_sweep_runs.csv", help="Per-run output CSV")
    out.add_argument("--key-csv", default="speed_pose_sweep_key_summary.csv", help="Per-key summary CSV")
    out.add_argument(
        "--fail-on-errors",
        action="store_true",
        help="Exit non-zero if any run fails or missing SPEED_SUMMARY",
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


def parse_speed_values(args: argparse.Namespace) -> List[float]:
    if args.speed_values:
        vals = []
        for tok in args.speed_values.split(","):
            s = tok.strip()
            if not s:
                continue
            vals.append(float(s))
        if not vals:
            raise SystemExit("--speed-values was provided but no values were parsed")
        return vals

    if args.speed_step <= 0.0:
        raise SystemExit("--speed-step must be > 0")
    if args.speed_max < args.speed_min:
        raise SystemExit("--speed-max must be >= --speed-min")

    vals: List[float] = []
    v = args.speed_min
    eps = args.speed_step * 1e-6
    while v <= args.speed_max + eps:
        vals.append(round(v, 9))
        v += args.speed_step
    if not vals:
        raise SystemExit("generated empty speed list; check speed range args")
    return vals


def run_one(
    test_balance: Path,
    model: Path,
    key: KeySpec,
    speed_mps: float,
    args: argparse.Namespace,
    lqr_v_ref_limit: float,
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
        "--target-speed-mps",
        f"{speed_mps:g}",
        "--speed-start-time",
        f"{args.speed_start_time:g}",
        "--speed-ramp-mps2",
        f"{args.speed_ramp_mps2:g}",
        "--speed-settle-err-mps",
        f"{args.speed_settle_err_mps:g}",
        "--speed-settle-hold",
        f"{args.speed_settle_hold:g}",
        "--wheel-limit-nm",
        f"{args.wheel_limit_nm:g}",
        "--lqr-v-ref-limit",
        f"{lqr_v_ref_limit:g}",
    ]
    env = dict(os.environ)
    env.update(env_base)
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    output = p.stdout + ("\n" + p.stderr if p.stderr else "")
    summary = parse_fields_from_output(SUMMARY_RE, output)
    speed = parse_fields_from_output(SPEED_RE, output)
    return RunResult(key=key, target_speed_mps=speed_mps, rc=p.returncode, summary=summary, speed=speed)


def to_float(v: object, fallback: float = math.nan) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return fallback


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


def count_failures(results: Iterable[RunResult]) -> int:
    failures = 0
    for r in results:
        if r.rc != 0 or not r.speed:
            failures += 1
    return failures


def main() -> int:
    args = parse_args()
    test_balance = Path(args.test_balance)
    model = Path(args.model)
    run_csv = Path(args.run_csv)
    key_csv = Path(args.key_csv)

    if not test_balance.exists():
        raise SystemExit(f"test_balance not found: {test_balance}")
    if not model.exists():
        raise SystemExit(f"model not found: {model}")
    if args.duration <= 0.0:
        raise SystemExit("--duration must be > 0")
    if args.speed_settle_err_mps <= 0.0:
        raise SystemExit("--speed-settle-err-mps must be > 0")
    if args.speed_settle_hold <= 0.0:
        raise SystemExit("--speed-settle-hold must be > 0")
    if args.post_settle_hold < 0.0:
        raise SystemExit("--post-settle-hold must be >= 0")
    if args.speed_ramp_mps2 < 0.0:
        raise SystemExit("--speed-ramp-mps2 must be >= 0")
    if args.wheel_limit_nm < 0.0:
        raise SystemExit("--wheel-limit-nm must be >= 0")

    keys = build_keys(args)
    if not keys:
        raise SystemExit("no keyframes to run")
    speed_values = parse_speed_values(args)
    max_abs_speed = max(abs(v) for v in speed_values)
    lqr_v_ref_limit = (
        args.lqr_v_ref_limit
        if args.lqr_v_ref_limit is not None
        else max_abs_speed + 0.25
    )
    if lqr_v_ref_limit < 0.0:
        raise SystemExit("--lqr-v-ref-limit must be >= 0")

    env_base: Dict[str, str] = {
        "SIM_USE_EKF": args.sim_use_ekf,
        "SIM_MC_DEBUG": args.sim_mc_debug,
    }

    print("Speed/pose sweep configuration:")
    print(f"  test_balance:          {test_balance}")
    print(f"  model:                 {model}")
    print(f"  keys ({len(keys)}):            {', '.join(k.name for k in keys)}")
    print(f"  speed targets:         {', '.join(f'{v:g}' for v in speed_values)} m/s")
    print(f"  duration:              {args.duration:g} s")
    print(f"  speed_start_time:      {args.speed_start_time:g} s")
    print(f"  speed_ramp_mps2:       {args.speed_ramp_mps2:g}")
    print(f"  speed_settle_err_mps:  {args.speed_settle_err_mps:g}")
    print(f"  speed_settle_hold:     {args.speed_settle_hold:g} s")
    print(f"  post_settle_hold:      {args.post_settle_hold:g} s")
    print(f"  wheel_limit_nm:        {args.wheel_limit_nm:g}")
    print(f"  lqr_v_ref_limit:       {lqr_v_ref_limit:g} m/s")
    print(f"  env:                   SIM_USE_EKF={args.sim_use_ekf} SIM_MC_DEBUG={args.sim_mc_debug}")
    print(f"  break_on_first_unstable: {1 if args.break_on_first_unstable else 0}")

    all_runs: List[RunResult] = []
    per_key_rows: List[Dict[str, object]] = []

    for idx, key in enumerate(keys, start=1):
        print(f"\n[{idx}/{len(keys)}] key={key.name} hip={key.hip if key.hip is not None else float('nan'):.6f}")
        key_runs: List[RunResult] = []
        best_run: Optional[RunResult] = None

        for speed_mps in speed_values:
            rr = run_one(
                test_balance=test_balance,
                model=model,
                key=key,
                speed_mps=speed_mps,
                args=args,
                lqr_v_ref_limit=lqr_v_ref_limit,
                env_base=env_base,
            )
            all_runs.append(rr)
            key_runs.append(rr)

            stabilized = rr.stabilized()
            sustained_ok = rr.sustained_stable(args.duration, args.post_settle_hold)
            settle_delay = to_float(rr.speed.get("settle_delay"), math.nan)
            tau_settle = to_float(rr.speed.get("max_tau_to_settle"), math.nan)
            print(
                f"  speed={speed_mps:6.3f}  rc={rr.rc}  stabilized={1 if stabilized else 0}  sustained={1 if sustained_ok else 0}  "
                f"settle_delay={settle_delay:7.3f}  max_tau_to_settle={tau_settle:7.3f}"
            )

            if sustained_ok:
                if best_run is None or rr.target_speed_mps > best_run.target_speed_mps:
                    best_run = rr
            elif args.break_on_first_unstable:
                break

        stable_count = sum(1 for r in key_runs if r.stabilized())
        sustained_count = sum(1 for r in key_runs if r.sustained_stable(args.duration, args.post_settle_hold))
        tested_count = len(key_runs)
        if best_run is None:
            per_key_rows.append(
                {
                    "key": key.name,
                    "hip": key.hip,
                    "tested_count": tested_count,
                    "stable_count": stable_count,
                    "sustained_count": sustained_count,
                    "max_stable_target_speed_mps": math.nan,
                    "settle_delay_s": math.nan,
                    "max_tau_to_settle_nm": math.nan,
                    "final_speed_mps": math.nan,
                    "max_speed_mps": math.nan,
                }
            )
            print("  best_stable_speed=NA")
        else:
            per_key_rows.append(
                {
                    "key": key.name,
                    "hip": key.hip,
                    "tested_count": tested_count,
                    "stable_count": stable_count,
                    "sustained_count": sustained_count,
                    "max_stable_target_speed_mps": best_run.target_speed_mps,
                    "settle_delay_s": to_float(best_run.speed.get("settle_delay")),
                    "max_tau_to_settle_nm": to_float(best_run.speed.get("max_tau_to_settle")),
                    "final_speed_mps": to_float(best_run.speed.get("final_speed_mps")),
                    "max_speed_mps": to_float(best_run.speed.get("max_speed_mps")),
                }
            )
            print(
                "  best_stable_speed="
                f"{best_run.target_speed_mps:.3f} m/s "
                f"(settle_delay={to_float(best_run.speed.get('settle_delay')):.3f} s, "
                f"max_tau_to_settle={to_float(best_run.speed.get('max_tau_to_settle')):.3f} Nm)"
            )

    run_csv.parent.mkdir(parents=True, exist_ok=True)
    with run_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "key",
                "hip",
                "target_speed_mps",
                "rc",
                "stabilized",
                "sustained_ok",
                "passed",
                "diverged",
                "settle_time_s",
                "settle_delay_s",
                "max_tau_l_nm",
                "max_tau_r_nm",
                "max_tau_nm",
                "max_tau_to_settle_nm",
                "final_speed_mps",
                "max_speed_mps",
            ],
        )
        writer.writeheader()
        for rr in all_runs:
            writer.writerow(
                {
                    "key": rr.key.name,
                    "hip": rr.key.hip,
                    "target_speed_mps": rr.target_speed_mps,
                    "rc": rr.rc,
                    "stabilized": int(round(rr.speed.get("stabilized", 0.0))),
                    "sustained_ok": 1 if rr.sustained_stable(args.duration, args.post_settle_hold) else 0,
                    "passed": int(round(rr.summary.get("passed", 0.0))),
                    "diverged": int(round(rr.summary.get("diverged", 0.0))),
                    "settle_time_s": to_float(rr.speed.get("settle_time")),
                    "settle_delay_s": to_float(rr.speed.get("settle_delay")),
                    "max_tau_l_nm": to_float(rr.speed.get("max_tau_l")),
                    "max_tau_r_nm": to_float(rr.speed.get("max_tau_r")),
                    "max_tau_nm": to_float(rr.speed.get("max_tau")),
                    "max_tau_to_settle_nm": to_float(rr.speed.get("max_tau_to_settle")),
                    "final_speed_mps": to_float(rr.speed.get("final_speed_mps")),
                    "max_speed_mps": to_float(rr.speed.get("max_speed_mps")),
                }
            )

    key_csv.parent.mkdir(parents=True, exist_ok=True)
    with key_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "key",
                "hip",
                "tested_count",
                "stable_count",
                "sustained_count",
                "max_stable_target_speed_mps",
                "settle_delay_s",
                "max_tau_to_settle_nm",
                "final_speed_mps",
                "max_speed_mps",
            ],
        )
        writer.writeheader()
        for row in per_key_rows:
            writer.writerow(row)

    best_pose: Optional[Dict[str, object]] = None
    for row in per_key_rows:
        speed = to_float(row.get("max_stable_target_speed_mps"), math.nan)
        if not math.isfinite(speed):
            continue
        if best_pose is None:
            best_pose = row
            continue
        best_speed = to_float(best_pose.get("max_stable_target_speed_mps"), -math.inf)
        if speed > best_speed:
            best_pose = row
            continue
        if speed == best_speed:
            # Tie-breaker: lower settle delay, then lower torque.
            delay = to_float(row.get("settle_delay_s"), math.inf)
            best_delay = to_float(best_pose.get("settle_delay_s"), math.inf)
            if delay < best_delay:
                best_pose = row
                continue
            if delay == best_delay:
                tau = to_float(row.get("max_tau_to_settle_nm"), math.inf)
                best_tau = to_float(best_pose.get("max_tau_to_settle_nm"), math.inf)
                if tau < best_tau:
                    best_pose = row

    print("\n=== Sweep Summary ===")
    print(f"runs={len(all_runs)}  failures={count_failures(all_runs)}")
    print(f"run_csv={run_csv}")
    print(f"key_csv={key_csv}")
    if best_pose is None:
        print("BEST_POSE unavailable (no stabilized runs)")
    else:
        print(
            "BEST_POSE "
            f"key={best_pose['key']} hip={to_float(best_pose['hip']):.6f} "
            f"max_stable_target_speed_mps={to_float(best_pose['max_stable_target_speed_mps']):.6f} "
            f"settle_delay_s={to_float(best_pose['settle_delay_s']):.6f} "
            f"max_tau_to_settle_nm={to_float(best_pose['max_tau_to_settle_nm']):.6f}"
        )

    if args.fail_on_errors and (count_failures(all_runs) > 0):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
