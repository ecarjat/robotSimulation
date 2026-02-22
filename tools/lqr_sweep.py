#!/usr/bin/env python3
import argparse
import glob
import math
import os
from pathlib import Path

import numpy as np


def load_csv_matrix(path):
    return np.loadtxt(path, delimiter=",")


def parse_hip_from_name(path):
    # expects .../Ared_hip_0.741678.csv
    name = Path(path).stem
    parts = name.split("_")
    try:
        idx = parts.index("hip")
        return float(parts[idx + 1])
    except Exception:
        # fallback: last token
        try:
            return float(parts[-1])
        except Exception:
            return None


def dare_iterative(A, B, Q, R, max_iter=2000, tol=1e-10):
    P = Q.copy()
    for _ in range(max_iter):
        BT_P = B.T @ P
        S = R + BT_P @ B
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return None
        P_next = A.T @ P @ A - A.T @ P @ B @ S_inv @ BT_P @ A + Q
        if np.max(np.abs(P_next - P)) < tol:
            return P_next
        P = P_next
    return P


def dlqr(A, B, Q, R):
    P = dare_iterative(A, B, Q, R)
    if P is None:
        return None
    S = R + B.T @ P @ B
    try:
        K = np.linalg.inv(S) @ (B.T @ P @ A)
    except np.linalg.LinAlgError:
        return None
    return K


def spectral_radius(A):
    vals = np.linalg.eigvals(A)
    return max(abs(vals))


def clamp01(v):
    return max(0.0, min(1.0, v))


def apply_low_hip_gain_shape(K, hip, state_dim, low_hip_min, low_hip_max,
                             k2_boost, k3_boost):
    if abs(k2_boost - 1.0) < 1e-12 and abs(k3_boost - 1.0) < 1e-12:
        return K

    if state_dim == 3:
        k2_idx = 1
        k3_idx = 2
    else:
        k2_idx = 2
        k3_idx = 3

    if hip > low_hip_max:
        return K

    if low_hip_max <= low_hip_min:
        w = 1.0
    else:
        w = clamp01((low_hip_max - hip) / (low_hip_max - low_hip_min))
    if w <= 0.0:
        return K

    K_shaped = K.copy()
    k2_scale = 1.0 + w * (k2_boost - 1.0)
    k3_scale = 1.0 + w * (k3_boost - 1.0)
    K_shaped[k2_idx] *= k2_scale
    K_shaped[k3_idx] *= k3_scale
    return K_shaped


def main():
    ap = argparse.ArgumentParser(
        description="Sweep Q/R for LQR across hip angles and pick stable/damped candidate."
    )
    ap.add_argument("--dir", default="linearize_out", help="Directory with Ared_hip_*.csv/Bred_hip_*.csv")
    ap.add_argument("--state-dim", type=int, default=4, choices=[3, 4],
                    help="Reduced state dimension (4 or 3). 3 uses Ared3/Bred3.")
    ap.add_argument("--include-eq", action="store_true",
                    help="If eq_hip_*.csv exists, include theta_eq and u_eq in LUT output.")
    ap.add_argument("--eq-dir", default=None,
                    help="Directory containing eq_hip_*.csv (default: --dir)")
    ap.add_argument("--eq-glob", default="eq_hip_*.csv",
                    help="Glob pattern for equilibrium files (default: eq_hip_*.csv)")
    ap.add_argument("--eq-tol", type=float, default=1e-6,
                    help="Tolerance for matching eq_hip values to hip keys (default: 1e-6)")
    ap.add_argument("--require-eq", action="store_true",
                    help="Fail if no eq_hip data is found for any hip.")
    ap.add_argument("--qx", type=float, default=0.0, help="Base Q weight for x")
    ap.add_argument("--qv", type=float, default=1.0, help="Base Q weight for xdot")
    ap.add_argument("--qt", type=float, default=500.0, help="Base Q weight for theta")
    ap.add_argument("--qtd", type=float, default=50.0, help="Base Q weight for thetadot")
    ap.add_argument("--q-scale", type=str, default="0.1,0.3,1,3,10",
                    help="Comma-separated Q scale factors")
    ap.add_argument("--r-scale", type=str, default="0.1,0.3,1,3,10",
                    help="Comma-separated R scale factors")
    ap.add_argument("--out", default="lqr_sweep_results.csv", help="Output CSV summary")
    ap.add_argument("--lut-out", default="lqr_lut.csv", help="Output LUT CSV (best candidate)")
    ap.add_argument("--diag", action="store_true", help="Print diagnostics per hip (open-loop rho, B norm, controllability rank)")
    ap.add_argument("--u-scale", type=str, default="1",
                    help="Comma-separated input scaling factors applied to B (u' = u / scale)")
    ap.add_argument("--rho-tol", type=float, default=1e-6,
                    help="Tolerance for marginal stability check (default: 1e-6)")
    ap.add_argument("--rho-target", type=float, default=None,
                    help="Require max spectral radius <= this value (e.g., 0.98)")
    ap.add_argument("--allow-marginal", action="store_true",
                    help="Allow one eigenvalue within tol of 1.0 (position integrator).")
    ap.add_argument("--strict-stability", action="store_true",
                    help="Disable automatic marginal-mode allowance for cascaded K0 with qx=0.")
    ap.add_argument("--best-effort", action="store_true",
                    help="Always emit best candidate (min max_rho) even if unstable.")
    ap.add_argument("--no-sign-flip", action="store_true",
                    help="Reject candidates whose selected gains change sign across hips.")
    ap.add_argument("--sign-gains", type=str, default="K1,K2,K3",
                    help="Comma-separated gains to enforce sign consistency on (valid: K0,K1,K2,K3).")
    ap.add_argument("--sign-eps", type=float, default=1e-9,
                    help="Treat |gain| <= sign_eps as zero when checking sign consistency.")
    ap.add_argument(
        "--cascaded-k0",
        action="store_true",
        help=(
            "Export K0 mapped for the cascaded MotionController architecture "
            "(v_ref_from_pos = K0 * x_err). For state-dim=4, map direct LQR "
            "K0_direct to K0_cascaded = -K0_direct / K1_direct."
        ),
    )
    ap.add_argument("--low-hip-min", type=float, default=-0.270,
                    help="Hip angle where low-hip gain shaping is full-strength.")
    ap.add_argument("--low-hip-max", type=float, default=-0.055,
                    help="Hip angle where low-hip gain shaping tapers to 1.0.")
    ap.add_argument("--low-hip-k2-boost", type=float, default=1.0,
                    help="Multiplier for K2 at full low-hip shaping weight.")
    ap.add_argument("--low-hip-k3-boost", type=float, default=1.0,
                    help="Multiplier for K3 at full low-hip shaping weight.")
    args = ap.parse_args()

    if args.low_hip_k2_boost <= 0.0 or args.low_hip_k3_boost <= 0.0:
        raise SystemExit("--low-hip-k2-boost and --low-hip-k3-boost must be > 0")
    shape_active = (abs(args.low_hip_k2_boost - 1.0) > 1e-12 or
                    abs(args.low_hip_k3_boost - 1.0) > 1e-12)
    if shape_active:
        print(
            "Info: low-hip LUT shaping enabled: "
            f"hip in [{args.low_hip_min:.6f}, {args.low_hip_max:.6f}] "
            f"K2x={args.low_hip_k2_boost:.6f} K3x={args.low_hip_k3_boost:.6f}"
        )

    if (args.cascaded_k0
            and args.state_dim == 4
            and abs(args.qx) < 1e-12
            and not args.allow_marginal
            and not args.strict_stability):
        args.allow_marginal = True
        print("Info: enabling marginal stability allowance (qx=0 with --cascaded-k0).")

    if args.state_dim == 3:
        a_glob = "Ared3_hip_*.csv"
        b_glob = "Bred3_hip_*.csv"
        available_gain_names = ["K1", "K2", "K3"]
    else:
        a_glob = "Ared_hip_*.csv"
        b_glob = "Bred_hip_*.csv"
        available_gain_names = ["K0", "K1", "K2", "K3"]

    sign_gain_indices = []
    if args.no_sign_flip:
        requested = [x.strip().upper() for x in args.sign_gains.split(",") if x.strip()]
        if not requested:
            raise SystemExit("--no-sign-flip requires at least one gain in --sign-gains")
        name_to_idx = {name: i for i, name in enumerate(available_gain_names)}
        invalid = [name for name in requested if name not in name_to_idx]
        if invalid:
            valid = ",".join(available_gain_names)
            raise SystemExit(
                f"Invalid --sign-gains entry: {','.join(invalid)} "
                f"(state-dim={args.state_dim}, valid: {valid})"
            )
        sign_gain_indices = [name_to_idx[name] for name in requested]
    a_files = sorted(glob.glob(os.path.join(args.dir, a_glob)))
    b_files = sorted(glob.glob(os.path.join(args.dir, b_glob)))
    if not a_files or not b_files:
        raise SystemExit("No Ared_hip_*.csv / Bred_hip_*.csv found.")

    # map hip -> matrices
    mats = {}
    for a_path in a_files:
        hip = parse_hip_from_name(a_path)
        if hip is None:
            continue
        b_path = os.path.join(args.dir, f"{'Bred3' if args.state_dim == 3 else 'Bred'}_hip_{hip:.6f}.csv")
        if not os.path.exists(b_path):
            # fallback: first match by prefix
            candidates = [p for p in b_files if f"hip_{hip:.6f}" in p]
            if not candidates:
                continue
            b_path = candidates[0]
        A = load_csv_matrix(a_path)
        B = load_csv_matrix(b_path)
        if B.ndim == 1:
            B = B.reshape(-1, 1)
        mats[hip] = (A, B)

    q_scales = [float(x) for x in args.q_scale.split(",") if x.strip()]
    r_scales = [float(x) for x in args.r_scale.split(",") if x.strip()]
    u_scales = [float(x) for x in args.u_scale.split(",") if x.strip()]

    # load equilibrium (theta_eq, u_eq) if present
    eq_dir = args.eq_dir or args.dir
    eq_list = []
    eq_files = sorted(glob.glob(os.path.join(eq_dir, args.eq_glob)))
    for path in eq_files:
        hip = parse_hip_from_name(path)
        if hip is None:
            continue
        try:
            with open(path, "r") as f:
                lines = [l.strip() for l in f.readlines() if l.strip()]
            if len(lines) < 2:
                continue
            parts = lines[1].split(",")
            if len(parts) < 3:
                continue
            theta_eq = float(parts[1])
            u_eq = float(parts[2])
            eq_list.append((hip, theta_eq, u_eq))
        except Exception:
            continue

    def match_eq(hip_value):
        if not eq_list:
            return None
        best = None
        best_err = None
        for hip, theta_eq, u_eq in eq_list:
            err = abs(hip - hip_value)
            if best_err is None or err < best_err:
                best_err = err
                best = (theta_eq, u_eq)
        if best_err is not None and best_err <= args.eq_tol:
            return best
        return None

    results = []
    best = None
    best_any = None

    for qs in q_scales:
        if args.state_dim == 3:
            Q = np.diag([args.qv, args.qt, args.qtd]) * qs
        else:
            Q = np.diag([args.qx, args.qv, args.qt, args.qtd]) * qs
        for rs in r_scales:
            R = np.array([[rs]])
            for us in u_scales:
                stable_all = True
                max_rho = 0.0
                max_score_rho = 0.0
                sign_consistent = True
                k_samples = []
                for hip, (A, B) in mats.items():
                    B_eff = B * us
                    Kp = dlqr(A, B_eff, Q, R)
                    if Kp is None:
                        stable_all = False
                        break
                    K = Kp * us  # back to real input u
                    K = apply_low_hip_gain_shape(
                        K.flatten(), hip, args.state_dim,
                        args.low_hip_min, args.low_hip_max,
                        args.low_hip_k2_boost, args.low_hip_k3_boost
                    ).reshape(K.shape)
                    k_samples.append((hip, K.flatten()))
                    Acl = A - B @ K
                    eig = np.linalg.eigvals(Acl)
                    abs_eig = np.array([abs(v) for v in eig], dtype=float)
                    rho = float(np.max(abs_eig))
                    max_rho = max(max_rho, rho)
                    if args.rho_target is not None and rho > args.rho_target:
                        stable_all = False
                        break
                    if args.allow_marginal:
                        # Allow at most one eigenvalue slightly above 1.0 within tolerance.
                        above = int(np.sum(abs_eig > 1.0 + args.rho_tol))
                        near_mask = np.logical_and(abs_eig >= 1.0 - args.rho_tol, abs_eig <= 1.0 + args.rho_tol)
                        near = int(np.sum(near_mask))
                        if above > 0:
                            stable_all = False
                            break
                        if near > 1:
                            stable_all = False
                            break
                        if near >= 1:
                            # Drop one marginal mode (typically x-position integrator)
                            marginal_idx = int(np.argmin(np.abs(abs_eig - 1.0)))
                            score_eigs = np.delete(abs_eig, marginal_idx)
                        else:
                            score_eigs = abs_eig
                        score_rho = float(np.max(score_eigs)) if score_eigs.size else 0.0
                    else:
                        if rho >= 1.0:
                            stable_all = False
                            break
                        score_rho = rho
                    max_score_rho = max(max_score_rho, score_rho)
                if stable_all and args.no_sign_flip:
                    # Enforce consistent sign for selected gains across all hips.
                    for gain_idx in sign_gain_indices:
                        ref_sign = 0
                        for _, kv in sorted(k_samples, key=lambda x: x[0]):
                            val = float(kv[gain_idx])
                            if abs(val) <= args.sign_eps:
                                continue
                            sgn = 1 if val > 0.0 else -1
                            if ref_sign == 0:
                                ref_sign = sgn
                            elif sgn != ref_sign:
                                sign_consistent = False
                                break
                        if not sign_consistent:
                            break
                    if not sign_consistent:
                        stable_all = False

                results.append((qs, rs, us, stable_all, max_rho, max_score_rho, sign_consistent))

                # best_any is fallback for --best-effort. If sign consistency is requested,
                # keep fallback within sign-consistent candidates.
                if (not args.no_sign_flip) or sign_consistent:
                    if best_any is None or max_score_rho < best_any[5]:
                        best_any = (qs, rs, us, stable_all, max_rho, max_score_rho, sign_consistent)
                if stable_all:
                    if best is None or max_score_rho < best[5]:
                        best = (qs, rs, us, stable_all, max_rho, max_score_rho, sign_consistent)

    if args.diag:
        print("Diagnostics per hip:")
        for hip in sorted(mats.keys()):
            A, B = mats[hip]
            rho = spectral_radius(A)
            bnorm = float(np.linalg.norm(B))
            # controllability rank
            n = A.shape[0]
            ctrb = B
            for i in range(1, n):
                ctrb = np.hstack((ctrb, np.linalg.matrix_power(A, i) @ B))
            rank = np.linalg.matrix_rank(ctrb)
            print(f"hip={hip:.6f}  rho(A)={rho:.6f}  ||B||={bnorm:.6e}  rank(C)={rank}")

    # write summary
    with open(args.out, "w") as f:
        f.write("q_scale,r_scale,u_scale,stable_all,max_spectral_radius,max_selection_radius,sign_consistent\n")
        for qs, rs, us, stable_all, max_rho, max_score_rho, sign_consistent in results:
            f.write(
                f"{qs},{rs},{us},{int(stable_all)},{max_rho:.6f},"
                f"{max_score_rho:.6f},{int(sign_consistent)}\n"
            )

    if best is None:
        print("No stable Q/R found. Check ranges.")
        if not args.best_effort or best_any is None:
            return
        best = best_any

    qs, rs, us, stable, max_rho, max_score_rho, sign_consistent = best
    status = "stable" if stable else "best-effort"
    print(
        f"Best ({status}): q_scale={qs}, r_scale={rs}, u_scale={us}, "
        f"max_rho={max_rho:.6f}, selection_rho={max_score_rho:.6f}, "
        f"sign_consistent={int(sign_consistent)}"
    )

    # write LUT for best candidate
    if args.state_dim == 3:
        Q = np.diag([args.qv, args.qt, args.qtd]) * qs
    else:
        Q = np.diag([args.qx, args.qv, args.qt, args.qtd]) * qs
    R = np.array([[rs]])
    include_eq = args.include_eq or (len(eq_list) > 0)
    with open(args.lut_out, "w") as f:
        if include_eq:
            f.write("hip,K0,K1,K2,K3,theta_eq,u_eq\n")
        else:
            f.write("hip,K0,K1,K2,K3\n")
        for hip in sorted(mats.keys()):
            A, B = mats[hip]
            B_eff = B * us
            Kp = dlqr(A, B_eff, Q, R)
            if Kp is None:
                continue
            K = (Kp * us).flatten()
            K = apply_low_hip_gain_shape(
                K, hip, args.state_dim,
                args.low_hip_min, args.low_hip_max,
                args.low_hip_k2_boost, args.low_hip_k3_boost
            )
            if args.cascaded_k0 and args.state_dim == 4:
                # MotionController cascaded form uses:
                #   v_ref_from_pos = K0 * x_err
                #   u_vel_term = -K1 * (v - v_ref)
                # so the effective x gain is +K1*K0. To match direct LQR:
                #   u_direct includes -K0_direct * x_err
                # therefore K0_cascaded = -K0_direct / K1_direct.
                k1 = K[1]
                if abs(k1) > 1e-9:
                    K[0] = -K[0] / k1
                else:
                    K[0] = 0.0
            eq = match_eq(hip)
            if eq is None:
                if args.require_eq:
                    raise SystemExit(f"Missing eq_hip for hip={hip:.6f}")
                theta_eq, u_eq = (0.0, 0.0)
            else:
                theta_eq, u_eq = eq
            theta_fmt = f"{theta_eq:.9e}"
            u_fmt = f"{u_eq:.9e}"
            if args.state_dim == 3:
                if include_eq:
                    f.write(f"{hip:.6f},0.000000,{K[0]:.6f},{K[1]:.6f},{K[2]:.6f},{theta_fmt},{u_fmt}\n")
                else:
                    f.write(f"{hip:.6f},0.000000,{K[0]:.6f},{K[1]:.6f},{K[2]:.6f}\n")
            else:
                if include_eq:
                    f.write(f"{hip:.6f},{K[0]:.6f},{K[1]:.6f},{K[2]:.6f},{K[3]:.6f},{theta_fmt},{u_fmt}\n")
                else:
                    f.write(f"{hip:.6f},{K[0]:.6f},{K[1]:.6f},{K[2]:.6f},{K[3]:.6f}\n")


if __name__ == "__main__":
    main()
