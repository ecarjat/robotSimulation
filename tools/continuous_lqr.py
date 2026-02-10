#!/usr/bin/env python3
"""
Design LQR in continuous time from discrete A,B matrices.

Discrete: x[k+1] = Ad*x[k] + Bd*u[k]  with timestep dt
Continuous approx: Ac = (Ad - I)/dt,  Bc = Bd/dt
Then solve continuous ARE for Ac, Bc, Q, R.
"""
import argparse
import glob
import os
from pathlib import Path
import numpy as np
from scipy import linalg

def load_csv(path):
    return np.loadtxt(path, delimiter=",")

def parse_hip(path):
    name = Path(path).stem
    parts = name.split("_")
    try:
        idx = parts.index("hip")
        return float(parts[idx + 1])
    except:
        return None

def continuous_lqr(Ac, Bc, Q, R):
    """Solve continuous-time LQR: min integral(x'Qx + u'Ru)dt"""
    P = linalg.solve_continuous_are(Ac, Bc, Q, R)
    K = np.linalg.inv(R) @ Bc.T @ P
    return K, P

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="linearize_out")
    ap.add_argument("--dt", type=float, default=0.002)
    ap.add_argument("--state-dim", type=int, default=3, choices=[3, 4])
    ap.add_argument("--qv", type=float, default=1.0)
    ap.add_argument("--qt", type=float, default=100.0)
    ap.add_argument("--qtd", type=float, default=10.0)
    ap.add_argument("--qx", type=float, default=0.0)
    ap.add_argument("--r", type=float, default=1.0)
    ap.add_argument("--include-eq", action="store_true")
    ap.add_argument("--eq-dir", default=None)
    ap.add_argument("--eq-tol", type=float, default=1e-5)
    ap.add_argument("--out", default="lqr_lut.csv")
    args = ap.parse_args()

    if args.eq_dir is None:
        args.eq_dir = args.dir

    # Load discrete A,B matrices
    if args.state_dim == 3:
        a_glob = "Ared3_hip_*.csv"
        b_glob = "Bred3_hip_*.csv"
    else:
        a_glob = "Ared_hip_*.csv"
        b_glob = "Bred_hip_*.csv"

    a_files = sorted(glob.glob(os.path.join(args.dir, a_glob)))
    b_files = sorted(glob.glob(os.path.join(args.dir, b_glob)))

    if not a_files:
        print(f"No {a_glob} found in {args.dir}")
        return

    # Load eq data
    eq_data = {}
    for f in glob.glob(os.path.join(args.eq_dir, "eq_hip_*.csv")):
        hip = parse_hip(f)
        if hip is None:
            continue
        try:
            lines = open(f).readlines()
            parts = lines[1].split(",")
            eq_data[hip] = (float(parts[1]), float(parts[2]))
        except:
            pass

    mats = {}
    for af, bf in zip(a_files, b_files):
        hip = parse_hip(af)
        if hip is None:
            continue
        Ad = load_csv(af)
        Bd = load_csv(bf).reshape(-1, 1)
        mats[hip] = (Ad, Bd)

    dt = args.dt
    n = args.state_dim

    if n == 3:
        Q = np.diag([args.qv, args.qt, args.qtd])
    else:
        Q = np.diag([args.qx, args.qv, args.qt, args.qtd])
    R = np.array([[args.r]])

    print(f"Q = diag({np.diag(Q)})")
    print(f"R = [[{args.r}]]")
    print(f"dt = {dt}")
    print()

    results = []
    for hip in sorted(mats.keys()):
        Ad, Bd = mats[hip]

        # Convert discrete to continuous
        I = np.eye(n)
        Ac = (Ad - I) / dt
        Bc = Bd / dt

        # Check eigenvalues of Ac
        eig_c = np.linalg.eigvals(Ac)
        
        try:
            K, P = continuous_lqr(Ac, Bc, Q, R)
        except Exception as e:
            print(f"hip={hip:.4f}: LQR failed: {e}")
            continue

        # Check closed-loop stability
        Acl = Ac - Bc @ K
        eig_cl = np.linalg.eigvals(Acl)
        max_real = max(e.real for e in eig_cl)

        K_flat = K.flatten()
        
        # Get equilibrium
        theta_eq, u_eq = eq_data.get(hip, (0.0, 0.0))
        # Find closest match
        if hip not in eq_data:
            best_err = 1e10
            for eh, ev in eq_data.items():
                err = abs(eh - hip)
                if err < best_err and err < args.eq_tol:
                    best_err = err
                    theta_eq, u_eq = ev

        print(f"hip={hip:+.4f}  K=[{K_flat[0]:+10.4f}, {K_flat[1]:+10.4f}, {K_flat[2]:+10.4f}]  "
              f"max_real(Acl)={max_real:.4f}  "
              f"eig_open=[{', '.join(f'{e.real:+.2f}' for e in sorted(eig_c, key=lambda x: -x.real))}]")

        results.append((hip, K_flat, theta_eq, u_eq, max_real))

    if not results:
        print("No results!")
        return

    # Write LUT
    with open(args.out, "w") as f:
        f.write("hip,K0,K1,K2,K3,theta_eq,u_eq\n")
        for hip, K, theta_eq, u_eq, _ in results:
            if n == 3:
                f.write(f"{hip:.6f},0.000000,{K[0]:.6f},{K[1]:.6f},{K[2]:.6f},"
                        f"{theta_eq:.9e},{u_eq:.9e}\n")
            else:
                f.write(f"{hip:.6f},{K[0]:.6f},{K[1]:.6f},{K[2]:.6f},{K[3]:.6f},"
                        f"{theta_eq:.9e},{u_eq:.9e}\n")

    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
