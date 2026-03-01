# Tools Guide

This README documents the core tuning tools under `tools/` used by the LQR pipeline.

## Current Toolset

Supported pipeline tools:

- `run_lqr_pipeline.py`: end-to-end orchestrator (`linearize_hip` -> `lqr_sweep` -> per-hip `k0_sweep` -> LUT header generation)
- `lqr_sweep.py`: selects `K1/K2/K3` (and `K0` mapping mode) from reduced models
- `k0_sweep.py`: per-hip `K0` sweep against `test_balance`
- `torque_sweep.py`: sweeps disturbance-recovery torque across hip keyframes (`eq_hip_*`)
- `speed_pose_sweep.py`: sweeps hip keyframes and speed targets to find the pose with highest stabilized speed
- `lqr_lut_to_header.py`: converts `lqr_lut.csv` to firmware header

Additional analysis tool:

- `continuous_lqr.py`: continuous-time LQR analysis helper (not used by the default orchestrated pipeline)

## lqr_sweep.py

This section explains what `lqr_sweep.py` does, how it selects gains, and how to use its options.

### Purpose

`lqr_sweep.py` searches for one LQR weighting set that works across all sampled hip operating points from `linearize_out`.

It:

1. Loads reduced models per hip (`Ared_hip_*.csv`, `Bred_hip_*.csv` or 3-state variants).
2. Sweeps Q/R/input scaling candidates.
3. Solves discrete LQR at each hip.
4. Rejects candidates that fail stability constraints at any hip.
5. Picks the best surviving candidate (lowest worst-case closed-loop radius).
6. Writes:
   - sweep summary CSV (`lqr_sweep_results.csv` by default)
   - LUT CSV (`lqr_lut.csv` by default)

### Input Files

Expected in `--dir` (default `linearize_out`):

- 4-state mode (`--state-dim 4`): `Ared_hip_*.csv`, `Bred_hip_*.csv`
- 3-state mode (`--state-dim 3`): `Ared3_hip_*.csv`, `Bred3_hip_*.csv`

Optional equilibrium files (`eq_hip_*.csv`) are read from `--eq-dir` (or `--dir` if omitted) and added to the LUT when available.

### Selection Logic

For each candidate tuple `(q_scale, r_scale, u_scale)`:

1. Build Q:
   - 4-state: `diag([qx, qv, qt, qtd]) * q_scale`
   - 3-state: `diag([qv, qt, qtd]) * q_scale`
2. Build R: `[[r_scale]]`.
3. For each hip:
   - Apply input scaling to B: `B_eff = B * u_scale`
   - Solve DARE iteratively and compute LQR gain on `B_eff`
   - Map back to real input gain: `K = Kp * u_scale`
   - Compute closed-loop matrix `Acl = A - B*K`
   - Compute eigenvalue magnitudes
4. Candidate is valid only if all hips pass stability checks.

#### Stability Checks

- Default strict rule: reject if any hip has `rho(Acl) >= 1.0`.
- With `--allow-marginal`: allow one near-1 eigenvalue (position integrator), reject if:
  - any `|lambda| > 1 + rho_tol`
  - more than one eigenvalue in `[1-rho_tol, 1+rho_tol]`
- Optional global cap: `--rho-target` rejects if `rho(Acl)` exceeds target.

#### Automatic Marginal Handling (Cascaded + `qx=0`)

When all are true:

- `--cascaded-k0`
- `--state-dim 4`
- `--qx 0`
- `--strict-stability` not set
- `--allow-marginal` not already set

the tool auto-enables marginal allowance. This matches the common cascaded case where x-position is not penalized and one marginal mode is expected.

#### Candidate Ranking

Two radii are tracked:

- `max_spectral_radius`: worst raw `rho(Acl)` across hips
- `max_selection_radius`: ranking radius used for best-candidate selection

With marginal allowance enabled, `max_selection_radius` ignores one marginal near-1 mode per hip and ranks by the remaining modes.

#### Optional Sign-Consistency Filter

With `--no-sign-flip`, candidates are additionally rejected if selected gains change sign across hip samples.

- Gains checked are configured via `--sign-gains` (default: `K1,K2,K3`).
- Values with `|K| <= sign_eps` are treated as zero (ignored for sign detection).
- If this filter is enabled, `--best-effort` fallback also stays within sign-consistent candidates.

### LUT Generation

After selecting the best sweep candidate, the tool recomputes K for each hip and writes LUT rows in hip-sorted order.

If `--cascaded-k0` and `--state-dim 4` are used, K0 is remapped for the cascaded controller:

`K0_cascaded = -K0_direct / K1_direct`

If `|K1_direct|` is very small, K0 is forced to `0.0` to avoid numeric blow-up.

Equilibrium columns are included when `--include-eq` is set or any `eq_hip` files are found.

### CLI Options

| Option | Default | Meaning |
|---|---:|---|
| `--dir` | `linearize_out` | Directory for A/B reduced matrices |
| `--state-dim` | `4` | `4` for `Ared/Bred`, `3` for `Ared3/Bred3` |
| `--include-eq` | off | Include `theta_eq,u_eq` columns in LUT |
| `--eq-dir` | `--dir` | Directory for equilibrium files |
| `--eq-glob` | `eq_hip_*.csv` | Pattern for equilibrium files |
| `--eq-tol` | `1e-6` | Hip-value matching tolerance for eq files |
| `--require-eq` | off | Fail if any hip lacks matched eq data |
| `--qx` | `0.0` | Base Q weight for x |
| `--qv` | `1.0` | Base Q weight for xdot |
| `--qt` | `500.0` | Base Q weight for theta |
| `--qtd` | `50.0` | Base Q weight for thetadot |
| `--q-scale` | `0.1,0.3,1,3,10` | Sweep multipliers for Q |
| `--r-scale` | `0.1,0.3,1,3,10` | Sweep multipliers for R |
| `--u-scale` | `1` | Sweep multipliers applied to B |
| `--rho-tol` | `1e-6` | Tolerance for marginal-mode checks |
| `--rho-target` | unset | Optional upper bound on rho |
| `--allow-marginal` | off | Allow one near-1 mode |
| `--strict-stability` | off | Disable automatic marginal-mode allowance |
| `--best-effort` | off | Emit best candidate even if unstable |
| `--no-sign-flip` | off | Reject candidates with sign changes in selected gains across hips |
| `--sign-gains` | `K1,K2,K3` | Gains checked by `--no-sign-flip` |
| `--sign-eps` | `1e-9` | Near-zero threshold for sign checks |
| `--cascaded-k0` | off | Remap K0 for cascaded architecture |
| `--low-hip-min` | `-0.270` | Hip angle where low-hip gain shaping is full-strength |
| `--low-hip-max` | `-0.055` | Hip angle where low-hip gain shaping tapers to 1.0 |
| `--low-hip-k2-boost` | `1.0` | K2 multiplier at full low-hip shaping weight |
| `--low-hip-k3-boost` | `1.0` | K3 multiplier at full low-hip shaping weight |
| `--diag` | off | Print per-hip `rho(A)`, `||B||`, controllability rank |
| `--out` | `lqr_sweep_results.csv` | Sweep summary CSV path |
| `--lut-out` | `lqr_lut.csv` | Output LUT CSV path |

### Output Files

`--out` CSV columns:

- `q_scale`
- `r_scale`
- `u_scale`
- `stable_all`
- `max_spectral_radius`
- `max_selection_radius`
- `sign_consistent` (1/0)

`--lut-out` CSV columns:

- without eq: `hip,K0,K1,K2,K3`
- with eq: `hip,K0,K1,K2,K3,theta_eq,u_eq`

### Typical Commands

Cascaded workflow (recommended for your current controller):

```bash
python3 tools/lqr_sweep.py \
  --dir linearize_out \
  --state-dim 4 \
  --include-eq \
  --cascaded-k0 \
  --lut-out lqr_lut.csv
```

3-state workflow with sign-flip rejection (useful when K3 flips at one hip):

```bash
python3 tools/lqr_sweep.py \
  --dir linearize_out \
  --state-dim 3 \
  --include-eq \
  --no-sign-flip \
  --low-hip-min -0.270 \
  --low-hip-max -0.055 \
  --low-hip-k2-boost 2.0 \
  --low-hip-k3-boost 1.5 \
  --lut-out lqr_lut.csv
```

Strict mode (no marginal allowance):

```bash
python3 tools/lqr_sweep.py \
  --dir linearize_out \
  --state-dim 4 \
  --include-eq \
  --cascaded-k0 \
  --strict-stability
```

Inspect model quality before trusting sweep output:

```bash
python3 tools/lqr_sweep.py --dir linearize_out --state-dim 4 --diag
```

### Related Tools

- `build/lqr_harness/linearize_hip`: generates `Ared/Bred` and `eq_hip` files used by this sweep.
- `tools/lqr_lut_to_header.py`: converts `lqr_lut.csv` into `lqr_lut_data.h` for simulation/firmware.

## k0_sweep.py

This tool sweeps `SIM_K0_OVERRIDE` values and runs `test_balance` to score candidate `K0` values.

### Purpose

`k0_sweep.py` is used to tune the outer position gain (`K0`) against simulation behavior (drift/stability), typically after `lqr_sweep.py` has produced `K1/K2/K3`.

Default behavior is ground-truth state (`SIM_USE_EKF=0`) so K0 tuning is not biased by estimator issues.

### Inputs

- `--test-balance`: path to `test_balance` executable
- `--model`: model path (usually `myRobot/scene.xml`)
- `--duration`: run length per candidate
- `--k0-values`: comma-separated K0 values to test
- `--keys`: keyframe(s) to test (`eq_hip_*`)
  If omitted, keys are derived from the `hip` column in `--lut`
- `--sim-use-ekf`: `0` or `1` (default `0`)

### Outputs

- Per-run CSV (`--run-csv`): one row per `(K0, keyframe)`
- Aggregate CSV (`--agg-csv`): one row per `K0`
- Console line:
  - `BEST_K0 ...`

### Selection logic

Each K0 is ranked by:

1. fail count (`rc != 0` or `diverged != 0` or `passed != 1`)
2. worst `|final_x|`
3. worst `max_x`
4. worst `max_theta_deg`

The best-ranked candidate is printed as `BEST_K0`.

### Example

```bash
python3 tools/k0_sweep.py \
  --duration 20 \
  --k0-values=-2,-1,-0.5,-0.25 \
  --keys eq_hip_p0525 \
  --run-csv linearize_out/k0_sweeps/k0_sweep_runs_eq_hip_p0525.csv \
  --agg-csv linearize_out/k0_sweeps/k0_sweep_agg_eq_hip_p0525.csv
```

## torque_sweep.py

This tool runs disturbance recovery tests across hip keyframes and reports max wheel torque.

### Purpose

`torque_sweep.py` automates:

1. selecting keyframes (`--keys` or derive from `lqr_lut.csv`)
2. running `test_balance` with disturbance options
3. parsing `RECOVERY_SUMMARY`
4. reporting global maximum torque across swept hip poses

### Inputs

- `--test-balance`: path to `test_balance`
- `--model`: model path (typically `myRobot/scene.xml`)
- `--keys`: optional comma-separated keyframes
- `--lut`: derive keyframes from LUT `hip` column when `--keys` is omitted
- disturbance/recovery settings:
  - `--theta-disturb-deg`
  - `--disturb-time`
  - `--wheel-limit-nm`
  - `--settle-theta-err-deg`
  - `--settle-theta-dot`
  - `--settle-hold`

### Outputs

- Per-key CSV (`--csv-out`, default `torque_sweep_results.csv`)
- Console lines:
  - `MAX_TAU key=... hip=... max_tau=...`
  - `MAX_TAU_TO_SETTLE key=... hip=... max_tau_to_settle=...`
  - `SWEEP_SUMMARY ...`

### Example

Sweep all hip keyframes from LUT with a 5° disturbance and high wheel limit:

```bash
python3 tools/torque_sweep.py \
  --duration 8 \
  --theta-disturb-deg 5 \
  --disturb-time 0.5 \
  --wheel-limit-nm 35 \
  --csv-out linearize_out/torque_sweep_results.csv
```

## speed_pose_sweep.py

This tool sweeps speed targets across hip keyframes and reports which pose stabilizes at the highest speed.

### Purpose

`speed_pose_sweep.py` automates:

1. selecting keyframes (`--keys` or derive from `lqr_lut.csv`)
2. running `test_balance` in speed mode for each `(keyframe, speed target)`
3. parsing `SPEED_SUMMARY` and `SUMMARY`
4. reporting:
   - best sustained stable speed per keyframe
   - global best pose (`BEST_POSE`)

### Inputs

- `--keys`: optional comma-separated keyframes
- `--lut`: derive keyframes from LUT `hip` column when `--keys` is omitted
- `--speed-values`: explicit speed targets list (comma-separated), or:
  - `--speed-min`, `--speed-max`, `--speed-step`
- speed-mode flags passed through to `test_balance`:
  - `--speed-start-time`
  - `--speed-ramp-mps2`
  - `--speed-settle-err-mps`
  - `--speed-settle-hold`
  - `--post-settle-hold` (additional no-divergence hold after settle; default `2.0 s`)
  - `--wheel-limit-nm`
  - `--lqr-v-ref-limit` (if omitted, auto-set to `max(|speed|)+0.25`)

### Outputs

- Per-run CSV (`--run-csv`, default `speed_pose_sweep_runs.csv`)
- Per-key summary CSV (`--key-csv`, default `speed_pose_sweep_key_summary.csv`)
- Console line:
  - `BEST_POSE key=... max_stable_target_speed_mps=...`

### Example

Sweep one pose from `0.5` to `3.0 m/s`:

```bash
python3 tools/speed_pose_sweep.py \
  --keys eq_hip_p0525 \
  --speed-min 0.5 \
  --speed-max 3.0 \
  --speed-step 0.25 \
  --duration 10 \
  --speed-ramp-mps2 0.8 \
  --post-settle-hold 2.0 \
  --wheel-limit-nm 35 \
  --lqr-v-ref-limit 3.2 \
  --break-on-first-unstable \
  --run-csv linearize_out/speed_pose_sweep_runs.csv \
  --key-csv linearize_out/speed_pose_sweep_key_summary.csv
```
