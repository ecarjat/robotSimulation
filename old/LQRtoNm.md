# LQR torque path (Nm) spec

Goal: make the LQR path output torque (N·m) instead of motor current (A), while keeping PID path unchanged. The conversion from torque → iq should happen in the motor backend.

## Scope

- Only the LQR branch (`computeLqrUSum` and downstream LQR blending path) changes units.
- PID path remains in current units (A).
- Yaw differential term (`uTurn`) must match the units of `uSum` when blended.

## Current behavior (baseline)

- `computeLqrUSum` returns **iq**-space (`u_sum` in A).
- `uSum` is blended: `uSum = (1-alpha)*uSumPid + alpha*uSumLqr`
- `uTurn` is added/subtracted in the same units as `uSum`.
- Output `iqLeft/iqRight` is sent downstream.

## Desired behavior

### LQR path

- `computeLqrUSum` returns **torque** in N·m.
- LQR blending should be performed in **torque** (for the LQR term only).
- When alpha=1 (pure LQR), `uSum` is in N·m.

### PID path

- PID computations remain in **iq** (A), as they are today.
- When alpha=0 (pure PID), `uSum` is in A.

### Mixing requirement

The blend must not mix A and N·m directly. Options:

1) Convert LQR torque → iq before blending (minimal code change).  
2) Convert PID iq → torque before blending (more intrusive).

Spec choice: **Option 1** (convert LQR torque to iq for blending), but **keep** LQR’s internal output in torque for clarity and logging.
The torque→iq conversion should reuse the existing logic in `motor_link_set_wheel_torques`
by factoring it into a shared helper (see Backend section).

That means:
- `computeLqrUSumNm` returns torque (N·m).
- Immediately before blending, convert torque → iq:
  ```
  uSumLqrIq = uSumLqrNm / motorKt
  ```
- Blend still happens in iq:
  ```
  uSumIq = (1-alpha)*uSumPidIq + alpha*uSumLqrIq
  ```
- After blending, do NOT convert again; output stays iq (current), because PID path expects iq.

## Required code changes

1) `MotionController::computeLqrUSum`  
   - Rename to `computeLqrUSumNm` (or keep name but change unit; prefer rename).
   - Return torque N·m.

2) LQR blend block in `computeControl`  
   - Convert LQR torque → iq before blend:
     ```
     float uSumLqrNm = computeLqrUSumNm(...);
     float uSumLqrIq = (_motorKt > eps) ? (uSumLqrNm / _motorKt) : 0.0f;
     ```
   - Keep PID terms unchanged.
   - Blend in iq as today.

3) Diagnostics  
   - Update `InnerCtrlDiag` fields to clarify units:
     - `u_sum_lqr` should be logged in **Nm** (or add `u_sum_lqr_nm`).
     - `u_sum_cmd` should remain **iq** (A), since that’s what outputs send.
   - If keeping existing field names, add inline comments or adjust documentation.

4) Saturation limits  
   - Current `u_limit` is treated as iq limit in LQR mode.
   - If LQR is now in torque, decide whether `lqr.u_limit` is in N·m or A.
     - Spec: **keep `lqr.u_limit` in iq** to avoid changing config.
     - So apply limit after converting LQR torque → iq.

5) Backend
   - Factor the torque→iq conversion currently inside `motor_link_set_wheel_torques`
     into a reusable helper (e.g., `motor_link_torque_to_iq`).
   - Use the same helper in the LQR path when converting `uSumLqrNm` to iq.
   - This keeps a single source of truth for `motorKt`/sign conventions/limits.

## Config impact

- LQR gains `K` remain **torque-domain** if derived from MuJoCo.
  - If you keep blending in iq, the conversion above handles units.
- No changes needed to PID or velocity loop gains.

## Test plan

1) Unit test or log check:
   - With `alpha=1`, confirm `uSumLqrNm` equals expected torque (from LQR gains).
   - Confirm `uSumLqrIq = uSumLqrNm / motorKt`.
2) Runtime:
   - Ensure output `iq` remains stable when LQR enabled.
   - Compare against previous behavior by applying a known torque in sim and matching iq.
