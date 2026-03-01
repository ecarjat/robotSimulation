# LQR LUT Integration Plan

This document describes how to integrate the LQR gain look‑up table (LUT) into the firmware so gains are scheduled by hip angle, and includes a ready‑to‑use C header generated from `lqr_lut.csv`.

## Plan (firmware route)

1) **Create LUT module**  
   - New files: `lqr_lut.h` / `lqr_lut.c` (or `.cpp` if preferred).  
   - Store the LUT arrays (hip angles + K gains).  
   - Provide:
     ```c
     bool lqr_lut_eval(float hip_rad, float K_out[4]);
     ```
     which linearly interpolates between the two nearest hip entries and clamps to endpoints.

2) **Read hip angle**  
   - Use existing hip state in `motion_control.cpp` via `hip_control_get_state(...)`.  
   - Use average left/right when both valid.

3) **Update controller gains & equilibrium**  
   - At a low rate (e.g., 20–50 Hz), call `lqr_lut_eval_full` and update `lqr_params_t.K[]` through `MotionController::setLqrParams`.
   - Also pass `theta_eq` and `u_eq` to the controller (e.g., `setLqrEquilibrium(theta_eq, u_eq)`).
   - Keep `u_limit`, `du_limit`, etc. unchanged.

4) **Units**  
   - LUT gains are compatible with **torque** LQR (N·m) as used in `computeLqrUSumNm`.

---

## C header generated from `lqr_lut.csv`

```c
#ifndef LQR_LUT_DATA_H
#define LQR_LUT_DATA_H

#define LQR_LUT_SIZE 7

static const float LQR_HIP_LUT[LQR_LUT_SIZE] = {
    0.418006f,
    0.525897f,
    0.633787f,
    0.741678f,
    0.849568f,
    0.957459f,
    1.065349f
};

static const float LQR_K_LUT[LQR_LUT_SIZE][4] = {
    { -3799.913746f,  -367.283627f,  -91036.105980f, -1045.472248f },
    { -3463.901470f,  -497.241151f,  -73229.897567f,  -966.151232f },
    { -3275.975481f,  -502.668022f,  -46473.689164f,  -868.887156f },
    { -3483.147476f,  -762.951243f,  -23714.739052f,  -770.058716f },
    {  4103.788661f,   761.720182f,  -43785.254514f,  -828.648454f },
    {  5342.452053f,   969.744883f, -102179.898345f,  -767.094694f },
    {  7650.850615f,  1536.756489f, -226220.202908f,  -727.472800f }
};

static const float LQR_THETA_EQ_LUT[LQR_LUT_SIZE] = {
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f,
    0.000000f
};

static const float LQR_U_EQ_LUT[LQR_LUT_SIZE] = {
    -6.089650f,
    -5.683540f,
    -5.211330f,
    -4.678520f,
    -4.091310f,
    -3.456520f,
    -2.781530f
};

#endif /* LQR_LUT_DATA_H */
```
