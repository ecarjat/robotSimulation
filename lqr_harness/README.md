# MuJoCo Linearization Harness

This directory now provides the `linearize_hip` tool used by the simulation LUT pipeline.

## Build

Set `MUJOCO_DIR` (or `MUJOCO_ROOT`) to your MuJoCo install path, then:

```bash
cd lqr_harness
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
cd lqr_harness/build
./linearize_hip --model ../../myRobot/scene.xml --equilibrium-wheels --out ../../linearize_out
```

For full options:

```bash
./linearize_hip --help
```

Detailed behavior and workflow references:

- `../README.md#linearize-hip-guide`
- `../tools/README.md#lqr_sweeppy`
- `../docs/run_lqr_pipeline.md`
