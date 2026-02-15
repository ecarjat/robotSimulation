# `tools/run_lqr_pipeline.py`

End-to-end orchestration for LUT generation and per-hip `K0` tuning.

## What it does

`run_lqr_pipeline.py` executes:

1. `linearize_hip` with:
   - `--equilibrium-wheels`
   - `--write-keyframes`
   - `--reduced-no-x`
2. `tools/lqr_sweep.py` with:
   - `--state-dim 3`
   - `--include-eq`
   - `--no-sign-flip` (default in orchestrator)
3. Per-hip `tools/k0_sweep.py` runs
4. LUT update:
   - rewrites `K0` column in `lqr_lut.csv` using selected per-hip values
5. Header generation:
   - `tools/lqr_lut_to_header.py --out ... lqr_lut.csv`

## Adaptive K0 search

Per hip angle:

1. Start with seed K0 set (`--k0-values`, default `-100,-50,-1`)
2. Evaluate all seeds with `k0_sweep.py`
3. Select best candidate by score:
   - fail flag
   - `|final_x|`
   - `max_x`
   - `max_theta_deg`
   - `|final_y|`
   - `|final_vx|`
4. Propose neighborhood candidates around the current best
5. Repeat for `--k0-refine-rounds` (default `2`)

Outputs for K0 search are written under `--k0-runs-dir`, including:

- round-by-round run CSVs
- round-by-round aggregate CSVs
- `k0_selected.csv` summary

## Key options

- `--linearize-bin`: path to `linearize_hip` binary
- `--model`: model path
- `--write-keyframes`: equilibrium keyframes output
- `--out-dir`: linearization output directory
- `--lut`: LUT CSV path
- `--header-out`: generated header path
- `--k0-values`: seed K0 list (comma-separated)
- `--k0-refine-rounds`: refinement rounds
- `--k0-max-new-per-round`: new candidates per round
- `--k0-sim-use-ekf`: `0`/`1` (`0` recommended for K0 identification)

## Example

```bash
python3 tools/run_lqr_pipeline.py \
  --model myRobot/scene.xml \
  --out-dir linearize_out \
  --write-keyframes myRobot/keyframes_eq.xml \
  --lut lqr_lut.csv \
  --k0-values=-100,-50,-1 \
  --k0-refine-rounds 3 \
  --k0-max-new-per-round 4 \
  --header-out ../stm32Controller/firmware/app/control/lqr_lut_data.h
```

