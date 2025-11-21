# Mixed Platoon Simulation – Results Notes

Use this doc to capture runs, plots, and key takeaways for the TTC controller comparison.

## How to reproduce runs
1. Activate venv and install deps (once):  
   `source .venv/bin/activate && pip install numpy matplotlib`
2. Metrics-only sanity:  
   `python3 simulation.py`
3. Baseline vs TTC sweep with plots:  
   `python3 run_scenarios.py`  
   This sweeps `k_ttc = [0.0, 1.0, 2.0, 3.0]` and shows plots for baseline and highest gain.

## What to record for the report/slides
- Plots: speed, gap, TTC for baseline (k_ttc=0) and chosen TTC gain (e.g., 2 or 3). Save as images from the matplotlib window for slides.
- Metrics to note (printed in console):
  - `min_ttc` (higher is safer; watch for near-zero dips)
  - `min_gap` (no collision if >0)
  - `max_accel` / `max_decel` (comfort bounds)
  - `speed_std` (string stability proxy)
  - `jerk_mean_abs` (comfort proxy)

## Recommended comparison narrative
- Baseline (k_ttc=0): reacts later to leader braking, larger gap oscillations, TTC dips closer to 0–1 s.
- With TTC (k_ttc > 0): earlier braking keeps TTC higher, reduces gap overshoot and speed oscillations, decel is clipped by `a_min`.
- Tradeoff: higher k_ttc gives more safety margin but can add conservatism; tune between 1–2 for a balanced slide story.

## Tuning notes (if rerunning)
- TTC gain: edit `k_ttc_values` in `run_scenarios.py` or the default in `build_demo_platoon` in `simulation.py`.
- Accel limits: `a_min`/`a_max` in `Controller/ACC.py` and `Controller/CACC.py`.
- Disturbances: `accel_noise_std`, `delay_steps` in `build_demo_platoon`.

## Suggested artifacts to commit (after generating)
- Saved plots: e.g., `plots/baseline_speed_gap_ttc.png`, `plots/ttc_gain2_speed_gap_ttc.png`.
- A short summary snippet (bullets) with the metrics above for the chosen runs.
