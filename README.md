# VIPSubTeam8
Mixed-traffic platoon simulation to evaluate ACC/CACC/IDM controllers (including TTC-based safety) with CAVs and HDVs under disturbances.

## What’s here
- Controllers: `Controller/ACC.py`, `Controller/CACC.py`, `Controller/IDM.py` (now with optional TTC term and accel limits for ACC/CACC).
- Simulation harness: `simulation.py` (vehicle state, physics update, noise/delay hooks, TTC calc, demo platoon, basic metrics).
- Scenario runner: `run_scenarios.py` (runs multiple TTC gains, prints metrics, plots speed/gap/TTC).
- Legacy/env stubs: `Env/CAV.py`, `Env/HDV.py` (placeholders), `Env/OVM.py` (standalone OVM demo).

## Quick setup
From the project root:
```
python3 -m venv .venv
source .venv/bin/activate          # macOS/Linux
pip install numpy matplotlib
```
On Windows PowerShell, activate with: `.venv\Scripts\Activate.ps1`

## How to run
- Smoke test / metrics only:
  ```
  python3 simulation.py
  ```
  Prints a metrics dict (min TTC/gap, accel bounds, speed spread, jerk proxy).
- Compare baseline vs TTC gains with plots:
  ```
  python3 run_scenarios.py
  ```
  Sweeps `k_ttc` over [0.0, 1.0, 2.0, 3.0], prints metrics, and pops speed/gap/TTC plots for baseline and highest gain.

## Tuning knobs
- Safety: `k_ttc`, `min_ttc` in `Controller/ACC.py` / `Controller/CACC.py` and via `build_demo_platoon`/`run_scenarios.py`.
- Comfort: accel limits `a_min`, `a_max` in the controllers.
- Following policy: `k_p`, `k_v`, `k_a`, `s0`, `T`.
- Disturbances: `accel_noise_std`, `delay_steps` per vehicle in `build_demo_platoon`.

## Notes
- TTC braking is optional; set `k_ttc=0` for baseline behavior.
- The current harness uses a scripted leader brake event and mixed CAV/HDV platoon (CAVs with CACC, HDVs with IDM + noise/delay).
