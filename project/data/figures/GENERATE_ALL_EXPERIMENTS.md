# Reproduce the report figures

Run these commands from the `project` directory. The interpreter is the
repository virtual environment. The controller arguments below are the ones
`main_demo.py` already accepts. Logging does not change the gains, the MPC,
the CBF, the gait or the residual.

## 1. Standing push, 80 N

```text
..\.venv\Scripts\python.exe main_demo.py --no-walk --headless --duration 6.5 --seed 0 --log-path data/logs/standing_push_latest.npz
```

The push is the built-in default: 80 N along +x at t = 2.0 s, held for 0.2 s.
`--duration` applies only with `--headless`.

## 2. Deterministic one-step walk

```text
..\.venv\Scripts\python.exe main_demo.py --headless --steps 1 --duration 24 --seed 0 --log-path data/logs/walking_single_step_latest.npz
```

`--steps` is the number of swings. The run stops early if the pelvis drops
below 0.72 m.

## 3. Multi-step schedule

```text
..\.venv\Scripts\python.exe main_demo.py --headless --steps 4 --duration 24 --seed 0 --log-path data/logs/walking_multi_step_latest.npz
```

`--steps 4` is the demo default. The footstep figure is the planner's landing
sequence for that step length and step count, using the ankles measured at
the start of this log. It is not a claim that four steps were completed.

## 4. Touchdown residual on the same one-step task

This uses the existing 20-episode checkpoint. It does not train.

```text
..\.venv\Scripts\python.exe main_demo.py --headless --steps 1 --duration 24 --seed 0 --rl-touchdown --rl-model data/runs/touchdown_costfix_20ep.pt --log-path data/logs/touchdown_rl_latest.npz
```

`--rl-touchdown` requires `--rl-model`. The residual is applied only in the
late-swing window already implemented in `rl/touchdown.py`.

## 5. Training diagnostics

Do not start another 500-episode run for these figures. The plotted training
series is the completed 20-episode cost-fix run:

```text
data/logs/touchdown_training_20ep.csv
data/runs/touchdown_costfix_20ep.pt
```

A completed 500-episode log was not found. Future training runs append their
own update rows, without changing the update itself:

```text
..\.venv\Scripts\python.exe -m rl.touchdown_train --episodes 20 --seed 0 --out data/runs/touchdown_costfix_20ep.pt
```

That command overwrites the checkpoint if you run it again. The per-update
CSV for a new run is `data/logs/touchdown_training_latest.csv`.

## 6. Unit tests

```text
..\.venv\Scripts\python.exe -m tests.test_lateral_shift
..\.venv\Scripts\python.exe -m tests.test_touchdown
```

## 7. Figures

```text
..\.venv\Scripts\python.exe utils/generate_project_figures.py --experiment all
```

One log at a time:

```text
..\.venv\Scripts\python.exe utils/generate_project_figures.py --log data/logs/walking_single_step_latest.npz
..\.venv\Scripts\python.exe utils/generate_project_figures.py --log data/logs/standing_push_latest.npz
..\.venv\Scripts\python.exe utils/generate_project_figures.py --log data/logs/touchdown_rl_latest.npz
```

Figures are written to `data/figures/` at 300 dpi. `data/figures/FIGURE_INDEX.txt`
records the log, variables, units and report section for each file.
