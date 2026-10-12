# Project status

Whole-body MPC, PD with gravity compensation, and a CBF torque filter on the Unitree H1 in MuJoCo. Standing and the 80 N torso push work. The shift onto the left sole works, and the right swing reaches touchdown. One step still does not finish in a stable double support. Continuous walking is not achieved.

The sections through [Checks from the swing study](#checks-from-the-swing-study) are the landing investigation already recorded in `ProjectStatusUpdate.txt`. [Measured on 12 October 2026](#measured-on-12-october-2026) adds the later seed-0 runs, the touchdown residual, and the report figures. Those later runs use the landing hold. `--late-swing-capture` and `--landing-handoff` stay off.

## Current failure

The backward fall was the landing hand-off. It was not the weight shift and it was not the swing itself. After the landing reference was held, the body stopped drifting backward and then rolled off the outside of the stance foot.

`first_swing="right"` shifts onto the **left** foot, then swings the **right** foot. In the baseline log that transition is at 17.32 s.

## What the swing logs showed

During the right swing the rear support edge barely moves and the centre of mass stays inside it.

| Quantity | Baseline 0.8 s, 0.10 m | Slow swing 1.2 s | Short step 0.05 m | Swing frozen | Ramp 0.01 rad/tick |
| --- | ---: | ---: | ---: | ---: | ---: |
| Rear margin during swing | 0.058 m | 0.055 m | 0.058 m | 0.063 m | 0.062 m |
| CoM x change during swing | −7 mm | −11 mm | −7 mm | −2 mm | −3 mm |
| Rear edge movement | −0.9 mm | −1.8 mm | −0.9 mm | −1.2 mm | −0.7 mm |
| Peak swing torque | 29 N·m | 27 N·m | 31 N·m | 20 N·m | 19 N·m |
| First `support_x_lower` ≤ 0 | 17.72 s, double support | 18.16 s, stand | 17.78 s, stand | none | 17.88 s, stand |
| First non-optimal CBF | 17.64 s, double support | 18.08 s, double support | 17.70 s, double support | 17.34 s, double support | 17.66 s, double support |

On the baseline landing tick the reference jumped from (0.060, 0.232) m to (0.092, 0.203) m while the centre of mass was at (0.049, 0.241) m. That target is the state machine's foot landmark, and it replaced the 8 mm clamp. The same tick dropped `LateralWeightShift.apply()`, including the heel ankle term. Forward speed then went from −0.07 m/s to −0.24 m/s inside the 0.4 s double-support window, and the rear margin reached zero.

Slowing the swing, shortening the step, and slowing the joint ramp all still fell at that same hand-off. Freezing the swing avoided the rear-edge collapse and failed laterally on the same tick: the reference jumped to y = 0.203 m while the centre of mass was already at y = 0.296 m, the CBF went fail-safe, and the peak intervention was 544 N·m. Swing-leg momentum in these logs is a few millimetres of sagittal drift.

Holding the clamped reference and the stance posture through the landing removed the backward fall. Centre-of-mass x stayed near 0.04 m and the rear margin stayed positive. The body then rolled outward. Lateral speed was about +0.04 m/s at touchdown and kept growing while the CBF was still optimal. The measured sole rolls with the body, because this model has no ankle-roll actuator, so the outer margin stayed non-negative until the speed was already large.

Reducing the lean after landing was tried and removed. The blend went from 0.47 to 0.27 in 0.4 s, and the outward speed still reached 1.57 m/s. Hip roll is the lateral actuator. Unwinding it after touchdown did not pull the centre of mass back between the feet.

## Where the hand-off is in the code

The jump is in `walk_tick` in `main_demo.py`. While `loaded_foot` is set, the reference comes from `LateralWeightShift.advance()`. On `DOUBLE_SUPPORT` and `STAND`, `loaded_foot` is `None`, so the reference used to fall through to `command.com_target` plus the sole-centre offset, and `active_posture` became `None`. `WalkingStateMachine._arm_state()` then aims the stand target at the midpoint of the two feet. The heel term that had been holding the rear edge lives only in `LateralWeightShift.apply()`.

## What that investigation changed

`swing_diagnostic.py` prints the swing summary from the logged rows after a walking run and saves `*_support.png` with centre-of-mass x, capture x, and the rear support edge.

`main_demo.py` logs, on every walking tick, both legs' hip roll, hip pitch, knee and ankle (angle, desired angle, safe torque, nominal torque), knee velocity, capture point, capture error, capture margin on each face, support margins, stance ankle correction, shift target, and pelvis pitch. Flags added then, all defaulting to the previous behaviour: `--step-length 0.1`, `--step-duration 0.8`, `--swing-ramp 0.03`, `--freeze-swing`, `--headless`, `--duration`.

After the swing, the clamped shift reference and the stance posture stay active, so the heel term is not removed on the landing tick. The landed leg's pitch, knee and ankle are set to the swing command the ramp actually reached. Its hip roll stays on the lean blend. `apply()` takes a `skip_side` so the landed pitch, knee and ankle are not pulled back to the pre-swing pose.

`--no-walk` does not enter this path.

## Commands for the swing study

Run from `project`. The interpreter is the repository virtual environment. System Python does not have the project packages.

```text
..\.venv\Scripts\python.exe main_demo.py --headless --steps 1 --duration 24 --log-path data/swing_A_baseline.npz
..\.venv\Scripts\python.exe main_demo.py --headless --steps 1 --step-duration 1.2 --log-path data/swing_B_slow_swing.npz
..\.venv\Scripts\python.exe main_demo.py --headless --steps 1 --step-length 0.05 --log-path data/swing_C_short_step.npz
..\.venv\Scripts\python.exe main_demo.py --headless --steps 1 --freeze-swing --log-path data/swing_D_freeze_swing.npz
..\.venv\Scripts\python.exe main_demo.py --headless --steps 1 --swing-ramp 0.01 --log-path data/swing_E_slow_ramp.npz
..\.venv\Scripts\python.exe main_demo.py --no-walk --headless --duration 6.5
..\.venv\Scripts\python.exe -m tests.test_lateral_shift
```

Defaults are unchanged: 0.8 s, 0.10 m, 0.03 rad per tick.

## Checks from the swing study

`python -m tests.test_lateral_shift` reported 6 passed.

`--no-walk` for 6.5 s stayed in `STAND`. Minimum pelvis height was 0.927 m. The 80 N push is applied at t = 2.0 s for 0.2 s. Peak CBF intervention was 120.27 N·m, the same value as the existing push-recovery record.

The one-step run with the landing hold still fell, at about 18.6 s, outward rather than backward. `support_x_lower` did not reach zero during the swing or at the landing.

## Constraints that still hold

Do not retune the swing duration, step length, or ramp to hide the fall. Those three experiments fell at the same hand-off. Do not unwind the hip-roll blend after touchdown. That was measured, and it did not create an inward acceleration. Do not widen the support polygon or the CBF trust region. The filter was optimal while the sole was still reported under the centre of mass.

The CBF remains the last torque layer. Its status (`optimal`, `relaxed`, `fail_safe`) is the quadratic-program result. A positive barrier is a signed distance inside the measured support box. It is not a formal safety certificate, and a negative barrier is the centre of mass outside that face of the box.

## What was added after the swing study

`--late-swing-capture` and `--landing-handoff` exist and default **off**. With the capture flag, the lean blend may fall by at most 0.04, at 0.15 per second, only while the outer capture margin is under 5 cm and the body is moving outward. The handoff flag freezes the touchdown centre-of-mass reference and keeps the landed pitch, knee, and ankle on the command the swing reached. Neither flag is part of the default demo, and neither has produced a stable landing.

The touchdown residual is optional. It is a bounded 2-D centre-of-mass offset, about ±0.02 m, inserted after the deterministic reference and before `control_step`. It does not command joint torque. The default demo is deterministic. The residual runs only with `--rl-touchdown` and `--rl-model`.

The training constraint uses the **mean per-step** CBF cost against a budget of 0.5. An earlier formulation compared the episode sum with a budget of 5.0. The standing trainer's budget is still 5.0. A completed 500-episode touchdown run was not found. The recorded series is 20 episodes, checkpoint `project/data/runs/touchdown_costfix_20ep.pt`.

Logging for the report figures is in `project/utils/experiment_logger.py`. It records existing signals. It does not change the timestep, gains, MPC, CBF, gait, residual, or physics. `project/utils/generate_project_figures.py` reads those logs and writes the PNGs.

## Measured on 12 October 2026

All of these use seed 0 and the default landing hold. The optional capture, handoff, and residual flags are off except in the residual row.

| Run | Stop | Pelvis height | Control ticks | Peak CBF intervention |
| --- | --- | ---: | ---: | ---: |
| Standing push, 6.5 s | 6.52 s, still `STAND` | 0.928 m final | 326 | 120.27 N·m |
| One step, `--steps 1` | 18.68 s, fall | 0.704 m | 934 | 133.90 N·m |
| Four-step schedule, `--steps 4` | 18.68 s, fall | 0.715 m | 934 | 228.58 N·m |
| One step plus 20-episode residual | 18.70 s, fall | 0.701 m | 935 | 133.87 N·m |

### One step

Log: `project/data/logs/walking_single_step_latest.npz`.

Right swing, left stance, 0.78 s, from 16.54 s to 17.32 s. Touchdown, the first `DOUBLE_SUPPORT` tick, is 17.34 s. The first non-optimal CBF status is 17.96 s, in `STAND`. The first backward centre-of-mass velocity below −0.05 m/s is 17.28 s, still in `RIGHT_SWING`.

During the swing, the minimum rear margin `support_x_lower` is 0.0575 m. CoM x changes by −7.3 mm. The rear edge moves −0.9 mm. Peak swing-leg torque is 29.5 N·m.

At 17.34 s the centre of mass is at (0.049, 0.241) m with velocity (−0.067, +0.044) m/s. The reference is at (0.058, 0.232) m, 5 mm away. The shift fraction is 0.479 and is still 0.479 after 0.4 s. The CBF is optimal and the intervention is 5.4 N·m. The outer support margin is +0.054 m. After the swing, outward speed reaches 1.551 m/s and the outer support margin reaches −0.551 m. The fall is that outward roll.

MPC solve time on this log: mean 0.95 ms, median 0.73 ms, 95th percentile 1.71 ms, maximum 12.81 ms. The control period is 20 ms.

### Four-step schedule

Log: `project/data/logs/walking_multi_step_latest.npz`.

The first swing matches the one-step run. After landing the gait enters `SHIFT_RIGHT`. The first non-optimal CBF status is 17.74 s, in that shift. Four steps are not completed. `fig_3_4_footstep_plan.png` is the planner's landing sequence from the measured initial ankles and a 0.10 m step length: about 0.14, 0.24, 0.34, and 0.44 m forward. Those are planned targets, not measured footfalls.

### Standing push

Log: `project/data/logs/standing_push_latest.npz`. Summary: `project/data/figures/push_recovery_summary.json`.

The push is 80 N in +x from 2.0 s to 2.2 s. Peak torso pitch is 0.218 rad (12.51°). Peak CoM x velocity is 0.136 m/s. Peak CoM speed is 0.177 m/s. Maximum CoM x displacement is 0.098 m. Final centre of mass is (0.070, 0.0004, 0.928) m. Final torso pitch is about 0.

### Touchdown residual

Evaluation log: `project/data/logs/touchdown_rl_latest.npz`. Comparison: `project/data/figures/rl_evaluation/rl_vs_baseline_metrics.json`. Training table: `project/data/logs/touchdown_training_20ep.csv`.

One shared seed. Both the deterministic step and the residual step touch down, and both fall. Stable landing rate is 0. Time to a stable landing is not defined for either trial. Lateral speed at double support is 0.044 m/s on the baseline and 0.042 m/s with the residual. Peak absolute lateral speed is 1.551 m/s and 1.556 m/s. Mean absolute residual is 0.23 mm. Maximum absolute residual is 7.0 mm, inside the ±0.02 m bound. This comparison does not show an improvement.

The 20-episode log records updates on episodes 1, 3, 5, …, 19. Each update uses 66 steps. The stable flag is 0 on every row. Batch-mean reward stays between −0.052 and 0. Mean per-step CBF cost runs from 0.450 to 1.309 against the budget of 0.5. The Lagrange multiplier moves from 1.007 to 1.082. Stop reasons in the table are `tipped` or `cbf_infeasible`.

### Tests

| Test | Result | What it checks |
| --- | --- | --- |
| `python -m tests.test_lateral_shift` | 6 passed | Shift reference and related planner behaviour |
| `python -m tests.test_touchdown` | 6 passed | Touchdown observation, residual clip, and the per-step cost used by the multiplier |

These are component tests. A passing test does not mean the robot completes a stable step. The same distinction is in `project/data/figures/test_results_summary.txt`.

## Report figures

Generated from the logs above by:

```text
..\.venv\Scripts\python.exe utils/generate_project_figures.py --experiment all
```

Written at 300 dpi under `project/data/figures/`. The index, with the log, variables, units, and report section for each file, is `FIGURE_INDEX.txt`. The reproduce commands are in `GENERATE_ALL_EXPERIMENTS.md`.

| Figure | Log | What it shows |
| --- | --- | --- |
| `fig_3_3_lateral_weight_shift.png` | one step | CoM y, lateral velocity, distance to the stance foot, shift blend |
| `fig_3_4_footstep_plan.png` | four-step schedule | Planner targets, not completed steps |
| `fig_3_5_swing_trajectory.png` | one step | Swing foot x, z, vertical velocity, progress |
| `fig_3_6_inverse_kinematics.png` | one step | Swing-foot command and hip pitch, knee, ankle |
| `fig_3_7_mpc_tracking.png` | one step | CoM and torso roll, pitch, yaw against the logged reference |
| `fig_3_7_mpc_error.png` | one step | CoM tracking error |
| `fig_3_7_mpc_solve_time.png` | one step | Solve-time distribution against the 20 ms period |
| `fig_3_8_cbf_intervention.png` | one step | Nominal torque, safe torque, intervention, minimum barrier |
| `fig_3_9_pd_tracking.png` | one step | Right hip pitch, knee, ankle, and nominal torque |
| `fig_3_10_single_step.png` | one step | State, foot x, CoM position and velocity through the fall |
| `fig_3_11_touchdown_failure.png` | one step | Lateral velocity, capture point, outer margins, intervention |
| `fig_full_robot_motion.png` | one step | CoM, torso, and both feet over the whole run |
| `fig_3_12_rl_touchdown_residual.png` | residual one step | Residual, adjusted reference, lateral velocity, intervention |
| `fig_3_13_rl_training.png` | 20-episode CSV | Reward, mean per-step cost, multiplier, stable flag |
| `rl_evaluation/fig_3_14_rl_vs_baseline.png` | both one-step logs | One-seed comparison. Both fall. |
| `fig_push_recovery.png` | standing push | CoM x, CoM x velocity, torso pitch, intervention |

## Still open

One step reaches touchdown and then falls off the outside of the left sole. The geometric margin can still be positive when the outward velocity is already growing. Multi-step walking waits until that landing stands up again. The 20-episode residual has not changed that outcome. There is no completed 500-episode training series to report.
