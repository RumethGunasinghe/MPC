"""Report figures from logged MuJoCo rollouts.

The script reads experiment logs and writes time-series plots. It does not
simulate the robot and it does not change the controller.

    python utils/generate_project_figures.py --log data/logs/walking_single_step_latest.npz
    python utils/generate_project_figures.py --experiment all

Run it from the ``project`` directory.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from utils.experiment_logger import load_experiment_log  # noqa: E402
from utils.plotting import plot_solve_times  # noqa: E402

FIGURES = ROOT / "data" / "figures"
LOGS = ROOT / "data" / "logs"
DPI = 300
WIDTH = 7.6
ACTUAL = "#1f4e79"
SECOND = "#c47b2b"
THIRD = "#2f6b45"
REF = "#222222"

INDEX: list[dict[str, str]] = []
SKIPPED: list[tuple[str, str]] = []


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate report figures from experiment logs")
    parser.add_argument("--log", type=Path, default=None, help="one experiment NPZ")
    parser.add_argument("--experiment", choices=("all",), default=None)
    parser.add_argument("--out", type=Path, default=FIGURES)
    args = parser.parse_args()
    if args.log is None and args.experiment is None:
        parser.error("pass --log PATH or --experiment all")
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.labelsize": 10,
            "axes.titlesize": 11,
            "legend.fontsize": 8,
            "axes.grid": True,
            "grid.alpha": 0.35,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )
    if args.experiment == "all":
        generate_all(out)
    else:
        log = load_experiment_log(args.log)
        kind = str(log.get("metadata", {}).get("experiment", ""))
        if kind == "standing_push":
            figure_push(log, out, args.log)
        elif kind == "touchdown_rl":
            figure_rl_residual(log, out, args.log)
            baseline = LOGS / "walking_single_step_latest.npz"
            if baseline.is_file():
                figure_rl_comparison(load_experiment_log(baseline), log, out, baseline, args.log)
            else:
                skip("fig_3_14_rl_vs_baseline.png", "baseline log data/logs/walking_single_step_latest.npz is missing")
        else:
            generate_walking(log, out, args.log)
            if kind == "walking_multi_step":
                figure_footsteps(log, out, args.log)
    write_index(out)
    print(f"figures written: {len(INDEX)}")
    for item in INDEX:
        print(f"  {item['filename']}")
    if SKIPPED:
        print("unavailable:")
        for name, reason in SKIPPED:
            print(f"  {name}: {reason}")


def generate_all(out: Path) -> None:
    single = LOGS / "walking_single_step_latest.npz"
    multi = LOGS / "walking_multi_step_latest.npz"
    standing = LOGS / "standing_push_latest.npz"
    residual = LOGS / "touchdown_rl_latest.npz"
    if single.is_file():
        generate_walking(load_experiment_log(single), out, single)
    else:
        for name in (
            "fig_3_3_lateral_weight_shift.png",
            "fig_3_5_swing_trajectory.png",
            "fig_3_6_inverse_kinematics.png",
            "fig_3_7_mpc_tracking.png",
            "fig_3_7_mpc_error.png",
            "fig_3_7_mpc_solve_time.png",
            "fig_3_8_cbf_intervention.png",
            "fig_3_9_pd_tracking.png",
            "fig_3_10_single_step.png",
            "fig_3_11_touchdown_failure.png",
            "fig_full_robot_motion.png",
        ):
            skip(name, f"missing {single}")
    if multi.is_file():
        figure_footsteps(load_experiment_log(multi), out, multi)
    else:
        skip("fig_3_4_footstep_plan.png", f"missing {multi}")
    if standing.is_file():
        figure_push(load_experiment_log(standing), out, standing)
    else:
        skip("fig_push_recovery.png", f"missing {standing}")
    if residual.is_file():
        learned = load_experiment_log(residual)
        figure_rl_residual(learned, out, residual)
        if single.is_file():
            figure_rl_comparison(load_experiment_log(single), learned, out, single, residual)
        else:
            skip("fig_3_14_rl_vs_baseline.png", "baseline single-step log is missing")
    else:
        skip("fig_3_12_rl_touchdown_residual.png", f"missing {residual}")
        skip("fig_3_14_rl_vs_baseline.png", f"missing {residual}")
    training = _training_table()
    if training is None:
        skip("fig_3_13_rl_training.png", "no training CSV with data rows")
    else:
        figure_training(training[0], training[1], out)


def generate_walking(log: dict, out: Path, source: Path) -> None:
    figure_lateral(log, out, source)
    figure_swing(log, out, source)
    figure_ik(log, out, source)
    figure_mpc(log, out, source)
    figure_solve_time(log, out, source)
    figure_pd(log, out, source)
    figure_cbf(log, out, source)
    figure_single_step(log, out, source)
    figure_failure(log, out, source)
    figure_motion(log, out, source)


def figure_lateral(log: dict, out: Path, source: Path) -> None:
    needed = ["time", "com", "com_velocity", "walk_state"]
    if missing(log, needed):
        skip("fig_3_3_lateral_weight_shift.png", "log has no CoM or walk state")
        return
    time, states = _time(log), _text(log["walk_state"])
    swing = _first(time, _is(states, "RIGHT_SWING"))
    stop = time[-1] if swing is None else swing + 0.2
    keep = _between(time, time[0], stop)
    t = time[keep]
    com = _xy(log["com"])[keep]
    vel = _xy(log["com_velocity"])[keep]
    distance = _optional_series(log, "com_to_stance", len(time))
    fraction = _optional_series(log, "shift_fraction", len(time))
    if distance is None and fraction is None:
        skip("fig_3_3_lateral_weight_shift.png", "log has neither com_to_stance nor shift_fraction")
        return
    events = _shift_events(time, states)
    fig, axes = _panels(4, "Lateral weight shift, through the start of the first swing")
    _series(axes[0], t, com[:, 1], "CoM y", ACTUAL)
    _series(axes[1], t, vel[:, 1], "CoM vy", ACTUAL)
    if distance is not None:
        _series(axes[2], t, distance[keep], "distance to stance-foot target", ACTUAL)
    else:
        axes[2].text(0.5, 0.5, "not logged", transform=axes[2].transAxes, ha="center")
    if fraction is not None:
        _series(axes[3], t, fraction[keep], "shift blend", ACTUAL)
    else:
        axes[3].text(0.5, 0.5, "not logged", transform=axes[3].transAxes, ha="center")
    axes[0].set_ylabel("CoM y [m]")
    axes[1].set_ylabel("CoM vy [m/s]")
    axes[2].set_ylabel("CoM to stance foot [m]")
    axes[3].set_ylabel("shift blend [-]")
    axes[-1].set_xlabel("time [s]")
    for index, axis in enumerate(axes):
        _events(axis, events, legend=index == 0)
    _save(fig, out / "fig_3_3_lateral_weight_shift.png")
    columns = {"time_s": t, "com_y_m": com[:, 1], "com_vy_m_s": vel[:, 1]}
    if distance is not None:
        columns["com_to_stance_m"] = distance[keep]
    if fraction is not None:
        columns["shift_fraction"] = fraction[keep]
    _write_csv(out / "fig_3_3_lateral_weight_shift.csv", columns)
    _note(
        "fig_3_3_lateral_weight_shift.png",
        source,
        "com_y, com_vy, com_to_stance, shift_fraction",
        "m, m/s, m, dimensionless blend in [0, 1]",
        "Show the centre of mass moving toward the stance foot before the swing.",
        "3.3 Lateral weight shift",
    )


def figure_footsteps(log: dict, out: Path, source: Path) -> None:
    meta = log.get("metadata") or {}
    if "left_foot_xyz" not in log or "right_foot_xyz" not in log:
        skip("fig_3_4_footstep_plan.png", "log has no measured initial feet")
        return
    n_steps = int(meta.get("n_steps") or 0)
    length = meta.get("step_length")
    if n_steps <= 0 or length is None:
        skip("fig_3_4_footstep_plan.png", "log metadata has no step count or step length")
        return
    from controllers.footstep_planner import FootstepConfig, FootstepPlanner

    left0 = np.asarray(log["left_foot_xyz"][0], dtype=float).reshape(3)
    right0 = np.asarray(log["right_foot_xyz"][0], dtype=float).reshape(3)
    planner = FootstepPlanner(
        FootstepConfig(
            step_length=float(length),
            step_height=float(meta.get("step_height") or 0.05),
            step_duration=float(meta.get("step_duration") or 0.8),
        )
    )
    first = str(meta.get("first_swing") or "right")
    planned = planner.plan(left0, right0, n_steps=n_steps, first_foot=first)
    step_no = np.arange(1, len(planned) + 1)
    forward = np.array([float(step.target[0]) for step in planned])
    fig, axes = plt.subplots(2, 1, figsize=(WIDTH, 6.4), sharex=False)
    fig.suptitle("Planned footsteps from FootstepPlanner (not a completed walk)", fontsize=12)
    colors = ["#c47b2b" if step.foot == "right" else "#1f4e79" for step in planned]
    axes[0].scatter(step_no, forward, c=colors, s=36, zorder=3)
    axes[0].plot(step_no, forward, color="0.45", lw=1.0)
    axes[0].set_xlabel("step number")
    axes[0].set_ylabel("planned foot x [m]")
    axes[0].set_xticks(step_no)
    axes[1].scatter([left0[0]], [left0[1]], c=ACTUAL, marker="s", s=40, label="left initial")
    axes[1].scatter([right0[0]], [right0[1]], c=SECOND, marker="s", s=40, label="right initial")
    for number, step in enumerate(planned, start=1):
        color = SECOND if step.foot == "right" else ACTUAL
        axes[1].annotate(
            "",
            xy=(step.target[0], step.target[1]),
            xytext=(step.start[0], step.start[1]),
            arrowprops={"arrowstyle": "->", "color": color, "lw": 1.1},
        )
        axes[1].scatter([step.target[0]], [step.target[1]], c=color, s=28, zorder=3)
        axes[1].text(step.target[0], step.target[1], f"  {number}", fontsize=8, va="center")
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("y [m]")
    xs = [left0[0], right0[0], *[float(step.target[0]) for step in planned], *[float(step.start[0]) for step in planned]]
    ys = [left0[1], right0[1], *[float(step.target[1]) for step in planned], *[float(step.start[1]) for step in planned]]
    pad = 0.03
    axes[1].set_xlim(min(xs) - pad, max(xs) + pad)
    axes[1].set_ylim(min(ys) - pad, max(ys) + pad)
    axes[1].set_aspect("equal", adjustable="box")
    axes[1].legend(loc="best", frameon=False)
    _save(fig, out / "fig_3_4_footstep_plan.png")
    _write_csv(
        out / "fig_3_4_footstep_plan.csv",
        {
            "step_number": step_no.astype(float),
            "foot": [step.foot for step in planned],
            "target_x_m": forward,
            "target_y_m": [float(step.target[1]) for step in planned],
            "target_z_m": [float(step.target[2]) for step in planned],
            "liftoff_x_m": [float(step.start[0]) for step in planned],
            "liftoff_y_m": [float(step.start[1]) for step in planned],
        },
    )
    _note(
        "fig_3_4_footstep_plan.png",
        source,
        "FootstepPlanner.plan targets from the logged initial ankles, step length and step count",
        "m",
        "Show the planned landing sequence. This is the planner output, not measured footfalls.",
        "3.4 Footstep planning",
    )


def figure_swing(log: dict, out: Path, source: Path) -> None:
    needed = ["time", "walk_state", "left_foot_xyz", "right_foot_xyz", "left_foot_vel", "right_foot_vel", "swing_progress"]
    if missing(log, needed):
        skip("fig_3_5_swing_trajectory.png", "swing foot traces are not in this log")
        return
    time, states = _time(log), _text(log["walk_state"])
    side = _text(log["swing_side"]) if "swing_side" in log else [""] * len(time)
    x, z, vz = _swing_channels(log, side)
    progress = np.asarray(log["swing_progress"], dtype=float).reshape(-1)
    start = _first(time, _swinging(states))
    touch = _touchdown_time(time, log)
    if start is None:
        skip("fig_3_5_swing_trajectory.png", "the log never enters a swing")
        return
    stop = time[-1] if touch is None else min(time[-1], touch + 0.25)
    keep = _between(time, start - 0.05, stop)
    swing_z = z.copy()
    swing_z[~_swinging(states)] = np.nan
    finite = np.flatnonzero(np.isfinite(swing_z) & keep)
    peak_t = float(time[finite[np.nanargmax(swing_z[finite])]]) if finite.size else None
    events = [
        (start, "swing start", THIRD),
        (peak_t, "maximum swing height", SECOND),
        (touch, "touchdown", "#8a4b08"),
    ]
    fig, axes = _panels(4, "Measured swing-foot motion")
    _series(axes[0], time[keep], x[keep], "swing foot x", ACTUAL)
    _series(axes[1], time[keep], z[keep], "swing foot z", ACTUAL)
    _series(axes[2], time[keep], vz[keep], "swing foot vz", ACTUAL)
    _series(axes[3], time[keep], progress[keep], "swing progress", ACTUAL)
    axes[0].set_ylabel("x [m]")
    axes[1].set_ylabel("z [m]")
    axes[2].set_ylabel("vz [m/s]")
    axes[3].set_ylabel("progress [-]")
    axes[-1].set_xlabel("time [s]")
    for index, axis in enumerate(axes):
        _events(axis, events, legend=index == 0)
    _save(fig, out / "fig_3_5_swing_trajectory.png")
    _write_csv(
        out / "fig_3_5_swing_trajectory.csv",
        {
            "time_s": time[keep],
            "swing_x_m": x[keep],
            "swing_z_m": z[keep],
            "swing_vz_m_s": vz[keep],
            "swing_progress": progress[keep],
        },
    )
    _note(
        "fig_3_5_swing_trajectory.png",
        source,
        "measured swing-foot x, z, vertical velocity, swing_progress",
        "m, m, m/s, fraction of the swing",
        "Show lift, forward motion, lowering and touchdown of the swing foot.",
        "3.5 Swing trajectory",
    )


def figure_ik(log: dict, out: Path, source: Path) -> None:
    if missing(log, ["time", "walk_state", "q_des", "joint_names", "swing_target"]):
        skip("fig_3_6_inverse_kinematics.png", "joint targets or swing target are not logged")
        return
    time, states = _time(log), _text(log["walk_state"])
    start = _first(time, _swinging(states))
    touch = _touchdown_time(time, log)
    if start is None:
        skip("fig_3_6_inverse_kinematics.png", "the log never enters a swing")
        return
    stop = time[-1] if touch is None else min(time[-1], touch + 0.15)
    keep = _between(time, start - 0.05, stop)
    side = "right" if "RIGHT_SWING" in states else "left"
    names = _text(log["joint_names"])
    target = np.asarray(log["swing_target"], dtype=float)
    indexes = []
    for joint in ("hip_pitch", "knee", "ankle"):
        name = f"{side}_{joint}"
        if name not in names:
            skip("fig_3_6_inverse_kinematics.png", f"{name} is not in joint_names")
            return
        indexes.append(names.index(name))
    q = np.asarray(log["q_des"], dtype=float)[:, indexes]
    low = np.asarray(log["joint_low"], dtype=float)[indexes] if "joint_low" in log else None
    high = np.asarray(log["joint_high"], dtype=float)[indexes] if "joint_high" in log else None
    fig, axes = _panels(3, f"Inverse kinematics during the {side} swing")
    _series(axes[0], time[keep], target[keep, 0], "desired x", ACTUAL)
    _series(axes[0], time[keep], target[keep, 2], "desired z", SECOND)
    labels = ("hip pitch", "knee", "ankle pitch")
    colors = (ACTUAL, SECOND, THIRD)
    for index, label in enumerate(labels):
        _series(axes[1], time[keep], q[keep, index], label, colors[index])
        if low is not None and high is not None:
            room = np.minimum(q[:, index] - low[index], high[index] - q[:, index])
            _series(axes[2], time[keep], room[keep], label, colors[index])
    axes[0].set_ylabel("desired foot [m]")
    axes[1].set_ylabel("joint target [rad]")
    axes[2].set_ylabel("distance to limit [rad]")
    axes[-1].set_xlabel("time [s]")
    for axis in axes:
        axis.legend(loc="best", frameon=False)
    _save(fig, out / "fig_3_6_inverse_kinematics.png")
    columns = {
        "time_s": time[keep],
        "desired_x_m": target[keep, 0],
        "desired_z_m": target[keep, 2],
        "hip_pitch_rad": q[keep, 0],
        "knee_rad": q[keep, 1],
        "ankle_pitch_rad": q[keep, 2],
    }
    if low is not None:
        for index, name in enumerate(("hip_pitch", "knee", "ankle_pitch")):
            columns[f"{name}_limit_distance_rad"] = np.minimum(q[keep, index] - low[index], high[index] - q[keep, index])
    _write_csv(out / "fig_3_6_inverse_kinematics.csv", columns)
    _note(
        "fig_3_6_inverse_kinematics.png",
        source,
        "swing-foot command passed to LegIK; commanded hip pitch, knee and ankle; distance of those commands to the joint limits",
        "m, rad, rad",
        "Relate the Cartesian swing target to the joint targets sent onward to the PD.",
        "3.6 Inverse kinematics",
    )


def figure_mpc(log: dict, out: Path, source: Path) -> None:
    if missing(log, ["time", "com", "com_reference", "com_error", "torso_rpy", "torso_rpy_ref"]):
        skip("fig_3_7_mpc_tracking.png", "CoM or torso reference is not logged")
        skip("fig_3_7_mpc_error.png", "CoM error is not logged")
        return
    time = _time(log)
    com = _xy(log["com"])
    ref = _xy(log["com_reference"])
    err = _xy(log["com_error"])
    torso = _xy(log["torso_rpy"])
    torso_ref = _xy(log["torso_rpy_ref"])
    labels = ("x", "y", "z")
    fig, axes = _panels(6, "Whole-body MPC task tracking")
    for index, label in enumerate(labels):
        _series(axes[index], time, ref[:, index], "reference", REF, style="--")
        _series(axes[index], time, com[:, index], "actual", ACTUAL)
        axes[index].set_ylabel(f"CoM {label} [m]")
        axes[index].legend(loc="best", frameon=False)
    for index, label in enumerate(("roll", "pitch", "yaw")):
        axis = axes[index + 3]
        _series(axis, time, torso_ref[:, index], "reference", REF, style="--")
        _series(axis, time, torso[:, index], "actual", ACTUAL)
        axis.set_ylabel(f"torso {label} [rad]")
        axis.legend(loc="best", frameon=False)
    axes[-1].set_xlabel("time [s]")
    _save(fig, out / "fig_3_7_mpc_tracking.png")
    fig_e, axes_e = _panels(3, "CoM tracking error")
    for index, label in enumerate(labels):
        _series(axes_e[index], time, err[:, index], f"error {label}", ACTUAL)
        axes_e[index].set_ylabel(f"error {label} [m]")
        axes_e[index].axhline(0.0, color="0.4", lw=0.8)
    axes_e[-1].set_xlabel("time [s]")
    _save(fig_e, out / "fig_3_7_mpc_error.png")
    _note(
        "fig_3_7_mpc_tracking.png",
        source,
        "measured CoM and torso RPY against the reference sent to WholeBodyMPC",
        "m, rad",
        "Compare the task reference with the motion MuJoCo produced.",
        "3.7 Whole-body MPC",
    )
    _note(
        "fig_3_7_mpc_error.png",
        source,
        "com_error_x, com_error_y, com_error_z",
        "m",
        "CoM tracking error, actual minus the logged reference.",
        "3.7 Whole-body MPC",
    )


def figure_solve_time(log: dict, out: Path, source: Path) -> None:
    if "mpc_solve_s" not in log:
        skip("fig_3_7_mpc_solve_time.png", "mpc_solve_s was not logged")
        return
    samples = np.asarray(log["mpc_solve_s"], dtype=float).reshape(-1) * 1e3
    samples = samples[np.isfinite(samples)]
    if samples.size == 0:
        skip("fig_3_7_mpc_solve_time.png", "no finite solve times")
        return
    period = float(log.get("metadata", {}).get("control_dt") or 0.02) * 1e3
    fig = plot_solve_times(samples, period)
    axis = fig.axes[0]
    stats = {
        "count": int(samples.size),
        "mean_ms": float(np.mean(samples)),
        "median_ms": float(np.median(samples)),
        "p95_ms": float(np.percentile(samples, 95)),
        "max_ms": float(np.max(samples)),
        "control_period_ms": period,
    }
    axis.set_xlim(0.0, max(period, stats["max_ms"]) * 1.08)
    axis.text(
        0.98,
        0.98,
        "\n".join(
            (
                f"mean {stats['mean_ms']:.2f} ms",
                f"median {stats['median_ms']:.2f} ms",
                f"95th {stats['p95_ms']:.2f} ms",
                f"max {stats['max_ms']:.2f} ms",
            )
        ),
        transform=axis.transAxes,
        ha="right",
        va="top",
        fontsize=8,
        bbox={"boxstyle": "round", "fc": "white", "ec": "0.8"},
    )
    fig.tight_layout()
    path = out / "fig_3_7_mpc_solve_time.png"
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    (out / "fig_3_7_mpc_solve_time.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
    _note(
        "fig_3_7_mpc_solve_time.png",
        source,
        "wall-clock duration of WholeBodyMPC.solve",
        "ms",
        "Compare the MPC solve time with the 20 ms control period.",
        "3.7 MPC timing",
    )


def figure_pd(log: dict, out: Path, source: Path) -> None:
    if missing(log, ["time", "q_des", "joint_qpos", "tau_nominal", "joint_names", "walk_state"]):
        skip("fig_3_9_pd_tracking.png", "PD joint traces are not logged")
        return
    time, states = _time(log), _text(log["walk_state"])
    names = _text(log["joint_names"])
    side = "right" if "RIGHT_SWING" in states else "left"
    joints = ("hip_pitch", "knee", "ankle")
    indexes = []
    for joint in joints:
        name = f"{side}_{joint}"
        if name not in names:
            skip("fig_3_9_pd_tracking.png", f"{name} is not in joint_names")
            return
        indexes.append(names.index(name))
    desired = np.asarray(log["q_des"], dtype=float)[:, indexes]
    actual = np.asarray(log["joint_qpos"], dtype=float)[:, indexes]
    torque = np.asarray(log["tau_nominal"], dtype=float)[:, indexes]
    start = _first(time, _swinging(states))
    keep = _between(time, time[0] if start is None else start - 0.2, time[-1])
    fig, axes = _panels(4, f"PD tracking on the {side} leg, from swing through the end of the log")
    titles = ("hip pitch", "knee", "ankle pitch")
    for index, title in enumerate(titles):
        _series(axes[index], time[keep], desired[keep, index], "desired", REF, style="--")
        _series(axes[index], time[keep], actual[keep, index], "actual", ACTUAL)
        axes[index].set_ylabel(f"{title} [rad]")
        axes[index].legend(loc="best", frameon=False)
    for index, title in enumerate(titles):
        _series(axes[3], time[keep], torque[keep, index], title, (ACTUAL, SECOND, THIRD)[index])
    axes[3].set_ylabel("nominal torque [N·m]")
    axes[3].legend(loc="best", frameon=False)
    axes[-1].set_xlabel("time [s]")
    _save(fig, out / "fig_3_9_pd_tracking.png")
    _write_csv(
        out / "fig_3_9_pd_tracking.csv",
        {
            "time_s": time[keep],
            "hip_pitch_des_rad": desired[keep, 0],
            "hip_pitch_rad": actual[keep, 0],
            "knee_des_rad": desired[keep, 1],
            "knee_rad": actual[keep, 1],
            "ankle_des_rad": desired[keep, 2],
            "ankle_rad": actual[keep, 2],
            "hip_pitch_nominal_nm": torque[keep, 0],
            "knee_nominal_nm": torque[keep, 1],
            "ankle_nominal_nm": torque[keep, 2],
        },
    )
    _note(
        "fig_3_9_pd_tracking.png",
        source,
        f"desired and measured {side} hip pitch, knee and ankle, and the nominal PD-plus-gravity torque",
        "rad, N·m",
        "Show how the swing leg follows the joint targets, including the landing transient.",
        "3.9 PD joint tracking",
    )


def figure_cbf(log: dict, out: Path, source: Path) -> None:
    if missing(log, ["time", "tau_nominal", "tau_safe", "intervention", "cbf_status"]):
        skip("fig_3_8_cbf_intervention.png", "CBF torques are not logged")
        return
    time = _time(log)
    nominal = np.linalg.norm(np.asarray(log["tau_nominal"], dtype=float), axis=1)
    safe = np.linalg.norm(np.asarray(log["tau_safe"], dtype=float), axis=1)
    correction = np.asarray(log["intervention"], dtype=float).reshape(-1)
    margins = _support_margins(log)
    states = _text(log["walk_state"]) if "walk_state" in log else [""] * len(time)
    status = _text(log["cbf_status"])
    events = []
    if "push_active" in log and np.any(np.asarray(log["push_active"], dtype=float) > 0.5):
        events.append((_first(time, np.asarray(log["push_active"], dtype=float) > 0.5), "push start", "0.35"))
    events.append((_first(time, _swinging(states)), "swing start", THIRD))
    events.append((_touchdown_time(time, log), "touchdown", "#8a4b08"))
    events.append((_first(time, np.array([item != "optimal" for item in status])), "first non-optimal CBF", "#6b3fa0"))
    events.append((_first(time, np.array([item == "fail_safe" for item in status])), "fail-safe", "#a33b3b"))
    fig, axes = _panels(4, "CBF torque correction and support-box margin")
    _series(axes[0], time, nominal, "nominal", ACTUAL)
    _series(axes[1], time, safe, "safe", ACTUAL)
    _series(axes[2], time, correction, "intervention", "#a33b3b")
    if margins is not None:
        _series(axes[3], time, np.min(margins, axis=1), "minimum signed margin", ACTUAL)
        axes[3].axhline(0.0, color="0.35", lw=0.8)
    axes[0].set_ylabel("||tau nominal|| [N·m]")
    axes[1].set_ylabel("||tau safe|| [N·m]")
    axes[2].set_ylabel("||tau safe - tau nom|| [N·m]")
    axes[3].set_ylabel("min barrier [m]")
    axes[-1].set_xlabel("time [s]")
    for index, axis in enumerate(axes):
        _events(axis, events, legend=index == 0)
    fig.text(
        0.01,
        0.005,
        "Barrier values are signed distances to the support box used by the filter. They are not a formal safety certificate.",
        fontsize=7.5,
        color="0.25",
    )
    _save(fig, out / "fig_3_8_cbf_intervention.png")
    columns = {
        "time_s": time,
        "nominal_torque_norm_nm": nominal,
        "safe_torque_norm_nm": safe,
        "intervention_nm": correction,
        "cbf_status": status,
    }
    if margins is not None:
        columns["min_barrier_m"] = np.min(margins, axis=1)
    _write_csv(out / "fig_3_8_cbf_intervention.csv", columns)
    _note(
        "fig_3_8_cbf_intervention.png",
        source,
        "norms of nominal and safe torque, intervention norm, minimum support-box barrier, CBF status",
        "N·m, m",
        "Show when the safety filter changes the torque. A negative barrier means the CoM is outside that face of the box.",
        "3.8 CBF intervention",
    )


def figure_single_step(log: dict, out: Path, source: Path) -> None:
    if missing(log, ["time", "walk_state", "left_foot_xyz", "right_foot_xyz", "com", "com_velocity"]):
        skip("fig_3_10_single_step.png", "the single-step traces are incomplete")
        return
    time, states = _time(log), _text(log["walk_state"])
    order = ["STAND", "SHIFT_LEFT", "RIGHT_SWING", "DOUBLE_SUPPORT", "SHIFT_RIGHT", "LEFT_SWING"]
    codes = np.array([order.index(name) if name in order else -1 for name in states], dtype=float)
    left = _xy(log["left_foot_xyz"])
    right = _xy(log["right_foot_xyz"])
    com = _xy(log["com"])
    vel = _xy(log["com_velocity"])
    events = _state_changes(time, states)
    touch = _touchdown_time(time, log)
    if touch is not None:
        events.append((touch, "touchdown", "#8a4b08"))
    fig, axes = _panels(4, "One-step sequence")
    axes[0].step(time, codes, where="post", color=ACTUAL, lw=1.4)
    axes[0].set_yticks(range(len(order)))
    axes[0].set_yticklabels(order)
    axes[0].set_ylabel("walk state")
    _series(axes[1], time, left[:, 0], "left foot", ACTUAL)
    _series(axes[1], time, right[:, 0], "right foot", SECOND)
    axes[1].set_ylabel("foot x [m]")
    axes[1].legend(loc="best", frameon=False)
    _series(axes[2], time, com[:, 0], "CoM x", ACTUAL)
    _series(axes[2], time, com[:, 1], "CoM y", SECOND)
    axes[2].set_ylabel("CoM [m]")
    axes[2].legend(loc="best", frameon=False)
    _series(axes[3], time, vel[:, 0], "vx", ACTUAL)
    _series(axes[3], time, vel[:, 1], "vy", SECOND)
    axes[3].set_ylabel("CoM velocity [m/s]")
    axes[3].legend(loc="best", frameon=False)
    axes[-1].set_xlabel("time [s]")
    for index, axis in enumerate(axes):
        _events(axis, events, legend=index == 1)
    _save(fig, out / "fig_3_10_single_step.png")
    _write_csv(
        out / "fig_3_10_single_step.csv",
        {
            "time_s": time,
            "walk_state": states,
            "left_foot_x_m": left[:, 0],
            "right_foot_x_m": right[:, 0],
            "com_x_m": com[:, 0],
            "com_y_m": com[:, 1],
            "com_vx_m_s": vel[:, 0],
            "com_vy_m_s": vel[:, 1],
        },
    )
    _note(
        "fig_3_10_single_step.png",
        source,
        "walk state, left and right foot x, CoM x/y and velocity",
        "s, m, m/s",
        "Show the state sequence through one step, including the landing and any fall.",
        "3.10 Single-step experiment",
    )


def figure_failure(log: dict, out: Path, source: Path) -> None:
    needed = ["time", "com_velocity", "capture_xy", "intervention", "walk_state", "cbf_status", "com"]
    if missing(log, needed) or "outer_capture_margin" not in log:
        skip("fig_3_11_touchdown_failure.png", "capture margin or capture point is not logged")
        return
    time, states = _time(log), _text(log["walk_state"])
    progress = np.asarray(log["swing_progress"], dtype=float).reshape(-1) if "swing_progress" in log else np.zeros(len(time))
    late = _first(time, _swinging(states) & (progress >= 0.65))
    swing_end = _last(time, _is(states, "RIGHT_SWING")) or _last(time, _is(states, "LEFT_SWING"))
    touch = _touchdown_time(time, log)
    status = _text(log["cbf_status"])
    nonoptimal = _first(time, np.array([item != "optimal" for item in status]))
    fall = _first(time, _xy(log["com"])[:, 2] < 0.72)
    start = time[0] if late is None else late - 0.15
    keep = _between(time, start, time[-1])
    events = [
        (late, "late swing", THIRD),
        (swing_end, "swing end", "0.35"),
        (touch, "touchdown / first double support", "#8a4b08"),
        (nonoptimal, "first non-optimal CBF", "#6b3fa0"),
        (fall, "fall", "#a33b3b"),
    ]
    fig, axes = _panels(4, "Observed touchdown: margin, lateral velocity and the filter")
    _series(axes[0], time[keep], _xy(log["com_velocity"])[keep, 1], "vy", ACTUAL)
    _series(axes[1], time[keep], np.asarray(log["capture_xy"], dtype=float)[keep, 1], "capture y", ACTUAL)
    _series(axes[2], time[keep], np.asarray(log["outer_capture_margin"], dtype=float)[keep], "outer capture margin", ACTUAL)
    if "outer_support_margin" in log:
        _series(axes[2], time[keep], np.asarray(log["outer_support_margin"], dtype=float)[keep], "outer support margin", SECOND)
        axes[2].legend(loc="best", frameon=False)
    axes[2].axhline(0.0, color="0.35", lw=0.8)
    _series(axes[3], time[keep], np.asarray(log["intervention"], dtype=float).reshape(-1)[keep], "intervention", "#a33b3b")
    axes[0].set_ylabel("vy [m/s]")
    axes[1].set_ylabel("capture y [m]")
    axes[2].set_ylabel("margin [m]")
    axes[3].set_ylabel("intervention [N·m]")
    axes[-1].set_xlabel("time [s]")
    for index, axis in enumerate(axes):
        _events(axis, events, legend=index == 0)
    fig.text(
        0.01,
        0.005,
        "Observed traces from the deterministic one-step run. They do not by themselves prove a single cause.",
        fontsize=7.5,
        color="0.25",
    )
    _save(fig, out / "fig_3_11_touchdown_failure.png")
    columns = {
        "time_s": time[keep],
        "com_vy_m_s": _xy(log["com_velocity"])[keep, 1],
        "capture_y_m": np.asarray(log["capture_xy"], dtype=float)[keep, 1],
        "outer_capture_margin_m": np.asarray(log["outer_capture_margin"], dtype=float)[keep],
        "intervention_nm": np.asarray(log["intervention"], dtype=float).reshape(-1)[keep],
    }
    if "outer_support_margin" in log:
        columns["outer_support_margin_m"] = np.asarray(log["outer_support_margin"], dtype=float)[keep]
    _write_csv(out / "fig_3_11_touchdown_failure.csv", columns)
    _note(
        "fig_3_11_touchdown_failure.png",
        source,
        "lateral CoM velocity, lateral capture point, outer capture and support margins, CBF intervention",
        "m/s, m, m, N·m",
        "Document the measured landing failure. A positive margin can coexist with outward velocity.",
        "3.11 Touchdown failure",
    )


def figure_motion(log: dict, out: Path, source: Path) -> None:
    needed = ["time", "com", "com_velocity", "torso_rpy", "left_foot_xyz", "right_foot_xyz"]
    if missing(log, needed):
        skip("fig_full_robot_motion.png", "full-motion traces are incomplete")
        return
    time = _time(log)
    com = _xy(log["com"])
    vel = _xy(log["com_velocity"])
    torso = _xy(log["torso_rpy"])
    left = _xy(log["left_foot_xyz"])
    right = _xy(log["right_foot_xyz"])
    states = _text(log["walk_state"]) if "walk_state" in log else None
    events = _state_changes(time, states) if states is not None else []
    fig, axes = _panels(6, "Measured robot motion")
    _series(axes[0], time, com[:, 0], "x", ACTUAL)
    _series(axes[0], time, com[:, 1], "y", SECOND)
    axes[0].set_ylabel("CoM x, y [m]")
    _series(axes[1], time, com[:, 2], "z", ACTUAL)
    axes[1].set_ylabel("CoM z [m]")
    _series(axes[2], time, vel[:, 0], "vx", ACTUAL)
    _series(axes[2], time, vel[:, 1], "vy", SECOND)
    axes[2].set_ylabel("CoM velocity [m/s]")
    _series(axes[3], time, torso[:, 0], "roll", ACTUAL)
    _series(axes[3], time, torso[:, 1], "pitch", SECOND)
    axes[3].set_ylabel("torso [rad]")
    _series(axes[4], time, left[:, 0], "left", ACTUAL)
    _series(axes[4], time, right[:, 0], "right", SECOND)
    axes[4].set_ylabel("foot x [m]")
    _series(axes[5], time, left[:, 2], "left", ACTUAL)
    _series(axes[5], time, right[:, 2], "right", SECOND)
    axes[5].set_ylabel("foot z [m]")
    for axis in axes:
        axis.legend(loc="best", frameon=False)
    axes[-1].set_xlabel("time [s]")
    for axis in axes:
        _events(axis, events, legend=False)
    _save(fig, out / "fig_full_robot_motion.png")
    _write_csv(
        out / "fig_full_robot_motion.csv",
        {
            "time_s": time,
            "com_x_m": com[:, 0],
            "com_y_m": com[:, 1],
            "com_z_m": com[:, 2],
            "com_vx_m_s": vel[:, 0],
            "com_vy_m_s": vel[:, 1],
            "torso_roll_rad": torso[:, 0],
            "torso_pitch_rad": torso[:, 1],
            "left_foot_x_m": left[:, 0],
            "right_foot_x_m": right[:, 0],
            "left_foot_z_m": left[:, 2],
            "right_foot_z_m": right[:, 2],
        },
    )
    _note(
        "fig_full_robot_motion.png",
        source,
        "CoM position and horizontal velocity, torso roll and pitch, both feet x and z",
        "m, m/s, rad",
        "Overview of the motion MuJoCo produced in this experiment.",
        "Full robot motion",
    )


def figure_rl_residual(log: dict, out: Path, source: Path) -> None:
    if missing(log, ["time", "rl_residual", "rl_active", "com_reference", "com_reference_deterministic", "com_velocity", "intervention"]):
        skip("fig_3_12_rl_touchdown_residual.png", "this log has no RL residual trace")
        return
    active = np.asarray(log["rl_active"], dtype=float).reshape(-1) > 0.5
    if not np.any(active):
        skip("fig_3_12_rl_touchdown_residual.png", "the residual was never active in this log")
        return
    time_full = _time(log)
    start = _first(time_full, active)
    stop = _last(time_full, active)
    touch = _touchdown_time(time_full, log)
    window = _between(time_full, time_full[0] if start is None else start - 0.4, time_full[-1])
    time = time_full[window]
    residual = np.asarray(log["rl_residual"], dtype=float)[window]
    det = _xy(log["com_reference_deterministic"])[window]
    adj = _xy(log["com_reference"])[window]
    bound = log.get("residual_bound")
    fig, axes = _panels(4, "Touchdown residual during activation. Not a successful recovery.")
    _series(axes[0], time, residual[:, 0], "residual x", ACTUAL)
    _series(axes[1], time, residual[:, 1], "residual y", ACTUAL)
    if bound is not None:
        limits = np.asarray(bound, dtype=float).reshape(-1)
        for axis, limit in ((axes[0], limits[0]), (axes[1], limits[1])):
            axis.axhline(float(limit), color="0.4", ls=":", lw=0.9, label="± bound" if axis is axes[0] else None)
            axis.axhline(-float(limit), color="0.4", ls=":", lw=0.9)
        span0 = max(abs(float(limits[0])), float(np.nanmax(np.abs(residual[:, 0])))) * 1.15
        span1 = max(abs(float(limits[1])), float(np.nanmax(np.abs(residual[:, 1])))) * 1.15
        axes[0].set_ylim(-span0, span0)
        axes[1].set_ylim(-span1, span1)
    _series(axes[2], time, det[:, 1], "deterministic y", REF, style="--")
    _series(axes[2], time, adj[:, 1], "RL-adjusted y", ACTUAL)
    _series(axes[2], time, det[:, 0], "deterministic x", "0.55", style="--")
    _series(axes[2], time, adj[:, 0], "RL-adjusted x", SECOND)
    vel = _xy(log["com_velocity"])[window, 1]
    correction = np.asarray(log["intervention"], dtype=float).reshape(-1)[window]
    _series(axes[3], time, vel, "vy", ACTUAL)
    twin = axes[3].twinx()
    twin.plot(time, correction, color="#a33b3b", lw=1.2, label="CBF intervention")
    twin.set_ylabel("intervention [N·m]")
    twin.grid(False)
    axes[0].set_ylabel("residual x [m]")
    axes[1].set_ylabel("residual y [m]")
    axes[2].set_ylabel("CoM reference [m]")
    axes[3].set_ylabel("vy [m/s]")
    axes[-1].set_xlabel("time [s]")
    for axis in axes[:3]:
        axis.legend(loc="best", frameon=False)
    handles, labels = axes[3].get_legend_handles_labels()
    handles2, labels2 = twin.get_legend_handles_labels()
    axes[3].legend(handles + handles2, labels + labels2, loc="best", frameon=False)
    events = [(start, "RL activation", ACTUAL), (touch, "touchdown", "#8a4b08"), (stop, "RL deactivation", "0.35")]
    for index, axis in enumerate(list(axes) + [twin]):
        _events(axis, events, legend=index == 0)
    _save(fig, out / "fig_3_12_rl_touchdown_residual.png")
    _write_csv(
        out / "fig_3_12_rl_touchdown_residual.csv",
        {
            "time_s": time,
            "rl_active": active[window].astype(float),
            "residual_x_m": residual[:, 0],
            "residual_y_m": residual[:, 1],
            "com_ref_x_m": det[:, 0],
            "com_ref_y_m": det[:, 1],
            "com_ref_rl_x_m": adj[:, 0],
            "com_ref_rl_y_m": adj[:, 1],
            "com_vy_m_s": vel,
            "intervention_nm": correction,
        },
    )
    _note(
        "fig_3_12_rl_touchdown_residual.png",
        source,
        "RL residual, deterministic and adjusted CoM references, lateral velocity, CBF intervention",
        "m, m/s, N·m",
        "Show the bounded centre-of-mass residual. Do not read this figure as a successful landing.",
        "3.12 RL touchdown residual",
    )


def figure_training(table: list[dict[str, str]], note_path: Path, out: Path) -> None:
    episode = np.array([float(row["episode"]) for row in table])
    reward = np.array([float(row["reward"]) for row in table])
    cost = np.array([float(row["mean_cbf_cost"]) for row in table])
    lagrange = np.array([float(row["lambda"]) for row in table])
    stable = np.array([float(row["stable"]) for row in table])
    budget = float(table[0]["cost_budget"]) if "cost_budget" in table[0] else None
    fig, axes = _panels(4, "Touchdown RL training log. Stable touchdowns in this file: 0.")
    _series(axes[0], episode, reward, "batch mean reward", ACTUAL)
    _series(axes[1], episode, cost, "mean per-step CBF cost", SECOND)
    if budget is not None:
        axes[1].axhline(budget, color="0.2", ls="--", lw=1.0, label=f"cost budget {budget:g}")
        axes[1].legend(loc="best", frameon=False)
    _series(axes[2], episode, lagrange, "Lagrange multiplier", ACTUAL)
    _series(axes[3], episode, stable, "stable touchdown in batch", "#a33b3b")
    axes[3].set_ylim(-0.05, 1.05)
    axes[0].set_ylabel("batch mean reward")
    axes[1].set_ylabel("mean CBF cost")
    axes[2].set_ylabel("lambda")
    axes[3].set_ylabel("stable flag")
    axes[-1].set_xlabel("episode")
    fig.text(
        0.01,
        0.005,
        "Logged updates from the 20-episode cost-fix run. No completed 500-episode series was available. Reward is the batch mean, not a discounted return.",
        fontsize=7.5,
        color="0.25",
    )
    _save(fig, out / "fig_3_13_rl_training.png")
    _write_csv(
        out / "fig_3_13_rl_training.csv",
        {
            "episode": episode,
            "steps": [float(row["steps"]) for row in table],
            "reward": reward,
            "mean_cbf_cost": cost,
            "max_cbf_cost": [float(row["max_cbf_cost"]) for row in table],
            "lambda": lagrange,
            "stable": stable,
        },
    )
    _note(
        "fig_3_13_rl_training.png",
        note_path,
        "batch mean reward, mean per-step CBF cost, Lagrange multiplier, stable flag",
        "reward units of the touchdown reward, cost in trust-region units, lambda dimensionless, stable in {0, 1}",
        "Show the recorded training updates. The stable-touchdown count in this log is zero.",
        "3.13 RL training",
    )


def figure_rl_comparison(baseline: dict, learned: dict, out: Path, base_path: Path, rl_path: Path) -> None:
    left = _trial_metrics(baseline)
    right = _trial_metrics(learned)
    dest = out / "rl_evaluation"
    dest.mkdir(parents=True, exist_ok=True)
    fields = list(left)
    rows = [{"controller": "baseline", **left}, {"controller": "rl", **right}]
    with (dest / "rl_vs_baseline_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["controller", *fields])
        writer.writeheader()
        writer.writerows(rows)
    payload = {
        "baseline_log": str(base_path),
        "rl_log": str(rl_path),
        "seed": baseline.get("metadata", {}).get("seed"),
        "note": "One identical seed. A rate of 0 or 1 is that single trial, not a multi-seed study. This comparison does not show a reliable landing.",
        "baseline": left,
        "rl": right,
    }
    (dest / "rl_vs_baseline_metrics.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    labels = ["touchdown", "stable landing", "fall"]
    keys = ["touchdown_rate", "stable_landing_rate", "fall_rate"]
    fig, axes = plt.subplots(1, 2, figsize=(WIDTH + 1.4, 4.2))
    xpos = np.arange(len(labels))
    axes[0].bar(xpos - 0.16, [left[key] for key in keys], width=0.32, label="baseline", color=ACTUAL)
    axes[0].bar(xpos + 0.16, [right[key] for key in keys], width=0.32, label="RL residual", color=SECOND)
    axes[0].set_xticks(xpos)
    axes[0].set_xticklabels(labels, rotation=15, ha="right")
    axes[0].set_ylim(0.0, 1.05)
    axes[0].set_ylabel("outcome of one trial")
    axes[0].legend(loc="best", frameon=False)
    axes[0].set_title("Same seed")
    metric_names = [
        "min support margin [m]",
        "min capture margin [m]",
        "peak |vy| [m/s]",
        "vy at double support [m/s]",
        "mean intervention [N·m]",
        "peak intervention [N·m]",
        "mean |residual| [m]",
        "max |residual| [m]",
    ]
    metric_keys = [
        "min_support_margin_m",
        "min_capture_margin_m",
        "peak_abs_vy_m_s",
        "vy_at_double_support_m_s",
        "mean_cbf_intervention_nm",
        "peak_cbf_intervention_nm",
        "mean_rl_residual_m",
        "max_rl_residual_m",
    ]
    axes[1].axis("off")
    cell = [[f"{left[key]:.4g}", f"{right[key]:.4g}"] for key in metric_keys]
    table = axes[1].table(cellText=cell, rowLabels=metric_names, colLabels=("baseline", "RL"), loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(7.5)
    table.scale(1.05, 1.25)
    fig.suptitle("Baseline and touchdown residual, one shared seed. Neither result is a stable landing.")
    fig.tight_layout()
    path = dest / "fig_3_14_rl_vs_baseline.png"
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    _note(
        "rl_evaluation/fig_3_14_rl_vs_baseline.png",
        base_path,
        "touchdown, stable landing, fall, margins, lateral velocity, CBF intervention, residual size",
        "rates in {0, 1} for one trial; m, m/s, N·m",
        "Compare one deterministic step with the same step plus the loaded residual. Not evidence of improvement.",
        "3.14 RL evaluation",
    )


def figure_push(log: dict, out: Path, source: Path) -> None:
    if missing(log, ["time", "com", "com_velocity", "torso_rpy", "intervention"]):
        skip("fig_push_recovery.png", "standing push traces are incomplete")
        return
    time = _time(log)
    com = _xy(log["com"])
    vel = _xy(log["com_velocity"])
    pitch = _xy(log["torso_rpy"])[:, 1]
    correction = np.asarray(log["intervention"], dtype=float).reshape(-1)
    meta = log.get("metadata") or {}
    window = meta.get("push_window_s") or [2.0, 2.2]
    force = meta.get("push_force_command") or [80.0, 0.0, 0.0]
    fig, axes = _panels(4, f"Standing push recovery, {float(force[0]):.0f} N for {float(window[1]) - float(window[0]):.1f} s")
    _series(axes[0], time, com[:, 0], "CoM x", ACTUAL)
    _series(axes[1], time, vel[:, 0], "vx", ACTUAL)
    _series(axes[2], time, pitch, "torso pitch", ACTUAL)
    _series(axes[3], time, correction, "intervention", "#a33b3b")
    axes[0].set_ylabel("CoM x [m]")
    axes[1].set_ylabel("vx [m/s]")
    axes[2].set_ylabel("torso pitch [rad]")
    axes[3].set_ylabel("intervention [N·m]")
    axes[-1].set_xlabel("time [s]")
    events = [(float(window[0]), "push start", "0.35"), (float(window[1]), "push end", "0.35")]
    for index, axis in enumerate(axes):
        axis.axvspan(float(window[0]), float(window[1]), color="0.85", zorder=0)
        _events(axis, events, legend=index == 0)
    _save(fig, out / "fig_push_recovery.png")
    _write_csv(
        out / "fig_push_recovery.csv",
        {
            "time_s": time,
            "com_x_m": com[:, 0],
            "com_vx_m_s": vel[:, 0],
            "torso_pitch_rad": pitch,
            "intervention_nm": correction,
        },
    )
    peak_index = int(np.nanargmax(np.abs(pitch)))
    summary = {
        "log": str(source),
        "push_force_n": [float(value) for value in force],
        "push_window_s": [float(window[0]), float(window[1])],
        "peak_pitch_rad": float(pitch[peak_index]),
        "peak_pitch_deg": float(np.degrees(pitch[peak_index])),
        "peak_com_vx_m_s": float(vel[int(np.nanargmax(np.abs(vel[:, 0]))), 0]),
        "peak_com_speed_m_s": float(np.nanmax(np.linalg.norm(vel, axis=1))),
        "max_com_x_displacement_m": float(np.nanmax(com[:, 0]) - com[0, 0]),
        "min_com_x_displacement_m": float(np.nanmin(com[:, 0]) - com[0, 0]),
        "peak_cbf_intervention_nm": float(np.nanmax(correction)),
        "final_com_m": [float(value) for value in com[-1]],
        "final_torso_pitch_rad": float(pitch[-1]),
        "final_robot_height_m": float(com[-1, 2]),
    }
    (out / "push_recovery_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    _note(
        "fig_push_recovery.png",
        source,
        "CoM x, CoM vx, torso pitch, CBF intervention during the standing push",
        "m, m/s, rad, N·m",
        "Standing recovery from the logged torso push.",
        "Push recovery",
    )


def _trial_metrics(log: dict) -> dict[str, float]:
    time = _time(log)
    com = _xy(log["com"])
    vel = _xy(log["com_velocity"])
    states = _text(log["walk_state"])
    touch = None
    seen = False
    for index, name in enumerate(states):
        if name in ("RIGHT_SWING", "LEFT_SWING"):
            seen = True
        if seen and name == "DOUBLE_SUPPORT" and touch is None:
            touch = index
    stable_flag = np.asarray(log["stable_double_support"], dtype=float).reshape(-1) if "stable_double_support" in log else np.zeros(len(time))
    stable = bool(np.any(stable_flag > 0.5))
    fell = bool(np.any(com[:, 2] < 0.72))
    margins = _support_margins(log)
    capture = _capture_margins(log)
    residual = np.asarray(log["rl_residual"], dtype=float) if "rl_residual" in log else np.zeros((len(time), 2))
    residual_norm = np.linalg.norm(residual, axis=1)
    intervention = np.asarray(log["intervention"], dtype=float).reshape(-1)
    stable_time = -1.0
    if stable and touch is not None:
        first_stable = _first(time, stable_flag > 0.5)
        if first_stable is not None:
            stable_time = float(first_stable - time[touch])
    return {
        "touchdown_rate": float(touch is not None),
        "stable_landing_rate": float(stable),
        "fall_rate": float(fell),
        "mean_time_to_stable_s": stable_time,
        "min_support_margin_m": float(np.nanmin(margins)) if margins is not None else float("nan"),
        "min_capture_margin_m": float(np.nanmin(capture)) if capture is not None else float("nan"),
        "peak_abs_vy_m_s": float(np.nanmax(np.abs(vel[:, 1]))),
        "vy_at_double_support_m_s": float(vel[touch, 1]) if touch is not None else float("nan"),
        "mean_cbf_intervention_nm": float(np.nanmean(intervention)),
        "peak_cbf_intervention_nm": float(np.nanmax(intervention)),
        "mean_rl_residual_m": float(np.nanmean(residual_norm)),
        "max_rl_residual_m": float(np.nanmax(residual_norm)),
    }


def _training_table() -> tuple[list[dict[str, str]], Path] | None:
    latest = LOGS / "touchdown_training_latest.csv"
    archived = LOGS / "touchdown_training_20ep.csv"
    for path in (latest, archived):
        if not path.is_file() or path.stat().st_size == 0:
            continue
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        if rows:
            return rows, path
    return None


def _panels(count: int, title: str):
    fig, axes = plt.subplots(count, 1, sharex=True, figsize=(WIDTH, 1.72 * count + 0.85), squeeze=False)
    fig.suptitle(title, fontsize=12)
    return fig, axes[:, 0]


def _series(axis, time, values, label: str, color: str, style: str = "-") -> None:
    axis.plot(time, values, style, color=color, lw=1.35, label=label)


def _events(axis, events, legend: bool) -> None:
    seen: set[str] = set()
    for instant, label, color in events:
        if instant is None or not np.isfinite(instant):
            continue
        show = legend and label not in seen
        axis.axvline(float(instant), color=color, lw=0.9, ls="--", label=label if show else None)
        seen.add(label)
    if legend and seen:
        axis.legend(loc="best", fontsize=7, frameon=False)


def _save(fig, path: Path) -> None:
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)


def _write_csv(path: Path, columns: dict) -> None:
    keys = list(columns)
    length = len(np.asarray(columns[keys[0]]).reshape(-1))
    lines = [",".join(keys)]
    for index in range(length):
        cells = []
        for key in keys:
            value = columns[key][index]
            if isinstance(value, str):
                cells.append(value)
            else:
                number = float(value)
                cells.append("" if not np.isfinite(number) else f"{number:.8g}")
        lines.append(",".join(cells))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _note(filename: str, source: Path, variables: str, units: str, purpose: str, section: str) -> None:
    INDEX.append(
        {
            "filename": filename,
            "log": str(source),
            "variables": variables,
            "units": units,
            "purpose": purpose,
            "section": section,
        }
    )


def skip(name: str, reason: str) -> None:
    SKIPPED.append((name, reason))


def write_index(out: Path) -> None:
    lines = ["Figure index", "============", ""]
    for item in INDEX:
        lines.extend(
            [
                f"filename: {item['filename']}",
                f"experiment/log: {item['log']}",
                f"variables: {item['variables']}",
                f"units: {item['units']}",
                f"purpose: {item['purpose']}",
                f"report section: {item['section']}",
                "",
            ]
        )
    if SKIPPED:
        lines.append("Unavailable")
        lines.append("-----------")
        for name, reason in SKIPPED:
            lines.append(f"{name}: {reason}")
        lines.append("")
    (out / "FIGURE_INDEX.txt").write_text("\n".join(lines), encoding="utf-8")


def missing(log: dict, keys: list[str]) -> bool:
    return any(key not in log for key in keys)


def _time(log: dict) -> np.ndarray:
    return np.asarray(log["time"], dtype=float).reshape(-1)


def _xy(values) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim == 1:
        array = array.reshape(-1, 1)
    return array


def _text(values) -> list[str]:
    out = []
    for value in np.asarray(values).reshape(-1):
        if isinstance(value, bytes):
            out.append(value.decode())
        else:
            out.append(str(value))
    return out


def _optional_series(log: dict, key: str, length: int) -> np.ndarray | None:
    if key not in log:
        return None
    return np.asarray(log[key], dtype=float).reshape(-1)


def _between(time: np.ndarray, start: float, stop: float) -> np.ndarray:
    return (time >= start) & (time <= stop)


def _first(time: np.ndarray, mask) -> float | None:
    index = np.flatnonzero(np.asarray(mask, dtype=bool))
    if index.size == 0:
        return None
    return float(time[index[0]])


def _last(time: np.ndarray, mask) -> float | None:
    index = np.flatnonzero(np.asarray(mask, dtype=bool))
    if index.size == 0:
        return None
    return float(time[index[-1]])


def _is(states: list[str], name: str) -> np.ndarray:
    return np.array([item == name for item in states])


def _swinging(states: list[str]) -> np.ndarray:
    return np.array([item in ("RIGHT_SWING", "LEFT_SWING") for item in states])


def _touchdown_time(time: np.ndarray, log: dict) -> float | None:
    if "touchdown" in log:
        flagged = _first(time, np.asarray(log["touchdown"], dtype=float) > 0.5)
        if flagged is not None:
            return flagged
    states = _text(log["walk_state"]) if "walk_state" in log else []
    seen = False
    for index, name in enumerate(states):
        if name in ("RIGHT_SWING", "LEFT_SWING"):
            seen = True
        if seen and name == "DOUBLE_SUPPORT":
            return float(time[index])
    return None


def _shift_events(time: np.ndarray, states: list[str]):
    return [
        (_first(time, _is(states, "SHIFT_LEFT")), "SHIFT_LEFT start", ACTUAL),
        (_last(time, _is(states, "SHIFT_LEFT")), "SHIFT_LEFT end", THIRD),
        (_first(time, _is(states, "RIGHT_SWING")), "RIGHT_SWING start", SECOND),
    ]


def _state_changes(time: np.ndarray, states: list[str]):
    events = [(float(time[0]), states[0], "0.45")]
    for index in range(1, len(states)):
        if states[index] != states[index - 1]:
            events.append((float(time[index]), states[index], "0.45"))
    return events


def _support_margins(log: dict) -> np.ndarray | None:
    keys = (
        ("barrier_support_x_lower", "support_x_lower"),
        ("barrier_support_x_upper", "support_x_upper"),
        ("barrier_support_y_lower", "support_y_lower"),
        ("barrier_support_y_upper", "support_y_upper"),
    )
    columns = []
    for primary, alternate in keys:
        if primary in log:
            columns.append(np.asarray(log[primary], dtype=float).reshape(-1))
        elif alternate in log:
            columns.append(np.asarray(log[alternate], dtype=float).reshape(-1))
    if len(columns) < 4:
        return None
    return np.column_stack(columns)


def _capture_margins(log: dict) -> np.ndarray | None:
    keys = (
        "capture_margin_x_lower",
        "capture_margin_x_upper",
        "capture_margin_y_lower",
        "capture_margin_y_upper",
    )
    if any(key not in log for key in keys):
        return None
    return np.column_stack([np.asarray(log[key], dtype=float).reshape(-1) for key in keys])


def _swing_channels(log: dict, side: list[str]):
    left = _xy(log["left_foot_xyz"])
    right = _xy(log["right_foot_xyz"])
    left_v = _xy(log["left_foot_vel"])
    right_v = _xy(log["right_foot_vel"])
    x = np.full(len(side), np.nan)
    z = np.full(len(side), np.nan)
    vz = np.full(len(side), np.nan)
    for index, name in enumerate(side):
        if name == "left":
            x[index], z[index], vz[index] = left[index, 0], left[index, 2], left_v[index, 2]
        elif name == "right":
            x[index], z[index], vz[index] = right[index, 0], right[index, 2], right_v[index, 2]
    return x, z, vz


if __name__ == "__main__":
    main()
