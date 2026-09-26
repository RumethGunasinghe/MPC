"""Record one push-recovery run and write the demonstration figures.

Runs the same MPC, PD and CBF stack as ``main_demo.py``, without opening the
viewer, from t = 0 through a few seconds after the shove. Figures land in
``data/figures/``.

Usage:
    python visualize_demo.py
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from main_demo import (
    PUSH_DURATION,
    PUSH_FORCE,
    PUSH_TIME,
    apply_push,
    build_stack,
    control_step,
    sole_center_xy,
    stand_reference,
)

ROOT = Path(__file__).resolve().parent
FIGURE_DIR = ROOT / "data" / "figures"
LOG_PATH = ROOT / "data" / "demo_analysis.npz"
DURATION = 6.5  # seconds of simulated time


def _rpy(rotation: np.ndarray) -> np.ndarray:
    """Roll, pitch, yaw in radians from a body-to-world rotation matrix."""
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    pitch = np.arctan2(-rotation[2, 0], np.hypot(rotation[2, 1], rotation[2, 2]))
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def record() -> dict[str, np.ndarray]:
    """Simulate the stand and the shove, returning column-stacked traces."""
    import argparse

    args = argparse.Namespace(
        model_path=ROOT / "models" / "unitree_h1" / "scene.xml",
        control_dt=0.02,
        horizon=20,
    )
    env, mpc, cbf, pd = build_stack(args)
    env.reset(seed=0)
    state0 = env.get_state()
    reference = stand_reference(env, state0)
    sole = sole_center_xy(env)
    names = list(env.joint_names)
    pushed = False
    rows: list[dict] = []
    try:
        while float(env.data.time) < DURATION:
            if not pushed and float(env.data.time) >= PUSH_TIME:
                apply_push(env, PUSH_FORCE)
                pushed = True
            row = control_step(env, mpc, cbf, pd, reference)
            rows.append(row)
            if row["com"][2] < 0.45:
                break
    finally:
        env.close()

    time = np.array([row["time"] for row in rows])
    com = np.vstack([row["com"] for row in rows])
    com_velocity = np.vstack([row["com_velocity"] for row in rows])
    tau_nominal = np.vstack([row["tau_nominal"] for row in rows])
    tau_safe = np.vstack([row["tau_safe"] for row in rows])
    q_des = np.vstack([row["q_des"] for row in rows])
    joint_qpos = np.vstack([row["joint_qpos"] for row in rows])
    joint_qvel = np.vstack([row["joint_qvel"] for row in rows])
    rpy = np.vstack([_rpy(row["base_rotation"]) for row in rows])
    barrier_names = list(rows[0]["barriers"].keys())
    barriers = np.column_stack(
        [[row["barriers"][name] for row in rows] for name in barrier_names]
    )
    intervention = np.array([row["intervention"] for row in rows])
    status = np.array([row["cbf_status"] for row in rows])
    reference_com = np.asarray(reference["com"], dtype=float)
    half = np.asarray(cbf.config.support_half_extents, dtype=float) - cbf.config.margin
    center = np.asarray(cbf.config.support_center, dtype=float)
    push = ((time >= PUSH_TIME) & (time < PUSH_TIME + PUSH_DURATION)).astype(float)

    payload = {
        "time": time,
        "com": com,
        "com_velocity": com_velocity,
        "reference_com": reference_com,
        "sole_center": sole,
        "tau_nominal": tau_nominal,
        "tau_safe": tau_safe,
        "q_des": q_des,
        "joint_qpos": joint_qpos,
        "joint_qvel": joint_qvel,
        "joint_names": np.array(names),
        "rpy": rpy,
        "barrier_names": np.array(barrier_names),
        "barriers": barriers,
        "intervention": intervention,
        "cbf_status": status,
        "support_center": center,
        "support_half": half,
        "push_active": push,
        "push_force": np.asarray(PUSH_FORCE, dtype=float),
        "push_time": np.array([PUSH_TIME, PUSH_TIME + PUSH_DURATION]),
        "home_qpos": state0["joint_qpos"].copy(),
    }
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    np.savez(LOG_PATH, **payload)
    return payload


def _shade_push(ax, push_time: np.ndarray) -> None:
    ax.axvspan(push_time[0], push_time[1], color="0.85", zorder=0, label="80 N push")


def _mark_push(ax, push_time: np.ndarray) -> None:
    ax.axvline(push_time[0], color="0.4", ls="--", lw=1.0)


def render(data: dict[str, np.ndarray]) -> list[Path]:
    """Write every demonstration figure and a small JSON summary."""
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    time = data["time"]
    com = data["com"]
    vel = data["com_velocity"]
    ref = data["reference_com"]
    push_time = data["push_time"]
    names = [str(name) for name in data["joint_names"]]
    paths: list[Path] = []

    def save(fig: plt.Figure, name: str) -> None:
        path = FIGURE_DIR / f"{name}.png"
        fig.savefig(path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        paths.append(path)

    # 1. CoM position vs the sole-centre reference.
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
    labels = ("x (m)", "y (m)", "z (m)")
    for index, label in enumerate(labels):
        _shade_push(axes[index], push_time)
        axes[index].plot(time, np.full(time.shape, ref[index]), "k--", lw=1.2, label="reference")
        axes[index].plot(time, com[:, index], lw=1.6, label="actual")
        axes[index].set_ylabel(label)
        axes[index].grid(alpha=0.3)
        axes[index].legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Centre-of-mass position")
    fig.tight_layout()
    save(fig, "com_position")

    # 2. CoM velocity.
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
    for index, label in enumerate(("vx (m/s)", "vy (m/s)", "vz (m/s)")):
        _shade_push(axes[index], push_time)
        axes[index].plot(time, vel[:, index], lw=1.5)
        axes[index].set_ylabel(label)
        axes[index].grid(alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Centre-of-mass velocity")
    fig.tight_layout()
    save(fig, "com_velocity")

    # 3. Speed.
    speed = np.linalg.norm(vel, axis=1)
    fig, ax = plt.subplots(figsize=(8, 3.2))
    _shade_push(ax, push_time)
    ax.plot(time, speed, color="tab:purple", lw=1.6, label="speed")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("speed (m/s)")
    ax.set_title("Centre-of-mass speed")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    save(fig, "com_speed")

    # 4. Horizontal path against the support box.
    center = data["support_center"]
    half = data["support_half"]
    box = np.array(
        [
            [center[0] - half[0], center[1] - half[1]],
            [center[0] + half[0], center[1] - half[1]],
            [center[0] + half[0], center[1] + half[1]],
            [center[0] - half[0], center[1] + half[1]],
            [center[0] - half[0], center[1] - half[1]],
        ]
    )
    push_mask = data["push_active"].astype(bool)
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    ax.fill(box[:, 0], box[:, 1], color="0.9", label="support box")
    ax.plot(box[:, 0], box[:, 1], color="k", lw=1.0)
    ax.plot(com[:, 0], com[:, 1], color="tab:blue", lw=1.6, label="CoM path")
    if np.any(push_mask):
        ax.plot(com[push_mask, 0], com[push_mask, 1], color="tab:red", lw=2.4, label="during push")
    ax.scatter(com[0, 0], com[0, 1], c="green", s=36, zorder=3, label="start")
    ax.scatter(com[-1, 0], com[-1, 1], c="black", s=36, zorder=3, label="end")
    ax.scatter(ref[0], ref[1], c="tab:orange", marker="x", s=60, zorder=3, label="reference")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Horizontal centre of mass and support box")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    save(fig, "com_support_path")

    # 5. Phase portrait.
    fig, ax = plt.subplots(figsize=(5.5, 4.5))
    ax.plot(com[:, 0], vel[:, 0], color="tab:blue", lw=1.4)
    ax.scatter(com[0, 0], vel[0, 0], c="green", s=36, zorder=3, label="start")
    ax.scatter(ref[0], 0.0, c="tab:orange", marker="x", s=60, zorder=3, label="reference")
    if np.any(push_mask):
        ax.scatter(com[push_mask, 0], vel[push_mask, 0], c="tab:red", s=16, zorder=3, label="during push")
    ax.set_xlabel("CoM x (m)")
    ax.set_ylabel("CoM vx (m/s)")
    ax.set_title("Sagittal phase portrait")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    save(fig, "phase_portrait")

    # 6. Base attitude.
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
    for index, label in enumerate(("roll (deg)", "pitch (deg)", "yaw (deg)")):
        _shade_push(axes[index], push_time)
        axes[index].plot(time, np.degrees(data["rpy"][:, index]), lw=1.5)
        axes[index].set_ylabel(label)
        axes[index].grid(alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Floating-base attitude")
    fig.tight_layout()
    save(fig, "base_attitude")

    # 7. Tracking error.
    error = com - ref.reshape(1, 3)
    fig, ax = plt.subplots(figsize=(8, 3.4))
    _shade_push(ax, push_time)
    ax.plot(time, error[:, 0], label="x error")
    ax.plot(time, error[:, 1], label="y error")
    ax.plot(time, error[:, 2], label="z error")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (m)")
    ax.set_title("Centre-of-mass tracking error")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    save(fig, "com_tracking_error")

    # 8. Barrier values.
    fig, ax = plt.subplots(figsize=(8, 3.8))
    _shade_push(ax, push_time)
    for index, name in enumerate(data["barrier_names"]):
        ax.plot(time, data["barriers"][:, index], lw=1.4, label=str(name))
    ax.axhline(0.0, color="k", ls="--", lw=1.0, label="h = 0")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("barrier h (m)")
    ax.set_title("Support-box barrier values")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    save(fig, "barrier_values")

    # 9. Minimum margin.
    margin = data["barriers"].min(axis=1)
    fig, ax = plt.subplots(figsize=(8, 3.2))
    _shade_push(ax, push_time)
    ax.plot(time, margin, color="tab:green", lw=1.6)
    ax.axhline(0.0, color="k", ls="--", lw=1.0)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("min h (m)")
    ax.set_title("Distance to the nearest face of the support box")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    save(fig, "support_margin")

    # 10. Intervention.
    fig, ax = plt.subplots(figsize=(8, 3.4))
    _shade_push(ax, push_time)
    ax.plot(time, data["intervention"], color="tab:red", lw=1.6)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("intervention (N·m)")
    ax.set_title(r"Safety-filter correction  $\|u_{safe}-u_{nom}\|$")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    save(fig, "cbf_intervention")

    # 11. Status as a step series.
    status_code = np.array(
        [{"optimal": 0, "relaxed": 1, "fail_safe": 2}.get(str(item), -1) for item in data["cbf_status"]]
    )
    fig, ax = plt.subplots(figsize=(8, 2.6))
    _shade_push(ax, push_time)
    ax.step(time, status_code, where="post", color="tab:brown", lw=1.4)
    ax.set_yticks([0, 1, 2], ["optimal", "relaxed", "fail_safe"])
    ax.set_xlabel("time (s)")
    ax.set_ylabel("QP status")
    ax.set_title("Safety-filter solve status")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    save(fig, "cbf_status")

    # 12. Torque norms.
    fig, ax = plt.subplots(figsize=(8, 3.4))
    _shade_push(ax, push_time)
    ax.plot(time, np.linalg.norm(data["tau_nominal"], axis=1), lw=1.4, label="nominal")
    ax.plot(time, np.linalg.norm(data["tau_safe"], axis=1), lw=1.4, label="safe")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("torque norm (N·m)")
    ax.set_title("Nominal and filtered torque magnitude")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    save(fig, "torque_norm")

    # 13. Per-joint correction at the peak intervention.
    delta = data["tau_safe"] - data["tau_nominal"]
    peak = int(np.argmax(data["intervention"]))
    order = np.argsort(-np.abs(delta[peak]))
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.barh([names[i] for i in order], delta[peak, order], color="tab:red")
    ax.axvline(0.0, color="k", lw=0.8)
    ax.set_xlabel("safe − nominal torque (N·m)")
    ax.set_title(f"Filter correction by joint at t = {time[peak]:.2f} s")
    ax.grid(alpha=0.3, axis="x")
    fig.tight_layout()
    save(fig, "torque_correction_by_joint")

    # 14. Torque heatmap of the correction.
    fig, ax = plt.subplots(figsize=(8, 5))
    image = ax.imshow(
        delta.T,
        aspect="auto",
        origin="lower",
        extent=[time[0], time[-1], -0.5, len(names) - 0.5],
        cmap="coolwarm",
    )
    ax.set_yticks(range(len(names)), names, fontsize=7)
    ax.set_xlabel("time (s)")
    ax.set_title("Safe minus nominal torque (N·m)")
    fig.colorbar(image, ax=ax, label="N·m")
    fig.tight_layout()
    save(fig, "torque_correction_heatmap")

    # 15. Leg joint tracking.
    leg = [index for index, name in enumerate(names) if any(token in name for token in ("hip", "knee", "ankle"))]
    fig, axes = plt.subplots(len(leg), 1, sharex=True, figsize=(8, 1.35 * len(leg)))
    for axis, index in zip(axes, leg):
        _shade_push(axis, push_time)
        axis.plot(time, data["q_des"][:, index], "k--", lw=1.0, label="target")
        axis.plot(time, data["joint_qpos"][:, index], lw=1.2, label="actual")
        axis.set_ylabel(names[index], fontsize=7)
        axis.grid(alpha=0.3)
    axes[0].legend(loc="best", fontsize=7)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Leg joint tracking")
    fig.tight_layout()
    save(fig, "leg_joint_tracking")

    # 16. Ankles and hip pitch, the balance joints.
    focus = [index for index, name in enumerate(names) if "ankle" in name or "hip_pitch" in name]
    fig, axes = plt.subplots(len(focus), 1, sharex=True, figsize=(8, 1.5 * len(focus)))
    for axis, index in zip(axes, focus):
        _shade_push(axis, push_time)
        axis.plot(time, np.degrees(data["q_des"][:, index]), "k--", lw=1.0, label="target")
        axis.plot(time, np.degrees(data["joint_qpos"][:, index]), lw=1.3, label="actual")
        axis.set_ylabel(names[index] + " (deg)", fontsize=7)
        axis.grid(alpha=0.3)
    axes[0].legend(loc="best", fontsize=7)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Ankle and hip-pitch angles")
    fig.tight_layout()
    save(fig, "balance_joints")

    # 17. Joint velocity norm.
    fig, ax = plt.subplots(figsize=(8, 3.2))
    _shade_push(ax, push_time)
    ax.plot(time, np.linalg.norm(data["joint_qvel"], axis=1), color="tab:cyan", lw=1.5)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("joint speed (rad/s)")
    ax.set_title("Actuated joint-velocity norm")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    save(fig, "joint_velocity_norm")

    # 18. Intervention histogram.
    fig, ax = plt.subplots(figsize=(7, 3.6))
    active = data["intervention"][data["intervention"] > 1.0]
    bins = np.linspace(0.0, max(float(data["intervention"].max()), 1.0), 20)
    ax.hist(data["intervention"], bins=bins, color="0.75", edgecolor="white", label="all steps")
    if active.size:
        ax.hist(active, bins=bins, color="tab:red", edgecolor="white", label="above 1 N·m")
    ax.set_xlabel("intervention (N·m)")
    ax.set_ylabel("control steps")
    ax.set_title("Distribution of safety-filter interventions")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    save(fig, "intervention_histogram")

    # 19. Window comparison.
    def window(mask: np.ndarray) -> dict[str, float]:
        if not np.any(mask):
            return {"peak_speed": 0.0, "peak_pitch": 0.0, "peak_intervention": 0.0, "mean_height": 0.0}
        return {
            "peak_speed": float(speed[mask].max()),
            "peak_pitch": float(np.degrees(np.abs(data["rpy"][mask, 1])).max()),
            "peak_intervention": float(data["intervention"][mask].max()),
            "mean_height": float(com[mask, 2].mean()),
        }

    before = (time >= 1.0) & (time < PUSH_TIME)
    during = push_mask
    after = (time >= PUSH_TIME + PUSH_DURATION) & (time < PUSH_TIME + PUSH_DURATION + 2.0)
    windows = {"before": window(before), "during push": window(during), "after": window(after)}
    metrics = ["peak_speed", "peak_pitch", "peak_intervention", "mean_height"]
    metric_labels = ["peak speed (m/s)", "peak |pitch| (deg)", "peak intervention (N·m)", "mean height (m)"]
    fig, axes = plt.subplots(1, 4, figsize=(10, 3.4))
    colors = ["0.65", "tab:red", "tab:blue"]
    for axis, metric, label in zip(axes, metrics, metric_labels):
        values = [windows[key][metric] for key in windows]
        axis.bar(list(windows), values, color=colors)
        axis.set_ylabel(label, fontsize=8)
        axis.tick_params(axis="x", labelrotation=20)
        axis.grid(alpha=0.3, axis="y")
    fig.suptitle("Before, during, and after the shove")
    fig.tight_layout()
    save(fig, "window_comparison")

    # 20. Push force timeline.
    fig, ax = plt.subplots(figsize=(8, 2.8))
    ax.step(time, data["push_active"] * float(data["push_force"][0]), where="post", color="tab:red", lw=1.6)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("force (N)")
    ax.set_title("External force on the torso, world +x")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    save(fig, "push_force")

    settled = (time >= 1.5) & (time < PUSH_TIME)
    summary = {
        "duration_s": float(time[-1]),
        "steps": int(time.size),
        "push_n": float(data["push_force"][0]),
        "push_window_s": [float(push_time[0]), float(push_time[1])],
        "reference_com_m": [float(v) for v in ref],
        "settled_com_m": [float(v) for v in com[settled].mean(axis=0)] if np.any(settled) else [],
        "peak_pitch_deg": float(np.degrees(np.abs(data["rpy"][:, 1])).max()),
        "peak_speed_m_s": float(speed.max()),
        "peak_forward_com_m": float(com[:, 0].max()),
        "peak_intervention_nm": float(data["intervention"].max()),
        "min_barrier_m": float(margin.min()),
        "final_com_m": [float(v) for v in com[-1]],
        "final_pitch_deg": float(np.degrees(data["rpy"][-1, 1])),
        "intervention_steps": int(np.sum(data["intervention"] > 1.0)),
        "relaxed_steps": int(np.sum(data["cbf_status"] == "relaxed")),
        "windows": windows,
    }
    (FIGURE_DIR / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return paths


def main() -> None:
    data = record()
    paths = render(data)
    print(f"wrote {len(paths)} figures to {FIGURE_DIR}")
    for path in paths:
        print(path.name)


if __name__ == "__main__":
    main()
