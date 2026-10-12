"""Summarise one single-support swing from a ``walk_tick`` log.

The numbers come from the logged rows. Nothing here is filled in by hand.
A swing that never starts is reported as such.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

_SWING_STATES = ("RIGHT_SWING", "LEFT_SWING")
_JOINTS = ("hip_roll", "hip_pitch", "knee", "ankle")


def summarize_single_support(rows: list[dict]) -> str:
    """Text report for the first swing and the landing window after it."""
    swing_rows = [row for row in rows if row.get("walk_state") in _SWING_STATES]
    lines = ["=== SINGLE SUPPORT DIAGNOSTIC ===", ""]
    if not swing_rows:
        lines.append("No swing occurred.")
        return "\n".join(lines)

    swing_side = str(swing_rows[0].get("swing_side") or "")
    stance_side = "left" if swing_side == "right" else "right" if swing_side == "left" else ""
    t0 = float(swing_rows[0]["time"])
    t1 = float(swing_rows[-1]["time"])
    # Landing is where the last runs actually fell: the second after the
    # swing state ends, not only the ticks labelled as a swing.
    landing = [row for row in rows if t0 <= float(row["time"]) <= t1 + 1.5]

    lines.append(f"Swing: {swing_side or 'unknown'}")
    lines.append(f"Stance: {stance_side or 'unknown'}")
    lines.append(f"Swing duration: {t1 - t0:.2f} s ({t0:.2f} s to {t1:.2f} s)")
    lines.append("")
    lines.append(_event_line("First support_x_lower <= 0", landing, lambda row: _margin(row, "support_x_lower") <= 0.0))
    lines.append(_event_line("First support_x_lower < 0.02 m", landing, lambda row: _margin(row, "support_x_lower") < 0.02))
    lines.append(_event_line("First CBF not optimal", landing, lambda row: str(row.get("cbf_status", "optimal")) != "optimal"))
    lines.append(_event_line("First backward CoM velocity < -0.05 m/s", landing, lambda row: float(row["com_velocity"][0]) < -0.05))
    lines.append("")

    vx = np.array([float(row["com_velocity"][0]) for row in swing_rows])
    ax = np.array([float(row.get("com_acceleration", [0.0, 0.0, 0.0])[0]) for row in swing_rows])
    margins = np.array([_margin(row, "support_x_lower") for row in swing_rows])
    capture_back = np.array([float(row.get("capture_margin_x_lower", np.nan)) for row in swing_rows])
    lines.append(f"Minimum support_x_lower during swing: {_fmt(float(np.min(margins)))} m")
    lines.append(f"Maximum backward CoM velocity during swing: {_fmt(float(np.min(vx)))} m/s")
    lines.append(f"Maximum backward CoM acceleration during swing: {_fmt(float(np.min(ax)))} m/s^2")
    lines.append(f"Minimum capture-point rear margin during swing: {_fmt(float(np.nanmin(capture_back)))} m")
    if stance_side:
        correction = np.array([float(row.get("stance_ankle_correction", 0.0)) for row in swing_rows])
        knee = np.array([float(row.get(f"{stance_side}_knee", 0.0)) for row in swing_rows])
        lines.append(f"Maximum |stance ankle correction|: {_fmt(float(np.max(np.abs(correction))))} rad")
        lines.append(f"Maximum stance knee movement from swing start: {_fmt(float(np.max(np.abs(knee - knee[0]))))} rad")
    if swing_side:
        torque = _swing_torque(swing_rows, swing_side)
        knee_acc = _knee_acceleration(swing_rows, swing_side)
        lines.append(f"Maximum |swing-leg torque|: {_fmt(float(np.max(torque)))} N.m")
        lines.append(f"Maximum |swing knee acceleration|: {_fmt(float(np.max(np.abs(knee_acc))))} rad/s^2")
    lines.append("")
    lines.extend(_geometry_lines(swing_rows))
    lines.append("")
    lines.extend(_late_swing_lines(swing_rows, stance_side))
    lines.append("")
    lines.extend(_landing_lines(landing, stance_side))
    lines.append("")
    lines.extend(_handoff_lines(rows, swing_rows, stance_side))
    return "\n".join(lines)


def save_support_plot(rows: list[dict], path: Path) -> None:
    """Landing-window plot: support edges, capture, lean, and the CBF."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    swing_rows = [row for row in rows if row.get("walk_state") in _SWING_STATES]
    if swing_rows:
        t0 = float(swing_rows[0]["time"]) - 0.4
        t1 = float(swing_rows[-1]["time"]) + 1.2
        stance = "left" if str(swing_rows[0].get("swing_side")) == "right" else "right"
    else:
        t0, t1 = 0.0, float(rows[-1]["time"]) if rows else 0.0
        stance = "left"
    window = [row for row in rows if t0 <= float(row["time"]) <= t1]
    if len(window) < 2:
        return
    time = np.array([float(row["time"]) for row in window])
    com = np.vstack([np.asarray(row["com"], dtype=float).reshape(3) for row in window])
    vel = np.vstack([np.asarray(row["com_velocity"], dtype=float).reshape(3) for row in window])
    ref = np.vstack([np.asarray(row["com_reference"], dtype=float).reshape(-1)[:2] for row in window])
    capture = np.vstack(
        [np.asarray(row.get("capture_xy", [np.nan, np.nan]), dtype=float).reshape(2) for row in window]
    )
    rear = com[:, 0] - np.array([_margin(row, "support_x_lower") for row in window])
    outer_name = "support_y_upper" if stance == "left" else "support_y_lower"
    outer_sign = 1.0 if stance == "left" else -1.0
    outer_margin = np.array([_margin(row, outer_name) for row in window])
    outer_edge = com[:, 1] + outer_sign * outer_margin
    hip = np.array([float(row.get(f"{stance}_hip_roll_des", np.nan)) for row in window])
    ankle = np.array([float(row.get("stance_ankle_correction", np.nan)) for row in window])
    cap_rear = np.array([float(row.get("capture_margin_x_lower", np.nan)) for row in window])
    cap_outer = np.array([float(row.get(f"capture_margin_{outer_name.removeprefix('support_')}", np.nan)) for row in window])
    intervention = np.array([float(row.get("intervention", np.nan)) for row in window])

    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(5, 1, figsize=(9.0, 12.0), sharex=True)
    axes[0].plot(time, com[:, 0], label="CoM x")
    axes[0].plot(time, capture[:, 0], label="capture x")
    axes[0].plot(time, rear, label="rear support edge")
    axes[0].plot(time, ref[:, 0], label="reference x")
    axes[0].set_ylabel("x (m)")
    axes[1].plot(time, com[:, 1], label="CoM y")
    axes[1].plot(time, capture[:, 1], label="capture y")
    axes[1].plot(time, outer_edge, label="outer support edge")
    axes[1].plot(time, ref[:, 1], label="reference y")
    axes[1].set_ylabel("y (m)")
    axes[2].plot(time, vel[:, 0], label="vx")
    axes[2].plot(time, vel[:, 1], label="vy")
    axes[2].set_ylabel("m/s")
    axes[3].plot(time, hip, label=f"{stance} hip-roll target")
    axes[3].plot(time, ankle, label="stance ankle correction")
    axes[3].set_ylabel("rad")
    axes[4].plot(time, cap_rear, label="capture margin, rear")
    axes[4].plot(time, cap_outer, label="capture margin, outer")
    axes[4].plot(time, intervention / 100.0, label="CBF intervention / 100")
    axes[4].set_ylabel("m, N.m/100")
    axes[4].set_xlabel("time (s)")
    marks = _event_marks(rows, swing_rows)
    for axis in axes:
        for label, instant in marks:
            axis.axvline(instant, color="0.4", lw=0.8, ls="--")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best", fontsize=8)
    if marks:
        axes[0].set_title("  ".join(f"{label} {instant:.2f}s" for label, instant in marks), fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)


def _margin(row: dict, name: str) -> float:
    barriers = row.get("barriers") or {}
    if name in barriers:
        return float(barriers[name])
    return float(row.get(name, np.nan))


def _event_line(label: str, rows: list[dict], test) -> str:
    for row in rows:
        try:
            hit = bool(test(row))
        except (KeyError, TypeError, ValueError):
            continue
        if hit:
            return f"{label}: t = {float(row['time']):.2f} s ({row.get('walk_state', '')})"
    return f"{label}: none"


def _fmt(value: float) -> str:
    if not np.isfinite(value):
        return "n/a"
    return f"{value:.4f}"


def _swing_torque(rows: list[dict], side: str) -> np.ndarray:
    peaks = []
    for row in rows:
        values = [abs(float(row.get(f"{side}_{joint}_torque", 0.0))) for joint in _JOINTS]
        peaks.append(max(values) if values else 0.0)
    return np.asarray(peaks, dtype=float)


def _knee_acceleration(rows: list[dict], side: str) -> np.ndarray:
    velocity = np.array([float(row.get(f"{side}_knee_vel", 0.0)) for row in rows])
    time = np.array([float(row["time"]) for row in rows])
    if velocity.size < 2:
        return np.zeros(1)
    dt = np.diff(time)
    dt = np.where(dt > 1e-6, dt, np.nan)
    accel = np.diff(velocity) / dt
    return np.nan_to_num(accel, nan=0.0)


def _late_swing_lines(rows: list[dict], stance_side: str) -> list[str]:
    """Capture margin over the last 0.3 s of the swing, before touchdown."""
    if not rows:
        return []
    t1 = float(rows[-1]["time"])
    late = [row for row in rows if float(row["time"]) >= t1 - 0.3]
    outer = "y_upper" if stance_side == "left" else "y_lower"
    margin = np.array([float(row.get(f"capture_margin_{outer}", np.nan)) for row in late])
    com = np.asarray(late[-1]["com"], dtype=float).reshape(3)
    vel = np.asarray(late[-1]["com_velocity"], dtype=float).reshape(3)
    capture = np.asarray(late[-1].get("capture_xy", [np.nan, np.nan]), dtype=float).reshape(2)
    return [
        f"Minimum outer capture margin in the last 0.3 s of swing: {_fmt(float(np.nanmin(margin)))} m",
        f"At swing end CoM ({com[0]:+.4f}, {com[1]:+.4f}) m, velocity ({vel[0]:+.4f}, {vel[1]:+.4f}) m/s",
        f"At swing end capture ({capture[0]:+.4f}, {capture[1]:+.4f}) m",
    ]


def _handoff_lines(rows: list[dict], swing_rows: list[dict], stance_side: str) -> list[str]:
    """State on either side of the swing-to-double-support tick."""
    if not swing_rows:
        return []
    touch_index = rows.index(swing_rows[-1]) + 1
    if touch_index >= len(rows):
        return ["Touchdown: swing did not end inside the log."]
    touch = float(rows[touch_index]["time"])
    outer = "y_upper" if stance_side == "left" else "y_lower"
    lines = [
        f"Touchdown (first tick after the swing): t = {touch:.2f} s",
        "time | state | posture | com xy | v xy | capture xy | ref xy | dref | "
        f"{stance_side} hip-roll des | ankle corr | fraction | cbf | iv | "
        "xl | outer y | cap rear | cap outer | pitch",
    ]
    before = np.asarray(swing_rows[-1]["com_reference"], dtype=float).reshape(-1)
    for offset in (-0.2, 0.0, 0.02, 0.1, 0.2, 0.4):
        row = _row_at(rows, touch + offset)
        if row is None:
            lines.append(f"{touch + offset:6.2f} | (no sample)")
            continue
        com = np.asarray(row["com"], dtype=float).reshape(3)
        vel = np.asarray(row["com_velocity"], dtype=float).reshape(3)
        cap = np.asarray(row.get("capture_xy", [np.nan, np.nan]), dtype=float).reshape(2)
        ref = np.asarray(row["com_reference"], dtype=float).reshape(-1)
        jump = float(np.linalg.norm(ref[:2] - before[:2]))
        before = ref
        lines.append(
            f"{float(row['time']):6.2f} | {str(row.get('walk_state', '')):16} | "
            f"{'on' if row.get('posture_active') else 'off':3} | "
            f"({com[0]:+.3f},{com[1]:+.3f}) | ({vel[0]:+.3f},{vel[1]:+.3f}) | "
            f"({cap[0]:+.3f},{cap[1]:+.3f}) | ({ref[0]:+.3f},{ref[1]:+.3f}) | {jump:.3f} | "
            f"{float(row.get(f'{stance_side}_hip_roll_des', np.nan)):+.3f} | "
            f"{float(row.get('stance_ankle_correction', np.nan)):+.3f} | "
            f"{float(row.get('shift_fraction', np.nan)):.3f} | "
            f"{row.get('cbf_status', '')} | {float(row.get('intervention', np.nan)):6.1f} | "
            f"{_margin(row, 'support_x_lower'):+.3f} | {_margin(row, f'support_{outer}'):+.3f} | "
            f"{float(row.get('capture_margin_x_lower', np.nan)):+.3f} | "
            f"{float(row.get(f'capture_margin_{outer}', np.nan)):+.3f} | "
            f"{float(row.get('pelvis_pitch', np.nan)):+.3f}"
        )
    swing_side = "right" if stance_side == "left" else "left"
    lines.append("Dense window every 0.04 s:")
    for offset in np.arange(-0.24, 0.40, 0.04):
        row = _row_at(rows, touch + float(offset))
        if row is None:
            continue
        vel = np.asarray(row["com_velocity"], dtype=float).reshape(3)
        com = np.asarray(row["com"], dtype=float).reshape(3)
        lines.append(
            f"{float(row['time']):6.2f} {str(row.get('walk_state', '')):16} "
            f"v ({float(vel[0]):+.3f},{float(vel[1]):+.3f}) y {float(com[1]):+.3f} "
            f"capOut {float(row.get(f'capture_margin_{outer}', np.nan)):+.3f} "
            f"yOut {_margin(row, f'support_{outer}'):+.3f} "
            f"hip {float(row.get(f'{stance_side}_hip_roll_des', np.nan)):+.3f}/"
            f"{float(row.get(f'{swing_side}_hip_roll_des', np.nan)):+.3f} "
            f"ank {float(row.get(f'{stance_side}_ankle_des', np.nan)):+.3f}/"
            f"{float(row.get(f'{swing_side}_ankle_des', np.nan)):+.3f} "
            f"knee {float(row.get(f'{swing_side}_knee_des', np.nan)):+.3f} "
            f"frac {float(row.get('shift_fraction', np.nan)):.3f} {row.get('shift_limit', '')}"
        )
    return lines


def _row_at(rows: list[dict], time_s: float) -> dict | None:
    if not rows:
        return None
    return min(rows, key=lambda row: abs(float(row["time"]) - time_s))


def _event_marks(rows: list[dict], swing_rows: list[dict]) -> list[tuple[str, float]]:
    marks: list[tuple[str, float]] = []
    if swing_rows:
        marks.append(("swing", float(swing_rows[0]["time"])))
        touch_index = rows.index(swing_rows[-1]) + 1
        if touch_index < len(rows):
            marks.append(("touchdown", float(rows[touch_index]["time"])))
    for label, test in (
        ("cbf", lambda row: str(row.get("cbf_status", "optimal")) != "optimal"),
        ("x rear", lambda row: _margin(row, "support_x_lower") <= 0.0),
    ):
        for row in rows:
            try:
                hit = bool(test(row))
            except (KeyError, TypeError, ValueError):
                continue
            if hit and swing_rows and float(row["time"]) >= float(swing_rows[0]["time"]):
                marks.append((label, float(row["time"])))
                break
    return marks


def _landing_lines(rows: list[dict], stance_side: str) -> list[str]:
    """Lateral motion after the swing, from the same logged rows."""
    if len(rows) < 2:
        return []
    after = [row for row in rows if row.get("walk_state") not in _SWING_STATES]
    if not after:
        return []
    sign = 1.0 if stance_side == "left" else -1.0
    outer = "support_y_upper" if stance_side == "left" else "support_y_lower"
    vy = np.array([float(row["com_velocity"][1]) for row in after])
    margin = np.array([_margin(row, outer) for row in after])
    fraction = np.array([float(row.get("shift_fraction", np.nan)) for row in after])
    return [
        f"Landing window: {float(after[0]['time']):.2f} s to {float(after[-1]['time']):.2f} s",
        f"Maximum outward CoM velocity after swing: {_fmt(float(np.max(vy * sign)))} m/s",
        f"Minimum outer support margin after swing: {_fmt(float(np.nanmin(margin)))} m",
        f"Shift fraction at landing: {_fmt(float(fraction[0]))}",
        f"Shift fraction 0.4 s later: {_fmt(float(fraction[min(20, fraction.size - 1)]))}",
    ]


def _geometry_lines(rows: list[dict]) -> list[str]:
    com0 = float(rows[0]["com"][0])
    com1 = float(rows[-1]["com"][0])
    edge0 = com0 - _margin(rows[0], "support_x_lower")
    edge1 = com1 - _margin(rows[-1], "support_x_lower")
    return [
        f"CoM x change during swing: {com1 - com0:+.4f} m",
        f"Rear support edge change during swing: {edge1 - edge0:+.4f} m",
    ]
