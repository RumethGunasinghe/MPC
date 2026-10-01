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
    lines.extend(_landing_lines(landing, stance_side))
    return "\n".join(lines)


def save_support_plot(rows: list[dict], path: Path) -> None:
    """CoM x and capture x against the rear support edge, around the swing."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    swing_rows = [row for row in rows if row.get("walk_state") in _SWING_STATES]
    if swing_rows:
        t0 = float(swing_rows[0]["time"]) - 1.0
        t1 = float(swing_rows[-1]["time"]) + 1.5
    else:
        t0, t1 = 0.0, float(rows[-1]["time"]) if rows else 0.0
    window = [row for row in rows if t0 <= float(row["time"]) <= t1]
    if len(window) < 2:
        return
    time = np.array([float(row["time"]) for row in window])
    com_x = np.array([float(row["com"][0]) for row in window])
    margin = np.array([_margin(row, "support_x_lower") for row in window])
    edge = com_x - margin
    capture = np.array([float(np.asarray(row.get("capture_xy", [np.nan, np.nan]))[0]) for row in window])
    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(8.0, 4.0))
    axis.plot(time, com_x, label="CoM x")
    axis.plot(time, capture, label="capture x")
    axis.plot(time, edge, label="support x lower edge")
    axis.set_xlabel("time (s)")
    axis.set_ylabel("x (m)")
    axis.legend()
    axis.grid(True, alpha=0.3)
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
