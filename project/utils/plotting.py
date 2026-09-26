"""Diagnostic plots for whole-body MPC rollouts and safe RL training runs."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover - keeps the scaffold importable
    plt = None

DEFAULT_FIGURE_DIR = Path(__file__).resolve().parents[1] / "data" / "figures"


def _require_pyplot():
    """Return matplotlib.pyplot or raise a clear import error."""
    if plt is None:
        raise ImportError("matplotlib is required for diagnostic plots")
    return plt


def _as_columns(values: np.ndarray) -> np.ndarray:
    """Return ``values`` as a 2-D array with one signal per column."""
    array = np.asarray(values, dtype=float)
    if array.ndim == 1:
        return array.reshape(-1, 1)
    if array.ndim != 2:
        raise ValueError(f"expected a 1-D or 2-D trace, got shape {array.shape}")
    return array


def plot_tracking(
    time: np.ndarray,
    actual: np.ndarray,
    reference: np.ndarray,
    labels: Sequence[str],
    title: str = "Task-space tracking",
) -> Any:
    """Overlay achieved and reference trajectories (CoM, hand, swing foot)."""
    pyplot = _require_pyplot()
    time = np.asarray(time, dtype=float).reshape(-1)
    achieved = _as_columns(actual)
    target = _as_columns(reference)
    if achieved.shape != target.shape:
        raise ValueError(
            f"actual shape {achieved.shape} does not match reference shape {target.shape}"
        )
    if len(labels) != achieved.shape[1]:
        raise ValueError(
            f"{len(labels)} labels for {achieved.shape[1]} signal(s)"
        )
    fig, axes = pyplot.subplots(achieved.shape[1], 1, sharex=True, figsize=(8, 2.2 * achieved.shape[1]), squeeze=False)
    for index, label in enumerate(labels):
        ax = axes[index, 0]
        ax.plot(time, target[:, index], "k--", lw=1.5, label="reference")
        ax.plot(time, achieved[:, index], lw=1.5, label="actual")
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    axes[-1, 0].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    return fig


def plot_joint_torques(time: np.ndarray, tau: np.ndarray, limit: float | None = None) -> Any:
    """Per-joint torque traces with the actuator saturation band shaded."""
    pyplot = _require_pyplot()
    time = np.asarray(time, dtype=float).reshape(-1)
    torque = _as_columns(tau)
    fig, ax = pyplot.subplots(figsize=(8, 4))
    for joint in range(torque.shape[1]):
        ax.plot(time, torque[:, joint], lw=1.0, label=f"joint {joint}")
    if limit is not None:
        ax.axhline(limit, color="k", ls="--", lw=1.0)
        ax.axhline(-limit, color="k", ls="--", lw=1.0)
        ax.axhspan(-limit, limit, color="0.85", alpha=0.4, zorder=0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("torque [N·m]")
    ax.set_title("Joint torques")
    ax.grid(alpha=0.3)
    if torque.shape[1] <= 8:
        ax.legend(loc="best", fontsize=8, ncol=2)
    fig.tight_layout()
    return fig


def plot_barrier_values(time: np.ndarray, barriers: dict[str, np.ndarray]) -> Any:
    """CBF values over time; the h = 0 line marks the edge of the safe set."""
    pyplot = _require_pyplot()
    time = np.asarray(time, dtype=float).reshape(-1)
    fig, ax = pyplot.subplots(figsize=(8, 4))
    for name, values in barriers.items():
        ax.plot(time, np.asarray(values, dtype=float).reshape(-1), lw=1.4, label=name)
    ax.axhline(0.0, color="k", ls="--", lw=1.0, label="h = 0")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("barrier h(x)")
    ax.set_title("Control barrier values")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    return fig


def plot_cbf_interventions(time: np.ndarray, u_nom: np.ndarray, u_safe: np.ndarray) -> Any:
    """Magnitude of the safety filter's correction ``||u_safe - u_nom||``."""
    pyplot = _require_pyplot()
    time = np.asarray(time, dtype=float).reshape(-1)
    nominal = _as_columns(u_nom)
    safe = _as_columns(u_safe)
    if nominal.shape != safe.shape:
        raise ValueError(f"u_nom shape {nominal.shape} does not match u_safe shape {safe.shape}")
    correction = np.linalg.norm(safe - nominal, axis=1)
    fig, ax = pyplot.subplots(figsize=(8, 3.5))
    ax.plot(time, correction, lw=1.5, color="tab:red")
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"$\|u_{safe} - u_{nom}\|$")
    ax.set_title("CBF intervention")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    return fig


def plot_support_polygon(com_xy: np.ndarray, polygon: np.ndarray) -> Any:
    """Top-down view of the CoM path against the support polygon."""
    pyplot = _require_pyplot()
    path = np.asarray(com_xy, dtype=float).reshape(-1, 2)
    vertices = np.asarray(polygon, dtype=float).reshape(-1, 2)
    closed = np.vstack((vertices, vertices[:1]))
    fig, ax = pyplot.subplots(figsize=(5, 5))
    ax.fill(closed[:, 0], closed[:, 1], color="0.85", label="support polygon")
    ax.plot(closed[:, 0], closed[:, 1], color="k", lw=1.0)
    ax.plot(path[:, 0], path[:, 1], color="tab:blue", lw=1.5, label="CoM")
    ax.scatter(path[0, 0], path[0, 1], c="green", s=30, zorder=3, label="start")
    ax.scatter(path[-1, 0], path[-1, 1], c="red", s=30, zorder=3, label="end")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Centre of mass vs support polygon")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    return fig


def plot_solve_times(solve_times_ms: np.ndarray, control_period_ms: float) -> Any:
    """MPC QP solve-time histogram against the real-time control budget."""
    pyplot = _require_pyplot()
    samples = np.asarray(solve_times_ms, dtype=float).reshape(-1)
    fig, ax = pyplot.subplots(figsize=(7, 4))
    ax.hist(samples, bins=min(40, max(5, samples.size // 2)), color="tab:blue", edgecolor="white")
    ax.axvline(control_period_ms, color="tab:red", ls="--", lw=1.5, label=f"budget {control_period_ms:.1f} ms")
    ax.set_xlabel("solve time [ms]")
    ax.set_ylabel("count")
    ax.set_title("MPC solve times")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    return fig


def plot_training_curves(
    steps: np.ndarray,
    returns: np.ndarray,
    costs: np.ndarray,
    cost_budget: float,
) -> Any:
    """Safe RL return and safety cost, with the budget drawn as a threshold."""
    pyplot = _require_pyplot()
    steps = np.asarray(steps, dtype=float).reshape(-1)
    returns = np.asarray(returns, dtype=float).reshape(-1)
    costs = np.asarray(costs, dtype=float).reshape(-1)
    fig, axes = pyplot.subplots(2, 1, sharex=True, figsize=(8, 5))
    axes[0].plot(steps, returns, color="tab:blue", lw=1.5)
    axes[0].set_ylabel("return")
    axes[0].set_title("Safe RL training")
    axes[0].grid(alpha=0.3)
    axes[1].plot(steps, costs, color="tab:orange", lw=1.5, label="episode cost")
    axes[1].axhline(cost_budget, color="k", ls="--", lw=1.0, label=f"budget {cost_budget:g}")
    axes[1].set_xlabel("environment steps")
    axes[1].set_ylabel("safety cost")
    axes[1].grid(alpha=0.3)
    axes[1].legend(loc="best", fontsize=8)
    fig.tight_layout()
    return fig


def save_figure(fig: Any, name: str, directory: Path = DEFAULT_FIGURE_DIR) -> Path:
    """Save a figure as PNG under ``data/figures/``."""
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{name}.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    return path
