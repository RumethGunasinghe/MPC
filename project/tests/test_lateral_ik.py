"""Swing-leg lateral IK on the Unitree H1 model.

The pitch solution is the existing one. These checks add a bounded world-y
ankle move through hip roll, and they confirm an unreachable request returns
the current-roll solution instead of an invalid command.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco

from controllers.leg_ik import LegIK

MODEL = ROOT / "models" / "unitree_h1" / "scene.xml"
OFFSETS_M = (0.0, 0.01, 0.02, 0.03, 0.04, -0.01, -0.02, -0.03, -0.04)


def _home():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


def _ankle(model, data, side: str) -> np.ndarray:
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_link")
    return np.asarray(data.xpos[body], dtype=float).copy()


def _limits(ik: LegIK, side: str) -> dict[str, tuple[float, float]]:
    leg = ik._legs[side]
    return {
        "hip_roll": leg.hip_roll_range,
        "hip_pitch": leg.hip_range,
        "knee": leg.knee_range,
        "ankle": leg.ankle_range,
    }


def _inside(value: float, limits: tuple[float, float]) -> bool:
    return limits[0] - 1e-6 <= value <= limits[1] + 1e-6


def check_no_ankle_roll(model) -> None:
    for side in ("left", "right"):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_ankle_roll")
        assert joint < 0


def check_sagittal_unchanged(model, data, ik: LegIK) -> None:
    for side in ("left", "right"):
        ankle = _ankle(model, data, side)
        joints = ik.solve(ankle, data, side=side)
        assert joints.reachable
        assert joints.hip_roll is None
        assert abs(joints.hip_pitch + 0.4) < 1e-3
        assert abs(joints.knee - 0.8) < 1e-3
        assert abs(joints.ankle_pitch + 0.4) < 1e-3
        before = np.asarray(data.qpos, dtype=float).copy()
        lateral = ik.solve_lateral(ankle, data, side=side, max_lateral=0.04)
        assert np.allclose(data.qpos, before)
        assert lateral.reachable
        assert lateral.failure_reason == ""
        assert lateral.ankle_roll is None
        assert abs(lateral.hip_roll - 0.0) < 1e-6
        assert abs(lateral.hip_yaw - 0.0) < 1e-6
        assert abs(lateral.lateral_correction) < 1e-6
        assert abs(lateral.hip_pitch - joints.hip_pitch) < 1e-6
        assert abs(lateral.knee - joints.knee) < 1e-6
        assert abs(lateral.ankle_pitch - joints.ankle_pitch) < 1e-6


def check_offsets(model, data, ik: LegIK) -> list[dict]:
    rows = []
    for side in ("left", "right"):
        ankle = _ankle(model, data, side)
        limits = _limits(ik, side)
        for offset in OFFSETS_M:
            target = ankle.copy()
            target[1] += float(offset)
            solved = ik.solve_lateral(target, data, side=side, max_lateral=0.04)
            assert solved.reachable, (side, offset, solved.failure_reason)
            assert solved.failure_reason == ""
            assert solved.ankle_roll is None
            assert abs(solved.hip_yaw) < 1e-6
            actual = ik.ankle_position(data, side, solved, hip_roll=solved.hip_roll)
            error = float(np.linalg.norm(actual - target))
            assert error < 1e-4, (side, offset, error)
            assert _inside(solved.hip_roll, limits["hip_roll"])
            assert _inside(solved.hip_pitch, limits["hip_pitch"])
            assert _inside(solved.knee, limits["knee"])
            assert _inside(solved.ankle_pitch, limits["ankle"])
            rows.append(
                {
                    "side": side,
                    "offset_m": float(offset),
                    "target_y": float(target[1]),
                    "actual_y": float(actual[1]),
                    "error_m": error,
                    "hip_roll": float(solved.hip_roll),
                    "hip_yaw": float(solved.hip_yaw),
                    "hip_pitch": float(solved.hip_pitch),
                    "knee": float(solved.knee),
                    "ankle": float(solved.ankle_pitch),
                    "reachable": True,
                }
            )
    return rows


def check_unreachable(model, data, ik: LegIK) -> None:
    for side in ("left", "right"):
        ankle = _ankle(model, data, side)
        target = ankle.copy()
        target[1] += 0.20
        sagittal = ik.solve(target, data, side=side)
        solved = ik.solve_lateral(target, data, side=side, max_lateral=0.04)
        assert not solved.reachable
        assert solved.failure_reason == "lateral_limit"
        assert abs(solved.hip_roll) < 1e-6
        assert abs(solved.hip_pitch - sagittal.hip_pitch) < 1e-3
        assert abs(solved.knee - sagittal.knee) < 1e-3
        assert abs(solved.ankle_pitch - sagittal.ankle_pitch) < 1e-3
        limits = _limits(ik, side)
        assert _inside(solved.hip_roll, limits["hip_roll"])
        assert _inside(solved.hip_pitch, limits["hip_pitch"])
        assert _inside(solved.knee, limits["knee"])
        assert _inside(solved.ankle_pitch, limits["ankle"])


def check_ramp(model, data, ik: LegIK) -> None:
    for side in ("left", "right"):
        ankle = _ankle(model, data, side)
        previous = None
        for offset in np.linspace(-0.04, 0.04, 33):
            target = ankle.copy()
            target[1] += float(offset)
            solved = ik.solve_lateral(target, data, side=side, max_lateral=0.04)
            assert solved.reachable
            current = np.array(
                [solved.hip_roll, solved.hip_pitch, solved.knee, solved.ankle_pitch],
                dtype=float,
            )
            if previous is not None:
                assert np.all(np.abs(current - previous) < 0.03), (side, offset, current - previous)
            previous = current


def maximum_lateral(model, data, ik: LegIK, side: str = "right") -> float:
    """Largest |world y| the hip-roll limit can still place, in metres."""
    ankle = _ankle(model, data, side)
    best = 0.0
    for sign in (1.0, -1.0):
        low = 0.0
        high = 0.80
        for _ in range(18):
            mid = 0.5 * (low + high)
            target = ankle.copy()
            target[1] += sign * mid
            solved = ik.solve_lateral(target, data, side=side, max_lateral=1.0)
            actual = (
                ik.ankle_position(data, side, solved, hip_roll=solved.hip_roll)
                if solved.reachable
                else None
            )
            ok = solved.reachable and actual is not None and float(np.linalg.norm(actual - target)) < 1e-3
            if ok:
                low = mid
                best = max(best, mid)
            else:
                high = mid
    return float(best)


def _print_table(rows: list[dict], reach_m: float) -> None:
    print(
        f"{'side':<6} {'dy_cm':>7} {'target_y':>10} {'actual_y':>10} "
        f"{'err_mm':>8} {'roll':>8} {'yaw':>8} {'pitch':>8} {'knee':>8} {'ankle':>8} {'ok':>4}"
    )
    for row in rows:
        print(
            f"{row['side']:<6} {100.0 * row['offset_m']:7.1f} {row['target_y']:10.4f} "
            f"{row['actual_y']:10.4f} {1000.0 * row['error_m']:8.3f} {row['hip_roll']:8.4f} "
            f"{row['hip_yaw']:8.4f} {row['hip_pitch']:8.4f} {row['knee']:8.4f} "
            f"{row['ankle']:8.4f} {'yes' if row['reachable'] else 'no':>4}"
        )
    print(f"max lateral before hip-roll limit: {100.0 * reach_m:.2f} cm")


def main() -> None:
    model, data = _home()
    ik = LegIK(model)
    check_no_ankle_roll(model)
    check_sagittal_unchanged(model, data, ik)
    rows = check_offsets(model, data, ik)
    check_unreachable(model, data, ik)
    check_ramp(model, data, ik)
    reach = maximum_lateral(model, data, ik, "right")
    _print_table(rows, reach)
    print("lateral IK tests passed")


if __name__ == "__main__":
    main()
