"""Controlled lateral fall. The sign check uses the H1 model; the rest does not."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco

from controllers.support_transition import (  # noqa: E402
    FALL_REACH,
    RELEASE_DWELL_S,
    RELEASE_FORCE_N,
    RELEASE_MARGIN_M,
    FallRelease,
    apply_stance_hip_roll,
    combined_support_active,
    fall_blend,
    fall_safe_box,
    stance_hip_roll_capture_target,
)

MODEL = ROOT / "models" / "unitree_h1" / "scene.xml"
LEFT = np.array([[0.00, 0.22], [0.18, 0.22], [0.18, 0.29], [0.00, 0.29]])
RIGHT = np.array([[0.05, -0.16], [0.18, -0.16], [0.18, -0.09], [0.05, -0.09]])


def _planted_com_dy(side: str, dq: float) -> float:
    """Change in CoM y when ``side`` hip roll moves and that foot stays put."""
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    ankle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_link")
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_hip_roll")
    address = int(model.jnt_qposadr[joint])
    com0 = data.subtree_com[0].copy()
    foot0 = data.xpos[ankle].copy()
    data.qpos[address] += float(dq)
    mujoco.mj_forward(model, data)
    data.qpos[0:3] += foot0 - data.xpos[ankle]
    mujoco.mj_forward(model, data)
    return float(data.subtree_com[0][1] - com0[1])


def test_early_swing_preserves_the_stance_hip():
    assert fall_blend(0.20, "RIGHT_SWING") == 0.0
    base = -0.155
    target = stance_hip_roll_capture_target(base, 0.24, 0.0, weight=0.0)
    assert abs(target - base) < 1e-12


def test_late_swing_changes_only_the_stance_hip():
    names = ["left_hip_roll", "right_hip_roll", "left_knee"]
    q = np.array([-0.155, 0.40, 0.80])
    low = np.full(3, -0.43)
    high = np.full(3, 0.43)
    updated = apply_stance_hip_roll(q, names, "left", -0.005, low, high)
    assert abs(float(updated[1]) - 0.40) < 1e-12
    assert abs(float(updated[2]) - 0.80) < 1e-12
    assert abs(float(updated[0]) - (-0.005)) < 1e-12
    assert abs(fall_blend(0.70, "RIGHT_SWING") - 1.0) < 1e-12


def test_right_swing_correction_moves_the_body_inward():
    base = -0.155
    target = stance_hip_roll_capture_target(base, 0.24, 0.0, weight=1.0)
    delta = target - base
    assert delta > 0.0
    # Positive left hip roll, foot planted, moves the CoM toward negative y.
    assert _planted_com_dy("left", delta) < 0.0


def test_left_swing_mirrors_the_correction():
    base = 0.155
    target = stance_hip_roll_capture_target(base, -0.24, 0.0, weight=1.0)
    delta = target - base
    assert delta < 0.0
    # Negative right hip roll, foot planted, moves the CoM toward positive y.
    assert _planted_com_dy("right", delta) > 0.0


def test_correction_ramps_smoothly():
    assert fall_blend(0.30, "RIGHT_SWING") == 0.0
    mid = fall_blend(0.50, "RIGHT_SWING")
    assert 0.0 < mid < 1.0
    assert abs(fall_blend(0.70, "RIGHT_SWING") - 1.0) < 1e-12
    assert fall_blend(0.40, "RIGHT_SWING") < mid < fall_blend(0.60, "RIGHT_SWING")


def test_correction_is_clipped_at_015_rad():
    base = -0.155
    near = stance_hip_roll_capture_target(base, 0.24, 0.0, weight=1.0)
    far = stance_hip_roll_capture_target(base, 2.0, 0.0, weight=1.0)
    assert abs(near - base - 0.15) < 1e-12
    assert abs(far - base - 0.15) < 1e-12
    other = stance_hip_roll_capture_target(0.10, -2.0, 0.0, weight=1.0)
    assert abs(other - (0.10 - 0.15)) < 1e-12


def test_swing_leg_ik_is_left_unchanged():
    names = ["left_hip_roll", "right_hip_roll"]
    q = np.array([-0.155, 0.33])
    updated = apply_stance_hip_roll(q, names, "left", 0.0, np.full(2, -1.0), np.full(2, 1.0))
    assert abs(float(updated[1]) - 0.33) < 1e-12


def test_inner_boundary_moves_smoothly_inward():
    shut = fall_safe_box(LEFT, "right", 0.0)
    full = fall_safe_box(LEFT, "right", 1.0)
    assert abs(float(shut[0][1]) - 0.22) < 1e-12
    assert abs(float(full[0][1]) - (0.22 - FALL_REACH)) < 1e-12
    mid = fall_safe_box(LEFT, "right", 0.5)
    assert float(full[0][1]) < float(mid[0][1]) < float(shut[0][1])


def test_outer_toe_and_heel_stay_put():
    shut = fall_safe_box(LEFT, "right", 0.0)
    full = fall_safe_box(LEFT, "right", 1.0)
    assert abs(float(full[1][1]) - float(shut[1][1])) < 1e-12
    assert abs(float(full[0][0]) - float(shut[0][0])) < 1e-12
    assert abs(float(full[1][0]) - float(shut[1][0])) < 1e-12
    mirrored = fall_safe_box(RIGHT, "left", 1.0)
    closed = fall_safe_box(RIGHT, "left", 0.0)
    assert abs(float(mirrored[1][1]) - (float(closed[1][1]) + FALL_REACH)) < 1e-12
    assert abs(float(mirrored[0][1]) - float(closed[0][1])) < 1e-12


def test_touchdown_keeps_the_correction():
    assert abs(fall_blend(0.0, "TOUCHDOWN") - 1.0) < 1e-12
    assert abs(fall_blend(0.0, "DOUBLE_SUPPORT_RECOVERY") - 1.0) < 1e-12
    base = -0.155
    held = stance_hip_roll_capture_target(base, 0.24, 0.0, weight=1.0, release=0.0)
    assert abs(held - (base + 0.15)) < 1e-12


def test_combined_support_waits_for_confirmed_contact():
    assert combined_support_active("TOUCHDOWN", True) is False
    assert combined_support_active("DOUBLE_SUPPORT_RECOVERY", False) is False
    assert combined_support_active("DOUBLE_SUPPORT_RECOVERY", True) is True


def test_release_waits_for_capture_margin_and_sustained_load():
    release = FallRelease()
    dt = 0.02
    steps = int(RELEASE_DWELL_S / dt)
    for _ in range(steps):
        blend = release.update(
            in_recovery=True,
            capture_margin=RELEASE_MARGIN_M + 0.01,
            new_foot_force=RELEASE_FORCE_N,
            dt=dt,
        )
        assert blend == 0.0
    started = release.update(
        in_recovery=True,
        capture_margin=RELEASE_MARGIN_M + 0.01,
        new_foot_force=RELEASE_FORCE_N,
        dt=dt,
    )
    assert started == 0.0
    assert release.started
    later = release.update(
        in_recovery=True,
        capture_margin=RELEASE_MARGIN_M + 0.01,
        new_foot_force=RELEASE_FORCE_N,
        dt=RELEASE_DWELL_S,
    )
    assert later > 0.0
    outside = FallRelease()
    for _ in range(20):
        blend = outside.update(
            in_recovery=True,
            capture_margin=-0.05,
            new_foot_force=200.0,
            dt=0.02,
        )
    assert blend == 0.0
    assert not outside.started
    returned = stance_hip_roll_capture_target(-0.155, 0.24, 0.0, weight=1.0, home=0.0, release=1.0)
    assert abs(returned - 0.0) < 1e-12


def test_standing_does_not_open_the_fall():
    assert fall_blend(0.0, "STAND") == 0.0
    assert fall_blend(1.0, "SHIFT_LEFT") == 0.0
    base = -0.02
    assert abs(stance_hip_roll_capture_target(base, 0.0, 0.0, weight=0.0) - base) < 1e-12


def _main() -> None:
    tests = [
        test_early_swing_preserves_the_stance_hip,
        test_late_swing_changes_only_the_stance_hip,
        test_right_swing_correction_moves_the_body_inward,
        test_left_swing_mirrors_the_correction,
        test_correction_ramps_smoothly,
        test_correction_is_clipped_at_015_rad,
        test_swing_leg_ik_is_left_unchanged,
        test_inner_boundary_moves_smoothly_inward,
        test_outer_toe_and_heel_stay_put,
        test_touchdown_keeps_the_correction,
        test_combined_support_waits_for_confirmed_contact,
        test_release_waits_for_capture_margin_and_sustained_load,
        test_standing_does_not_open_the_fall,
    ]
    for test in tests:
        test()
        print("passed", test.__name__)
    print(f"{len(tests)} passed")


if __name__ == "__main__":
    _main()
