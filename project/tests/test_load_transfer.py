"""Load-transfer schedule. No MuJoCo and no torque.

Desired fractions move the lateral reference and a bounded hip-roll offset.
They are not contact-force commands.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers.lateral_shift import LateralWeightShift
from controllers.load_transfer import (
    LoadTransferTracker,
    hip_roll_correction,
    load_fractions,
    load_sample_ok,
    plan_load_transfer,
    smoothstep,
)
from controllers.mpc_controller import suppress_landed_ankle_capture


def test_alpha_starts_at_zero_and_ends_at_one():
    new_0, old_0 = load_fractions(0.0, 0.40)
    new_1, old_1 = load_fractions(0.40, 0.40)
    assert new_0 == 0.0
    assert old_0 == 1.0
    assert abs(new_1 - 1.0) < 1e-12
    assert abs(old_1) < 1e-12
    assert abs(smoothstep(0.0)) < 1e-12
    assert abs(smoothstep(1.0) - 1.0) < 1e-12


def test_alpha_is_monotonic_and_fractions_sum_to_one():
    previous = -1.0
    for step in range(21):
        time_since = 0.40 * step / 20.0
        new_fraction, old_fraction = load_fractions(time_since, 0.40)
        assert new_fraction + 1e-12 >= previous
        assert abs((new_fraction + old_fraction) - 1.0) < 1e-12
        previous = new_fraction


def test_com_target_stays_within_the_offset_and_between_the_soles():
    plan = plan_load_transfer(
        state="DOUBLE_SUPPORT_RECOVERY",
        time_since=0.40,
        duration=0.40,
        old_y=0.25,
        new_y=-0.05,
        reference_x=0.06,
        reference_y=0.20,
        max_com_offset=0.04,
        max_hip_roll=0.08,
    )
    assert plan.active
    assert plan.target_x == 0.06
    assert abs((plan.target_y - 0.25)) <= 0.04 + 1e-12
    assert -0.05 <= plan.target_y <= 0.25
    assert abs(plan.target_y - 0.21) < 1e-12


def test_hip_roll_correction_stays_inside_its_limit():
    for alpha in (0.0, 0.25, 0.5, 1.0):
        toward_right = hip_roll_correction(0.25, -0.05, alpha, 0.08)
        toward_left = hip_roll_correction(-0.05, 0.25, alpha, 0.08)
        assert abs(toward_right) <= 0.08 + 1e-12
        assert abs(toward_left) <= 0.08 + 1e-12
        assert toward_right >= 0.0
        assert toward_left <= 0.0
    assert abs(hip_roll_correction(0.25, -0.05, 1.0, 0.08) - 0.08) < 1e-12


def test_load_transfer_is_inactive_outside_recovery():
    for state in ("STAND", "RIGHT_SWING", "TOUCHDOWN_PREP", "TOUCHDOWN", ""):
        plan = plan_load_transfer(
            state=state,
            time_since=0.40,
            duration=0.40,
            old_y=0.25,
            new_y=-0.05,
            reference_x=0.06,
            reference_y=0.20,
            max_com_offset=0.04,
            max_hip_roll=0.08,
        )
        assert not plan.active
        assert plan.hip_roll == 0.0
        assert plan.new_fraction == 0.0
        assert plan.old_fraction == 1.0
        assert plan.target_y == 0.20


def test_offset_is_added_on_top_of_hold_and_of_capture():
    shift = LateralWeightShift(None)
    shift.solved = True
    shift.side = "left"
    shift.fraction = 0.5
    shift.pose = {"left_hip_roll": -0.4, "right_hip_roll": -0.4}
    shift.mode = "recovery"
    shift._com = np.array([0.05, 0.24, 0.95])
    shift.origin_xy = np.array([0.05, 0.0])
    shift.load_transfer_hip_roll = 0.05
    shift.load_transfer_hip_roll_limit = 0.08
    names = ["left_hip_roll", "right_hip_roll"]
    home = np.zeros(2)
    commanded = np.array([0.11, -0.05])
    limits = np.full(2, 2.0)
    shift.hold_touchdown_hip_roll = True
    held = shift.apply(commanded, names, home, -limits, limits)
    # Blend is -0.2. The hold keeps that base, then adds +0.05.
    assert np.allclose(held, [-0.15, -0.15])
    shift.hold_touchdown_hip_roll = False
    released = shift.apply(commanded, names, home, -limits, limits)
    # Blend plus capture, then the same +0.05 offset.
    assert np.allclose(released, [-0.04, -0.20])


def test_success_requires_the_final_window_and_the_dwell():
    tracker = LoadTransferTracker()
    for _ in range(15):
        tracker.observe(
            active=True, time=0.2, sample_ok=True, in_final_window=False, dwell=0.10, dt=0.02
        )
    assert not tracker.success
    for _ in range(4):
        tracker.observe(
            active=True, time=0.34, sample_ok=True, in_final_window=True, dwell=0.10, dt=0.02
        )
    assert not tracker.success
    tracker.observe(
        active=True, time=0.42, sample_ok=True, in_final_window=True, dwell=0.10, dt=0.02
    )
    assert tracker.success
    weak = load_sample_ok(
        new_force=10.0,
        old_force=400.0,
        both_contact=True,
        com_z=0.95,
        vy=0.0,
        capture_inside=True,
        cbf_status="optimal",
        min_fraction=0.25,
        max_vy=0.03,
    )
    assert not weak


def test_landed_ankle_capture_stays_suppressed():
    landing = -0.2443
    capture = -0.15
    names = ["left_ankle", "right_ankle"]
    kept = suppress_landed_ankle_capture(
        np.array([capture, capture]), names, "right", allow_capture=False
    )
    assert abs((landing + float(kept[1])) - landing) < 1e-9
    assert abs((landing + capture) - (-0.3943)) < 1e-9


if __name__ == "__main__":
    test_alpha_starts_at_zero_and_ends_at_one()
    test_alpha_is_monotonic_and_fractions_sum_to_one()
    test_com_target_stays_within_the_offset_and_between_the_soles()
    test_hip_roll_correction_stays_inside_its_limit()
    test_load_transfer_is_inactive_outside_recovery()
    test_offset_is_added_on_top_of_hold_and_of_capture()
    test_success_requires_the_final_window_and_the_dwell()
    test_landed_ankle_capture_stays_suppressed()
    print("8 passed")
