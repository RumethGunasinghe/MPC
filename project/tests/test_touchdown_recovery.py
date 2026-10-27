"""Closed-loop touchdown confirmation and double-support recovery.

These checks do not run MuJoCo. They exercise the phase rules and the
support geometry the walking tick uses after the swing trajectory ends.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers.lateral_shift import LateralWeightShift
from controllers.mpc_controller import suppress_landed_ankle_capture
from controllers.walk_state_machine import (
    WalkConfig,
    WalkingStateMachine,
    WalkState,
    contact_qualified_corners,
    recovery_conditions_met,
    touchdown_sample_ok,
)


def _gait() -> WalkingStateMachine:
    config = WalkConfig(
        stand_duration=0.0,
        shift_duration=0.02,
        step_duration=0.08,
        touchdown_prep_duration=0.04,
        touchdown_confirm_ticks=3,
        stabilization_dwell=0.06,
        recovery_duration=0.40,
        step_length=0.1,
    )
    gait = WalkingStateMachine(config)
    gait.start(
        np.array([0.0, 0.2, 0.0]),
        np.array([0.0, -0.2, 0.0]),
        0.95,
        n_steps=1,
        first_swing="right",
    )
    for _ in range(30):
        sample = gait.update(0.02, True)
        if sample.state is WalkState.TOUCHDOWN:
            return gait
    raise AssertionError(f"did not reach touchdown, state={gait.state}")


def test_touchdown_requires_multiple_samples():
    gait = _gait()
    assert gait.touchdown_confirm_count == 0
    for expected in (1, 2):
        sample = gait.update(0.02, True, foot_sample_ok=True)
        assert sample.state is WalkState.TOUCHDOWN
        assert gait.touchdown_confirm_count == expected
    sample = gait.update(0.02, True, foot_sample_ok=True)
    assert sample.state is WalkState.DOUBLE_SUPPORT_RECOVERY
    assert gait.touchdown_confirm_count == 3


def test_touchdown_rejected_without_contact():
    assert touchdown_sample_ok(
        in_contact=False, foot_z=0.02, foot_vz=0.0, height_limit=0.09, vz_min=-0.2
    ) is False
    gait = _gait()
    for _ in range(25):
        sample = gait.update(0.02, True, foot_sample_ok=False)
    assert sample.state is WalkState.TOUCHDOWN
    assert gait.touchdown_confirm_count == 0
    # One true sample does not survive a missed contact.
    gait.update(0.02, True, foot_sample_ok=True)
    gait.update(0.02, True, foot_sample_ok=True)
    sample = gait.update(0.02, True, foot_sample_ok=False)
    assert sample.state is WalkState.TOUCHDOWN
    assert gait.touchdown_confirm_count == 0


def test_recovery_stays_while_lateral_velocity_is_high():
    stable = _box()
    stable["vy"] = 0.2
    assert recovery_conditions_met(**stable) is False
    gait = _enter_recovery()
    for _ in range(30):
        sample = gait.update(0.02, True, recovery_sample_ok=False)
    assert sample.state is WalkState.DOUBLE_SUPPORT_RECOVERY


def test_recovery_stays_while_capture_is_outside():
    stable = _box()
    stable["capture_xy"] = stable["capture_xy"] + np.array([0.0, 0.5])
    assert recovery_conditions_met(**stable) is False
    gait = _enter_recovery()
    for _ in range(25):
        sample = gait.update(0.02, True, recovery_sample_ok=False)
    assert sample.state is WalkState.DOUBLE_SUPPORT_RECOVERY


def test_recovery_completes_only_after_the_dwell():
    gait = _enter_recovery()
    # recovery_duration is 0.40 s and the dwell is 0.06 s. Time alone, with
    # the conditions false, must not start the next phase.
    for _ in range(20):
        sample = gait.update(0.02, True, recovery_sample_ok=False)
    assert sample.state is WalkState.DOUBLE_SUPPORT_RECOVERY
    # Conditions then have to hold for the dwell, and the minimum residence
    # has to have elapsed. 0.40 s already passed, so the dwell is the remainder.
    dwell_ticks = int(round(0.06 / 0.02))
    for _ in range(dwell_ticks - 1):
        sample = gait.update(0.02, True, recovery_sample_ok=True)
        assert sample.state is WalkState.DOUBLE_SUPPORT_RECOVERY
    sample = gait.update(0.02, True, recovery_sample_ok=True)
    assert sample.state is WalkState.STAND


def test_support_uses_only_contact_qualified_feet():
    left = np.array([[0.0, 0.18], [0.1, 0.18], [0.1, 0.22], [0.0, 0.22]])
    right = np.array([[0.1, -0.22], [0.2, -0.22], [0.2, -0.18], [0.1, -0.18]])
    soles = {"left": left, "right": right}
    one = contact_qualified_corners(soles, {"left": True, "right": False})
    assert one.shape == (4, 2)
    assert np.allclose(one.min(axis=0), left.min(axis=0))
    assert np.allclose(one.max(axis=0), left.max(axis=0))
    both = contact_qualified_corners(soles, {"left": True, "right": True})
    assert both.shape == (8, 2)
    assert np.allclose(both.min(axis=0), [0.0, -0.22])
    assert np.allclose(both.max(axis=0), [0.2, 0.22])
    none = contact_qualified_corners(soles, {"left": False, "right": False})
    assert none.shape == (0, 2)


def test_recovery_hip_roll_keeps_the_capture_term():
    shift = LateralWeightShift(None)
    shift.solved = True
    shift.side = "left"
    shift.fraction = 0.5
    shift.pose = {"left_hip_roll": -0.4, "right_hip_roll": -0.4}
    shift.mode = "recovery"
    shift._com = np.array([0.05, 0.24, 0.95])
    shift.origin_xy = np.array([0.05, 0.0])
    names = ["left_hip_roll", "right_hip_roll"]
    home = np.zeros(2)
    commanded = np.array([0.11, -0.05])
    limits = np.full(2, 2.0)
    recovered = shift.apply(commanded, names, home, -limits, limits)
    # Blend is -0.2. Recovery adds the MPC command measured from home.
    assert np.allclose(recovered, [-0.09, -0.25])
    shift.mode = "shift"
    latched = shift.apply(commanded, names, home, -limits, limits)
    assert np.allclose(latched, [-0.2, -0.2])


def test_hold_touchdown_hip_roll_keeps_the_shift_blend():
    shift = LateralWeightShift(None)
    shift.solved = True
    shift.side = "left"
    shift.fraction = 0.5
    shift.pose = {"left_hip_roll": -0.4, "right_hip_roll": -0.4}
    shift.mode = "recovery"
    shift.hold_touchdown_hip_roll = True
    shift._com = np.array([0.05, 0.24, 0.95])
    shift.origin_xy = np.array([0.05, 0.0])
    names = ["left_hip_roll", "right_hip_roll"]
    home = np.zeros(2)
    commanded = np.array([0.11, -0.05])
    limits = np.full(2, 2.0)
    held = shift.apply(commanded, names, home, -limits, limits)
    # Blend is -0.2 on both legs. The hold does not add the capture term,
    # and it does not single out one side.
    assert np.allclose(held, [-0.2, -0.2])
    assert shift.hip_roll_capture["left_hip_roll"] == 0.0
    assert shift.hip_roll_capture["right_hip_roll"] == 0.0
    assert abs(shift.hip_roll_capture_available["left_hip_roll"] - 0.11) < 1e-9
    assert abs(shift.hip_roll_capture_available["right_hip_roll"] - (-0.05)) < 1e-9


def test_recovery_reference_does_not_follow_the_body():
    shift = LateralWeightShift(None)
    shift.solved = True
    shift.side = "left"
    shift.fraction = 0.5
    shift._on_sole = True
    shift.mode = "recovery"
    shift.reference_speed = 0.1
    shift.origin_xy = np.array([0.05, 0.0])
    shift.target_xy = np.array([0.05, 0.20])
    shift.sole_low = np.array([0.0, 0.18])
    shift.sole_high = np.array([0.12, 0.22])
    shift.recovery_target = np.array([0.05, 0.0])
    shift._held_xy = np.array([0.05, 0.20])
    reference = shift.advance(
        0.02,
        np.array([0.05, 0.30, 0.95]),
        np.array([0.0, 0.4, 0.0]),
        shift.sole_low,
    )
    # The body is at y = 0.30 and moving outward. The reference steps toward
    # the support centre, 0.1 m/s * 0.02 s = 2 mm, and does not follow y = 0.30.
    assert abs(float(reference[1]) - 0.198) < 1e-9
    assert float(reference[1]) < 0.20


def test_landed_ankle_keeps_the_landing_target():
    landing = -0.2443
    capture = -0.15
    names = ["left_hip_roll", "left_ankle", "right_hip_roll", "right_ankle"]
    offset = np.array([0.15, capture, 0.15, capture])
    right = suppress_landed_ankle_capture(offset, names, "right", allow_capture=False)
    # The bug added the clipped sagittal term: -0.2443 + -0.15 = -0.3943.
    assert abs((landing + capture) - (-0.3943)) < 1e-9
    assert abs((landing + float(right[3])) - landing) < 1e-9
    # The stance ankle and both hip rolls still carry their capture terms.
    assert abs(float(right[1]) - capture) < 1e-9
    assert np.allclose(right[[0, 2]], [0.15, 0.15])
    left = suppress_landed_ankle_capture(offset, names, "left", allow_capture=False)
    assert abs((landing + float(left[1])) - landing) < 1e-9
    assert abs(float(left[3]) - capture) < 1e-9
    restored = suppress_landed_ankle_capture(offset, names, "right", allow_capture=True)
    assert abs(float(restored[3]) - capture) < 1e-9


def _enter_recovery() -> WalkingStateMachine:
    gait = _gait()
    for _ in range(3):
        sample = gait.update(0.02, True, foot_sample_ok=True)
    assert sample.state is WalkState.DOUBLE_SUPPORT_RECOVERY
    return gait


def _box() -> dict:
    return dict(
        both_contact=True,
        com_xy=np.array([0.05, 0.0]),
        capture_xy=np.array([0.05, 0.0]),
        box_low=np.array([-0.02, -0.2]),
        box_high=np.array([0.2, 0.2]),
        vx=0.0,
        vy=0.0,
        roll=0.0,
        pitch=0.0,
        margin=0.01,
        max_vx=0.08,
        max_vy=0.03,
        max_roll=0.35,
        max_pitch=0.35,
    )


if __name__ == "__main__":
    test_touchdown_requires_multiple_samples()
    test_touchdown_rejected_without_contact()
    test_recovery_stays_while_lateral_velocity_is_high()
    test_recovery_stays_while_capture_is_outside()
    test_recovery_completes_only_after_the_dwell()
    test_support_uses_only_contact_qualified_feet()
    test_recovery_hip_roll_keeps_the_capture_term()
    test_recovery_reference_does_not_follow_the_body()
    test_landed_ankle_keeps_the_landing_target()
    test_hold_touchdown_hip_roll_keeps_the_shift_blend()
    print("10 passed")
