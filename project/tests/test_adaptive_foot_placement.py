"""Late-swing lateral landing adjustment.

These checks do not run MuJoCo. Reachability is a flag the walking tick
sets from the leg IK; the state machine only has to honour it.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers.adaptive_foot_placement import (  # noqa: E402
    lateral_landing_correction,
    placement_ramp,
)
from controllers.walk_state_machine import WalkConfig, WalkingStateMachine, WalkState  # noqa: E402


def _machine(adaptive: bool, first: str = "right") -> WalkingStateMachine:
    gait = WalkingStateMachine(
        WalkConfig(
            stand_duration=0.0,
            shift_duration=0.02,
            step_duration=1.0,
            touchdown_prep_duration=0.20,
            step_length=0.1,
            adaptive_foot_placement=adaptive,
            capture_footstep_gain=0.5,
            max_lateral_step_adjustment=0.08,
        )
    )
    gait.start(
        np.array([0.0, 0.2, 0.0]),
        np.array([0.0, -0.2, 0.0]),
        0.95,
        n_steps=1,
        first_swing=first,
    )
    return gait


def _drive(gait: WalkingStateMachine, capture_y: float, stance_y: float, reachable: bool = True):
    rows = []
    sample = None
    for _ in range(80):
        sample = gait.update(
            0.02,
            True,
            capture_xy=np.array([0.0, capture_y]),
            placement_reachable=reachable,
            stance_foot_y=stance_y,
        )
        progress = gait.swing_elapsed / gait.config.step_duration
        foot = sample.right_foot if gait.active_swing == "right" else sample.left_foot
        rows.append((gait.state, progress, foot.copy(), gait.adaptive_lateral_correction))
        if gait.state is WalkState.TOUCHDOWN:
            break
    return rows, sample


def test_disabled_keeps_the_original_target():
    plain = _machine(False)
    plain_rows, plain_final = _drive(plain, 0.05, 0.2, True)
    # Same capture, but the flag is off, so the landing y stays at liftoff.
    off = _machine(False)
    off_rows, off_final = _drive(off, 0.05, 0.2, True)
    assert plain_final is not None and off_final is not None
    assert off_final.state is WalkState.TOUCHDOWN
    assert abs(float(off_final.right_foot[1]) - (-0.2)) < 1e-9
    assert abs(float(off_final.right_foot[0]) - float(plain_final.right_foot[0])) < 1e-9
    assert abs(float(off_final.right_foot[0]) - 0.1) < 1e-9
    assert all(abs(correction) < 1e-12 for _state, _progress, _foot, correction in off_rows)


def test_correction_is_bounded_by_8_cm():
    huge = lateral_landing_correction(
        -0.2, 0.2, 0.5, gain=0.5, max_adjustment=0.08
    )
    other_way = lateral_landing_correction(
        0.2, -0.2, -0.5, gain=1.0, max_adjustment=0.08
    )
    assert abs(huge) <= 0.08 + 1e-12
    assert abs(other_way) <= 0.08 + 1e-12
    assert huge > 0.0
    assert other_way < 0.0
    gait = _machine(True)
    _rows, final = _drive(gait, 0.5, 0.2, True)
    assert final is not None
    assert abs(float(final.right_foot[1]) - (-0.2)) <= 0.08 + 1e-9
    assert abs(float(final.right_foot[0]) - 0.1) < 1e-9


def test_correction_ramps_smoothly_and_is_zero_before_065():
    assert placement_ramp(0.64) == 0.0
    assert placement_ramp(0.65) == 0.0
    assert abs(placement_ramp(1.0) - 1.0) < 1e-12
    earlier = [placement_ramp(progress) for progress in (0.70, 0.80, 0.90, 1.0)]
    assert earlier == sorted(earlier)
    # The smoothstep slope is zero at the opening, so the first tick is small.
    assert placement_ramp(0.67) < 0.02
    gait = _machine(True)
    rows, final = _drive(gait, 0.05, 0.2, True)
    assert final is not None
    swing = [(progress, foot[1]) for state, progress, foot, _correction in rows if state in (
        WalkState.RIGHT_SWING, WalkState.TOUCHDOWN_PREP, WalkState.TOUCHDOWN
    )]
    for progress, y in swing:
        if progress < 0.65:
            assert abs(y - (-0.2)) < 1e-9
    moving = [y for progress, y in swing if progress >= 0.65]
    assert moving[0] <= moving[-1]
    assert all(moving[i] <= moving[i + 1] + 1e-12 for i in range(len(moving) - 1))
    steps = [abs(moving[i + 1] - moving[i]) for i in range(len(moving) - 1)]
    assert max(steps) < 0.01
    assert abs(float(final.right_foot[1]) - (-0.15)) < 1e-6
    assert abs(float(final.right_foot[0]) - 0.1) < 1e-9


def test_capture_correction_has_the_right_sign_on_both_feet():
    right = lateral_landing_correction(-0.2, 0.2, 0.05, gain=0.5, max_adjustment=0.08)
    left = lateral_landing_correction(0.2, -0.2, -0.05, gain=0.5, max_adjustment=0.08)
    assert abs(right - 0.05) < 1e-9
    assert abs(left - (-0.05)) < 1e-9
    right_gait = _machine(True, "right")
    _rows, right_final = _drive(right_gait, 0.05, 0.2, True)
    assert right_final is not None
    assert abs(float(right_final.right_foot[1]) - (-0.15)) < 1e-6
    left_gait = _machine(True, "left")
    _rows, left_final = _drive(left_gait, -0.05, -0.2, True)
    assert left_final is not None
    assert left_final.state is WalkState.TOUCHDOWN
    assert abs(float(left_final.left_foot[1]) - 0.15) < 1e-6
    assert abs(float(left_final.left_foot[0]) - 0.1) < 1e-9


def test_unreachable_target_falls_back_to_the_original():
    gait = _machine(True)
    _rows, final = _drive(gait, 0.5, 0.2, reachable=False)
    assert final is not None
    assert gait.adaptive_placement_fallback is True
    assert abs(float(final.right_foot[1]) - (-0.2)) < 1e-9
    assert abs(gait.adaptive_lateral_correction) < 1e-12
    # A later reachable flag does not revive a rejected landing.
    revived = gait.update(
        0.02,
        True,
        capture_xy=np.array([0.0, 0.5]),
        placement_reachable=True,
        stance_foot_y=0.2,
    )
    assert abs(float(revived.right_foot[1]) - (-0.2)) < 1e-9


def test_no_correction_before_swing_progress_065():
    gait = _machine(True)
    rows, _final = _drive(gait, 0.5, 0.2, True)
    early = [
        foot[1]
        for state, progress, foot, _correction in rows
        if state in (WalkState.RIGHT_SWING, WalkState.TOUCHDOWN_PREP) and progress < 0.65
    ]
    assert early
    assert all(abs(y - (-0.2)) < 1e-9 for y in early)


def main() -> None:
    test_disabled_keeps_the_original_target()
    test_correction_is_bounded_by_8_cm()
    test_correction_ramps_smoothly_and_is_zero_before_065()
    test_capture_correction_has_the_right_sign_on_both_feet()
    test_unreachable_target_falls_back_to_the_original()
    test_no_correction_before_swing_progress_065()
    print("adaptive foot placement tests passed")


if __name__ == "__main__":
    main()
