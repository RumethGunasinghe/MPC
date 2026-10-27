"""Capture-step foot placement.

The capture point is a landing target. These checks do not run the walking
demo. The leg-IK check uses the H1 model; the rest are pure.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers.capture_point_controller import capture_point  # noqa: E402
from controllers.capture_step import (  # noqa: E402
    FREEZE_PROGRESS,
    MAX_FORWARD_CORRECTION,
    MAX_LATERAL_CORRECTION,
    SOLE_HALF_WIDTH,
    CaptureBodyPrep,
    CaptureStepTracker,
    bounded_foot_correction,
    fall_or_recover,
    hip_track_interval,
    horizontal_capture,
    candidate_ankle_y,
    capture_is_prepared,
    PREPARE_WORSEN_TICKS,
    catchable_capture_interval,
    dynamic_catchable,
    dynamic_foot_interval,
    dynamic_q_interval,
    feasible_capture_interval,
    foot_is_airborne,
    interval_distance,
    liftoff_permitted,
    HIP_OFFSET_LIMIT,
    PREPARE_CAPTURE_HIP_LIMIT,
    prepare_hip_roll_command,
    prepare_hip_targets,
    prepare_hip_trace,
    prepare_lateral_velocity,
    select_capture_target,
    support_transfer_sample_ok,
    torque_lead,
    update_worsen_count,
    landing_is_capture_valid,
    lateral_is_trackable,
    predict_touchdown_capture,
    preparation_target_y,
    search_reachable_y,
    project_correction,
    project_to_interval,
)
from controllers.footstep_planner import SwingFootTrajectory  # noqa: E402
from controllers.walk_state_machine import WalkConfig, WalkingStateMachine, WalkState  # noqa: E402


def test_capture_point_calculation():
    com = np.array([0.1, 0.2, 0.9])
    vel = np.array([0.0, 0.0, 0.0])
    point, omega = horizontal_capture(com, vel)
    assert abs(omega - np.sqrt(9.81 / 0.9)) < 1e-12
    assert np.allclose(point, com[:2])
    assert np.allclose(point, capture_point(com, vel))


def test_capture_point_responds_to_com_velocity():
    com = np.array([0.0, 0.24, 0.9])
    inward = np.array([0.0, -0.20, 0.0])
    outward = np.array([0.0, 0.20, 0.0])
    point_in, omega = horizontal_capture(com, inward)
    point_out, _omega = horizontal_capture(com, outward)
    assert point_in[1] < com[1]
    assert point_out[1] > com[1]
    assert abs((point_in[1] - com[1]) - (-0.20 / omega)) < 1e-12


def test_capture_target_is_bounded():
    nominal = np.array([0.05, -0.20])
    capture = np.array([1.0, 0.40])
    correction = bounded_foot_correction(nominal, capture)
    assert abs(correction[0]) <= MAX_FORWARD_CORRECTION + 1e-12
    assert abs(correction[1]) <= MAX_LATERAL_CORRECTION + 1e-12
    assert abs(correction[0] - MAX_FORWARD_CORRECTION) < 1e-12
    assert abs(correction[1] - MAX_LATERAL_CORRECTION) < 1e-12
    small = bounded_foot_correction(nominal, nominal + np.array([0.02, -0.03]))
    assert np.allclose(small, [0.02, -0.03])


def test_unreachable_capture_target_is_projected():
    nominal = np.array([0.0, -0.20])
    correction = np.array([0.0, 0.16])

    def reachable(xy: np.ndarray) -> bool:
        return abs(float(xy[1]) - (-0.20)) <= 0.05

    accepted, fallback = project_correction(nominal, correction, reachable)
    assert fallback
    assert reachable(accepted)
    assert abs(float(accepted[1]) - (-0.15)) < 1e-3
    assert abs(float(accepted[0]) - nominal[0]) < 1e-9


def test_lateral_target_uses_existing_3d_ik():
    import mujoco

    from controllers.leg_ik import LegIK

    model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "unitree_h1" / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    ik = LegIK(model)
    ankle = np.asarray(
        data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_link")],
        dtype=float,
    )
    near = ankle.copy()
    near[1] += 0.04
    reached = ik.solve_lateral(near, data, side="right", max_lateral=MAX_LATERAL_CORRECTION)
    assert reached.reachable
    assert reached.hip_roll is not None
    far = ankle.copy()
    far[1] += 1.0
    missed = ik.solve_lateral(far, data, side="right", max_lateral=MAX_LATERAL_CORRECTION)
    assert not missed.reachable

    def reachable(xy: np.ndarray) -> bool:
        target = np.array([float(xy[0]), float(xy[1]), float(ankle[2])])
        return bool(
            ik.solve_lateral(target, data, side="right", max_lateral=MAX_LATERAL_CORRECTION).reachable
        )

    accepted, fallback = project_correction(ankle[:2], np.array([0.0, 1.0]), reachable)
    assert fallback
    assert reachable(accepted)
    assert abs(float(accepted[1] - ankle[1])) < 1.0


def test_sagittal_trajectory_unchanged_for_lateral_correction_only():
    start = np.array([0.0, -0.20, 0.07])
    nominal = np.array([0.10, -0.20, 0.07])
    lateral = nominal.copy()
    lateral[1] += 0.08
    plain = SwingFootTrajectory(start, nominal, 0.8, 0.05)
    shifted = SwingFootTrajectory(start, lateral, 0.8, 0.05)
    for progress in (0.0, 0.25, 0.5, 0.75, 1.0):
        a, _va = plain.evaluate(progress * 0.8)
        b, _vb = shifted.evaluate(progress * 0.8)
        assert abs(float(a[0] - b[0])) < 1e-9
        assert abs(float(a[2] - b[2])) < 1e-9


def test_swing_height_is_unchanged():
    start = np.array([0.0, -0.20, 0.07])
    landing = np.array([0.10, -0.04, 0.07])
    trajectory = SwingFootTrajectory(start, landing, 0.8, 0.05)
    heights = [float(trajectory.evaluate(t)[0][2]) for t in np.linspace(0.0, 0.8, 21)]
    assert max(heights) <= 0.07 + 0.05 + 1e-9
    assert max(heights) >= 0.07 + 0.05 - 1e-6


def test_foot_target_is_smoothly_ramped():
    tracker = CaptureStepTracker()
    tracker.reset("right")
    nominal = np.array([-0.20, -0.20])
    capture = np.array([0.0, 0.0])

    def reachable(_xy: np.ndarray) -> bool:
        return True

    previous = nominal.copy()
    for step in range(1, 20):
        progress = step / 40.0
        plan = tracker.plan(nominal, capture, progress, reachable, adapting=True, dt=0.02)
        jump = float(np.linalg.norm(plan.commanded_xy - previous))
        assert jump <= 0.50 * 0.02 + 1e-9
        previous = plan.commanded_xy.copy()
    assert abs(float(plan.correction_xy[1])) > 0.0


def test_foot_target_freezes_during_final_landing_phase():
    tracker = CaptureStepTracker()
    tracker.reset("right")
    nominal = np.array([0.10, -0.20])

    def reachable(_xy: np.ndarray) -> bool:
        return True

    first = tracker.plan(
        nominal, np.array([0.10, -0.05]), FREEZE_PROGRESS, reachable, adapting=True, dt=0.02
    )
    assert first.frozen
    later = tracker.plan(
        nominal, np.array([0.10, 0.30]), 0.95, reachable, adapting=True, dt=0.02
    )
    assert later.frozen
    assert np.allclose(later.commanded_xy, first.commanded_xy)


def test_touchdown_stops_target_adaptation():
    tracker = CaptureStepTracker()
    tracker.reset("right")
    nominal = np.array([0.10, -0.20])

    def reachable(_xy: np.ndarray) -> bool:
        return True

    held = tracker.plan(
        nominal, np.array([0.10, -0.08]), 0.40, reachable, adapting=False, dt=0.02
    )
    assert held.frozen
    moved = tracker.plan(
        nominal, np.array([0.10, 0.20]), 0.50, reachable, adapting=True, dt=0.02
    )
    assert np.allclose(moved.commanded_xy, held.commanded_xy)


def test_support_foot_switches_only_after_confirmed_touchdown():
    gait = WalkingStateMachine(
        WalkConfig(
            stand_duration=0.0,
            shift_duration=0.02,
            step_duration=0.40,
            touchdown_prep_duration=0.08,
            touchdown_confirm_ticks=3,
            capture_step=True,
        )
    )
    gait.start(
        np.array([0.0, 0.20, 0.07]),
        np.array([0.0, -0.20, 0.07]),
        0.95,
        n_steps=1,
        first_swing="right",
    )
    saw_swing = False
    for _ in range(40):
        sample = gait.update(
            0.02,
            True,
            foot_sample_ok=False,
            capture_landing=np.array([0.08, -0.10]),
            preparation_complete=True,
        )
        if sample.state in (WalkState.RIGHT_SWING, WalkState.TOUCHDOWN_PREP, WalkState.TOUCHDOWN):
            saw_swing = True
            assert sample.support_foot == "left"
        if sample.state is WalkState.TOUCHDOWN:
            break
    assert saw_swing
    assert gait.state is WalkState.TOUCHDOWN
    for _ in range(2):
        sample = gait.update(0.02, True, foot_sample_ok=True, preparation_complete=True)
        assert sample.state is WalkState.TOUCHDOWN
        assert sample.support_foot == "left"
    sample = gait.update(0.02, True, foot_sample_ok=True, preparation_complete=True)
    assert sample.state is WalkState.DOUBLE_SUPPORT_RECOVERY


def test_predicted_capture_point():
    com = np.array([0.05, 0.20, 0.90])
    vel = np.array([0.0, -0.20, 0.0])
    point, omega = predict_touchdown_capture(com, vel, 0.40)
    assert abs(omega - np.sqrt(9.81 / 0.90)) < 1e-12
    predicted_com_y = 0.20 + (-0.20) * 0.40
    assert abs(point[1] - (predicted_com_y + (-0.20) / omega)) < 1e-12
    assert abs(point[0] - 0.05) < 1e-12


def test_predicted_capture_point_follows_velocity():
    com = np.array([0.0, 0.20, 0.90])
    inward, _omega = predict_touchdown_capture(com, np.array([0.0, -0.30, 0.0]), 0.40)
    outward, _omega = predict_touchdown_capture(com, np.array([0.0, 0.30, 0.0]), 0.40)
    assert inward[1] < outward[1]
    faster, _omega = predict_touchdown_capture(com, np.array([0.0, -0.50, 0.0]), 0.40)
    assert faster[1] < inward[1]


def test_trackability_detects_impossible_landing():
    low, high = hip_track_interval(-0.40, 0.30, 1.5, -0.43, 0.43, 0.10)
    assert low >= -0.43 - 1e-12
    assert high <= -0.30 + 1e-9
    assert not lateral_is_trackable(0.10, -0.30, -0.20)


def test_trackability_detects_reachable_landing():
    low, high = hip_track_interval(-0.10, 0.50, 1.5, -0.43, 0.43, 0.15)
    assert low <= -0.12 <= high
    assert lateral_is_trackable(-0.12, -0.25, -0.05)


def test_target_projection_stays_within_joint_limits():
    low, high = hip_track_interval(0.40, 1.0, 1.5, -0.43, 0.43, 0.50)
    assert -0.43 - 1e-12 <= low <= high <= 0.43 + 1e-12
    assert project_to_interval(1.0, low, high) <= 0.43 + 1e-12
    assert project_to_interval(-2.0, -0.20, -0.05) == -0.20
    assert project_to_interval(1.0, -0.20, -0.05) == -0.05


def test_target_projection_is_smooth():
    tracker = CaptureStepTracker()
    tracker.reset("right")
    nominal = np.array([0.10, -0.20])
    predicted = np.array([0.10, 0.20])
    actual = np.array([0.10, -0.22])
    previous = actual[1]
    for _step in range(8):
        plan = tracker.plan_trackable(
            nominal,
            predicted,
            actual,
            (-0.24, -0.16),
            0.50,
            0.30,
            None,
            adapting=True,
            dt=0.02,
        )
        assert abs(float(plan.commanded_xy[1] - previous)) <= 0.50 * 0.02 + 1e-9
        previous = float(plan.commanded_xy[1])
        actual = plan.commanded_xy.copy()


def test_sagittal_swing_unchanged_by_lateral_override():
    start = np.array([0.0, -0.20, 0.07])
    landing = np.array([0.10, -0.20, 0.07])
    trajectory = SwingFootTrajectory(start, landing, 0.8, 0.05)
    sample, _velocity = trajectory.evaluate(0.4)
    overridden = sample.copy()
    overridden[1] = -0.08
    assert abs(float(overridden[0] - sample[0])) < 1e-12
    assert abs(float(overridden[2] - sample[2])) < 1e-12


def test_swing_height_unchanged_by_lateral_landing():
    start = np.array([0.0, -0.20, 0.07])
    shifted = np.array([0.10, -0.04, 0.07])
    nominal = np.array([0.10, -0.20, 0.07])
    moved = SwingFootTrajectory(start, shifted, 0.8, 0.05)
    plain = SwingFootTrajectory(start, nominal, 0.8, 0.05)
    for time in np.linspace(0.0, 0.8, 13):
        a, _va = moved.evaluate(time)
        b, _vb = plain.evaluate(time)
        assert abs(float(a[2] - b[2])) < 1e-9


def test_capture_valid_foot_is_detected():
    capture = np.array([0.10, -0.05])
    ankle = np.array([0.08, -0.05])
    assert landing_is_capture_valid(capture, ankle)
    assert abs(capture[1] - ankle[1]) <= SOLE_HALF_WIDTH


def test_capture_invalid_foot_is_detected():
    capture = np.array([0.05, 0.087])
    ankle = np.array([0.14, -0.043])
    assert not landing_is_capture_valid(capture, ankle)


def test_impossible_catch_triggers_recovery_instead_of_fall():
    building, recover = fall_or_recover(0.60, False)
    assert recover
    assert not building
    early_building, early_recover = fall_or_recover(0.30, False)
    assert early_building
    assert not early_recover
    body = CaptureBodyPrep()
    body.update("left", -0.155, 0.0, True, 0.0, 0.20)
    inward = body.update("left", -0.155, 0.0, True, 0.0, 0.20)
    assert inward > -0.155
    braked = body.update("left", -0.155, 0.0, False, 0.0, 0.02, braking=True)
    assert braked < inward
    assert braked >= -0.155 - 1e-9


def _capture_gait() -> WalkingStateMachine:
    gait = WalkingStateMachine(
        WalkConfig(stand_duration=0.0, shift_duration=0.02, step_duration=0.40, capture_step=True)
    )
    gait.start(
        np.array([0.0, 0.20, 0.07]),
        np.array([0.0, -0.20, 0.07]),
        0.95,
        n_steps=1,
        first_swing="right",
    )
    return gait


def test_prepare_step_exists():
    assert WalkState.PREPARE_STEP.value == "PREPARE_STEP"
    gait = _capture_gait()
    sample = gait.update(0.02, True)
    assert sample.state is WalkState.PREPARE_STEP


def test_both_feet_stay_planted_during_prepare():
    gait = _capture_gait()
    sample = gait.update(0.02, True)
    assert sample.state is WalkState.PREPARE_STEP
    assert sample.support_foot == "both"
    assert gait.loaded_foot is None
    assert gait.active_swing is None
    np.testing.assert_allclose(sample.left_foot[:2], [0.0, 0.20])
    np.testing.assert_allclose(sample.right_foot[:2], [0.0, -0.20])
    held = gait.update(0.02, True, preparation_complete=False)
    assert held.state is WalkState.PREPARE_STEP
    assert gait.active_swing is None


def test_prepare_uses_the_capture_point():
    point, omega = horizontal_capture(np.array([0.0, 0.20, 0.90]), np.array([0.0, -0.15, 0.0]))
    assert abs(omega - np.sqrt(9.81 / 0.90)) < 1e-12
    assert abs(point[1] - (0.20 + (-0.15) / omega)) < 1e-12
    still, _omega = horizontal_capture(np.array([0.0, 0.20, 0.90]), np.zeros(3))
    assert abs(still[1] - 0.20) < 1e-12
    assert abs(point[1] - still[1]) > 0.01


def test_feasible_capture_region_comes_from_foot_reach():
    def accepts(y: float) -> bool:
        return -0.35 <= float(y) <= -0.05

    foot_min, foot_max = search_reachable_y(-0.20, accepts)
    assert abs(foot_min - (-0.35)) < 1e-3
    assert abs(foot_max - (-0.05)) < 1e-3
    low, high = feasible_capture_interval(foot_min, foot_max, margin=0.01)
    assert abs(low - (foot_min - SOLE_HALF_WIDTH + 0.01)) < 1e-9
    assert abs(high - (foot_max + SOLE_HALF_WIDTH - 0.01)) < 1e-9
    assert abs(preparation_target_y(0.20, low, high) - high) < 1e-12
    assert low <= -0.20 <= high
    assert abs(preparation_target_y(-0.20, low, high) - (-0.20)) < 1e-12


def test_prepare_does_not_finish_while_capture_is_outside():
    foot_min, foot_max = -0.30, -0.10
    low, high = feasible_capture_interval(foot_min, foot_max, margin=0.01)
    assert not capture_is_prepared(0.16, low, high)
    gait = _capture_gait()
    gait.update(0.02, True)
    held = gait.update(0.02, True, preparation_complete=False)
    assert held.state is WalkState.PREPARE_STEP
    assert gait.active_swing is None


def test_prepare_finishes_once_capture_is_inside():
    low, high = feasible_capture_interval(-0.30, -0.10, margin=0.01)
    assert capture_is_prepared(0.5 * (low + high), low, high)
    gait = _capture_gait()
    gait.update(0.02, True)
    released = gait.update(0.02, True, preparation_complete=True)
    assert released.state is WalkState.RIGHT_SWING


def test_swing_does_not_start_before_preparation():
    gait = _capture_gait()
    for _ in range(5):
        sample = gait.update(0.02, True, preparation_complete=False)
        assert sample.state is not WalkState.RIGHT_SWING
        assert gait.active_swing is None
    assert gait.state is WalkState.PREPARE_STEP


def test_candidate_foot_stays_inside_physical_reach():
    foot_min, foot_max = -0.30, -0.10
    for capture_y in (-0.50, -0.20, 0.0, 0.40):
        ankle = candidate_ankle_y(capture_y, foot_min, foot_max)
        assert foot_min - 1e-9 <= ankle <= foot_max + 1e-9
    covered = candidate_ankle_y(-0.20, foot_min, foot_max)
    assert abs(covered - (-0.20)) < 1e-9


def _foot_from_hip(q: float, q0: float = -0.02, y0: float = -0.21, slope: float = 0.75) -> float:
    return y0 + slope * (float(q) - q0)


def test_static_reach_is_distinct_from_dynamic_reach():
    static_low, static_high = feasible_capture_interval(-0.457, 0.112, margin=0.01)
    assert capture_is_prepared(0.015, static_low, static_high)
    q_low, q_high = dynamic_q_interval(-0.02, 0.80, 0.03, 0.02, -0.43, 0.43, torque_lead())
    foot_low, foot_high = dynamic_foot_interval(
        _foot_from_hip(q_low), _foot_from_hip(q_high), -0.21
    )
    assert foot_high - foot_low < (0.112 - (-0.457))
    assert not dynamic_catchable(0.015, foot_low, foot_high, margin=0.01)


def test_dynamic_reach_starts_from_current_q():
    low, high = dynamic_q_interval(-0.32, 0.80, 0.03, 0.02, -0.43, 0.43, torque_lead())
    assert low >= -0.43 - 1e-9
    assert high < 0.0
    assert abs(low - (-0.42)) < 1e-9
    assert abs(high - (-0.22)) < 1e-9
    home_low, home_high = dynamic_q_interval(0.0, 0.80, 0.03, 0.02, -0.43, 0.43, torque_lead())
    assert abs(home_low - low) > 0.1


def test_dynamic_reach_respects_remaining_time_and_ramp():
    short_low, short_high = dynamic_q_interval(-0.02, 0.02, 0.03, 0.02, -0.43, 0.43, torque_lead())
    long_low, long_high = dynamic_q_interval(-0.02, 0.80, 0.03, 0.02, -0.43, 0.43, torque_lead())
    assert (short_high - short_low) < (long_high - long_low) - 1e-9
    slow_low, slow_high = dynamic_q_interval(-0.02, 0.02, 0.01, 0.02, -0.43, 0.43, torque_lead())
    assert (slow_high - slow_low) < (short_high - short_low) - 1e-9


def test_dynamic_reach_rejects_a_large_move_without_time():
    low, high = dynamic_q_interval(-0.02, 0.05, 0.03, 0.02, -0.43, 0.43, torque_lead())
    foot_low, foot_high = dynamic_foot_interval(_foot_from_hip(low), _foot_from_hip(high), -0.21)
    target = -0.21 + 0.22
    assert not (foot_low - 1e-9 <= target <= foot_high + 1e-9)
    assert not dynamic_catchable(0.015, foot_low, foot_high, margin=0.01)


def test_dynamic_reach_accepts_a_small_move():
    low, high = dynamic_q_interval(-0.02, 0.80, 0.03, 0.02, -0.43, 0.43, torque_lead())
    foot_low, foot_high = dynamic_foot_interval(_foot_from_hip(low), _foot_from_hip(high), -0.21)
    target = -0.17
    assert foot_low - 1e-9 <= target <= foot_high + 1e-9
    assert dynamic_catchable(target, foot_low, foot_high, margin=0.01)


def test_prepare_does_not_complete_on_static_reach_alone():
    q_low, q_high = dynamic_q_interval(-0.02, 0.80, 0.03, 0.02, -0.43, 0.43, torque_lead())
    foot_low, foot_high = dynamic_foot_interval(
        _foot_from_hip(q_low), _foot_from_hip(q_high), -0.21
    )
    assert not dynamic_catchable(0.015, foot_low, foot_high, margin=0.01)
    gait = _capture_gait()
    gait.update(0.02, True)
    held = gait.update(0.02, True, preparation_complete=False)
    assert held.state is WalkState.PREPARE_STEP


def test_prepare_waits_for_dynamic_catchability():
    gait = _capture_gait()
    gait.update(0.02, True)
    for _ in range(4):
        sample = gait.update(0.02, True, preparation_complete=False)
        assert sample.state is WalkState.PREPARE_STEP
    released = gait.update(0.02, True, preparation_complete=True)
    assert released.state is WalkState.RIGHT_SWING


def test_foot_lift_requires_airborne_height():
    assert not foot_is_airborne(0.078, 0.074)
    assert foot_is_airborne(0.10, 0.07)
    gait = _capture_gait()
    gait.update(0.02, True, preparation_complete=True)
    sample = gait.update(0.02, True, preparation_complete=True)
    assert sample.state is WalkState.RIGHT_SWING
    assert not foot_is_airborne(0.074, 0.074)


def test_support_transfer_rejects_force_alone_and_a_falling_impact():
    falling = dict(
        contact=True,
        new_force=457.0,
        old_force=0.0,
        capture_inside=True,
        vy=-0.89,
        vx=-0.40,
        roll=0.67,
        pitch=-0.37,
    )
    assert not support_transfer_sample_ok(**falling)
    force_only = dict(falling)
    force_only.update(vy=0.0, vx=0.0, roll=0.0, pitch=0.0, capture_inside=False)
    assert not support_transfer_sample_ok(**force_only)
    stable = dict(falling)
    stable.update(vy=0.01, vx=0.01, roll=0.05, pitch=0.05, new_force=80.0, old_force=20.0)
    assert support_transfer_sample_ok(**stable)


def test_support_transfer_requires_a_stability_dwell():
    gait = WalkingStateMachine(
        WalkConfig(
            stand_duration=0.0,
            shift_duration=0.02,
            step_duration=0.08,
            touchdown_prep_duration=0.0,
            touchdown_confirm_ticks=1,
            capture_step=True,
            stabilization_dwell=0.04,
        )
    )
    gait.start(
        np.array([0.0, 0.20, 0.07]),
        np.array([0.0, -0.20, 0.07]),
        0.95,
        n_steps=1,
        first_swing="right",
    )
    gait.update(0.02, True, preparation_complete=True)
    for _ in range(8):
        gait.update(0.02, True, preparation_complete=True, support_transfer_stable=False)
    assert gait.state is WalkState.TOUCHDOWN
    entered = gait.update(0.02, True, foot_sample_ok=True, support_transfer_stable=False)
    assert entered.state is WalkState.SUPPORT_TRANSFER
    assert entered.support_foot == "left"
    still = gait.update(0.02, True, support_transfer_stable=False)
    assert still.state is WalkState.SUPPORT_TRANSFER
    assert gait._support_transfer_accepted is False
    gait.update(0.02, True, support_transfer_stable=True)
    finished = gait.update(0.02, True, support_transfer_stable=True)
    assert finished.state is WalkState.DOUBLE_SUPPORT_RECOVERY
    assert gait._support_transfer_accepted is True


def test_prepare_target_is_not_zero_for_a_side_foot():
    low, high = catchable_capture_interval(-0.285, -0.141, sole_half_width=SOLE_HALF_WIDTH, margin=0.01)
    target = select_capture_target(0.0, low, high)
    assert abs(target) > 0.05
    assert low - 1e-9 <= target <= high + 1e-9
    assert target != 0.0


def test_catchable_region_uses_the_sole_width():
    low, high = catchable_capture_interval(-0.285, -0.141, sole_half_width=0.020, margin=0.01)
    assert abs(low - (-0.285 - 0.020 + 0.01)) < 1e-12
    assert abs(high - (-0.141 + 0.020 - 0.01)) < 1e-12
    assert high > low


def test_prepare_target_stays_inside_the_catchable_interval():
    low, high = catchable_capture_interval(-0.285, -0.141, margin=0.01)
    for capture in (-1.0, -0.20, 0.0, 0.40):
        target = select_capture_target(capture, low, high)
        assert low - 1e-9 <= target <= high + 1e-9


def test_prepare_does_not_move_when_the_capture_is_already_inside():
    low, high = catchable_capture_interval(-0.285, -0.141, margin=0.01)
    current = 0.5 * (low + high)
    target = select_capture_target(current, low, high)
    assert abs(target - current) < 1e-12
    assert prepare_lateral_velocity(current, target, 3.2, max_lateral=0.20, hold=False) == 0.0


def test_prepare_moves_toward_the_nearest_feasible_capture():
    low, high = catchable_capture_interval(-0.285, -0.141, margin=0.01)
    target = select_capture_target(0.0, low, high)
    assert abs(target - high) < 1e-12
    velocity = prepare_lateral_velocity(0.0, target, 3.2, max_lateral=0.20, hold=False)
    assert velocity < 0.0
    assert abs(velocity) <= 0.20 + 1e-12


def test_prepare_target_follows_a_moving_reachable_interval():
    first_low, first_high = catchable_capture_interval(-0.285, -0.141, margin=0.01)
    first = select_capture_target(0.0, first_low, first_high)
    second_low, second_high = catchable_capture_interval(-0.335, -0.191, margin=0.01)
    second = select_capture_target(0.0, second_low, second_high)
    assert second < first
    assert second_low - 1e-9 <= second <= second_high + 1e-9


def test_a_worsening_gap_drops_the_stale_push():
    count = 0
    gap = 0.10
    for _ in range(PREPARE_WORSEN_TICKS):
        new_gap = gap + 0.01
        count = update_worsen_count(gap, new_gap, count)
        gap = new_gap
    assert count >= PREPARE_WORSEN_TICKS
    low, high = catchable_capture_interval(-0.335, -0.191, margin=0.01)
    target = select_capture_target(-0.02, low, high)
    assert low - 1e-9 <= target <= high + 1e-9
    held = prepare_lateral_velocity(-0.02, target, 3.2, max_lateral=0.20, hold=True)
    assert held == 0.0
    assert update_worsen_count(gap, gap, count) == 0


def test_static_ik_alone_does_not_permit_liftoff():
    assert not liftoff_permitted(
        predicted_inside=False, speed_ok=True, dwell_met=True, static_reachable=True
    )
    assert liftoff_permitted(
        predicted_inside=True, speed_ok=True, dwell_met=True, static_reachable=False
    )


def test_liftoff_requires_the_predicted_capture_inside_the_margin():
    low, high = catchable_capture_interval(-0.285, -0.141, margin=0.01)
    inside = interval_distance(-0.20, low, high) == 0.0
    outside = interval_distance(0.0, low, high) == 0.0
    assert inside
    assert not outside
    assert not liftoff_permitted(predicted_inside=outside, speed_ok=True, dwell_met=True)
    assert not liftoff_permitted(predicted_inside=inside, speed_ok=False, dwell_met=True)
    assert liftoff_permitted(predicted_inside=inside, speed_ok=True, dwell_met=True)


_WEIGHT_SHIFT_HIP = -0.155


def test_prepare_hip_moves_toward_a_lower_capture_target():
    command = prepare_hip_roll_command(0.0, 0.0, -0.131, 0.0, ramp=0.03)
    assert command > 0.0
    assert command == 0.03
    settled = 0.0
    for _ in range(20):
        settled = prepare_hip_roll_command(0.0, 0.0, -0.131, settled, ramp=0.03)
    assert abs(settled - 0.15) < 1e-9


def test_prepare_hip_is_not_the_fixed_weight_shift():
    command = prepare_hip_roll_command(0.0, 0.0, -0.131, _WEIGHT_SHIFT_HIP, ramp=1.0)
    assert abs(command - _WEIGHT_SHIFT_HIP) > 0.2
    assert command > 0.0


def test_final_q_des_keeps_the_prepare_hip_command():
    from controllers.support_transition import apply_stance_hip_roll

    names = ["left_hip_roll", "right_hip_roll"]
    prepare = prepare_hip_roll_command(0.0, 0.0, -0.131, 0.0, ramp=1.0)
    q_des = np.array([0.0, 0.0])
    q_des = apply_stance_hip_roll(
        q_des, names, "left", prepare, np.array([-0.43, -0.43]), np.array([0.43, 0.43])
    )
    assert abs(float(q_des[0]) - prepare) < 1e-9


def test_prepare_target_produces_lateral_com_velocity():
    velocity = prepare_lateral_velocity(0.0, -0.131, 3.2, max_lateral=0.20, hold=False)
    assert velocity < 0.0
    assert abs(velocity) > 1e-3


def test_prepare_hip_wins_when_mpc_disagrees():
    from controllers.support_transition import apply_stance_hip_roll

    names = ["left_hip_roll"]
    prepare = prepare_hip_roll_command(0.0, 0.0, -0.131, 0.0, ramp=0.03)
    mpc_q = np.array([_WEIGHT_SHIFT_HIP])
    final = apply_stance_hip_roll(
        mpc_q, names, "left", prepare, np.array([-0.43]), np.array([0.43])
    )
    assert abs(float(final[0]) - prepare) < 1e-9
    assert abs(float(final[0]) - _WEIGHT_SHIFT_HIP) > 0.1


def test_prepare_cbf_intervention_is_logged():
    trace = prepare_hip_trace(
        mpc_q=_WEIGHT_SHIFT_HIP,
        prepare_q=0.03,
        after_override=_WEIGHT_SHIFT_HIP,
        before_pd=0.03,
        tau_nominal=12.0,
        tau_safe=4.0,
    )
    assert trace["q_des_before_pd"] == 0.03
    assert trace["q_des_prepare_hip_roll"] == trace["q_des_before_pd"]
    assert trace["q_des_mpc_hip_roll"] != trace["q_des_before_pd"]
    assert trace["cbf_intervention_hip_roll"] == 8.0
    assert "cbf_intervention_hip_roll" in trace


def test_walking_capture_offset_stays_at_015():
    from controllers.mpc_controller import lateral_capture_hip_offset

    offset = lateral_capture_hip_offset(0.0, -0.5)
    assert abs(offset) == HIP_OFFSET_LIMIT
    assert abs(HIP_OFFSET_LIMIT - 0.15) < 1e-12
    assert abs(offset) < PREPARE_CAPTURE_HIP_LIMIT


def test_prepare_hip_limit_is_025():
    command = prepare_hip_roll_command(
        0.0, 0.0, -0.131, 0.0, ramp=1.0, limit=PREPARE_CAPTURE_HIP_LIMIT
    )
    assert abs(command - PREPARE_CAPTURE_HIP_LIMIT) < 1e-9
    assert abs(PREPARE_CAPTURE_HIP_LIMIT - 0.25) < 1e-12
    _raw, clipped, error = prepare_hip_targets(0.0, 0.0, -0.131, limit=PREPARE_CAPTURE_HIP_LIMIT)
    assert abs(clipped - 0.25) < 1e-9
    assert error > 0.0


def test_prepare_hip_does_not_jump_from_015_to_025():
    command = prepare_hip_roll_command(
        0.0, 0.0, -0.131, 0.15, ramp=0.03, limit=PREPARE_CAPTURE_HIP_LIMIT
    )
    assert abs(command - 0.18) < 1e-9
    assert command < PREPARE_CAPTURE_HIP_LIMIT


def test_prepare_hip_tracks_the_capture_error():
    far = prepare_hip_roll_command(
        0.0, 0.0, -0.131, 0.0, ramp=1.0, limit=PREPARE_CAPTURE_HIP_LIMIT
    )
    near = prepare_hip_roll_command(
        0.0, -0.120, -0.131, 0.0, ramp=1.0, limit=PREPARE_CAPTURE_HIP_LIMIT
    )
    assert near < far
    assert abs(near - 12.0 * 0.011) < 1e-6
    reversed_command = prepare_hip_roll_command(
        0.0, -0.200, -0.131, 0.20, ramp=0.03, limit=PREPARE_CAPTURE_HIP_LIMIT
    )
    assert reversed_command < 0.20


def test_measured_hip_past_the_command_adds_no_extra_offset():
    previous_command = 0.15
    measured = 0.20
    command = prepare_hip_roll_command(
        0.0,
        0.0,
        -0.131,
        previous_command,
        ramp=0.03,
        limit=PREPARE_CAPTURE_HIP_LIMIT,
    )
    assert abs(command - 0.18) < 1e-9
    assert command < measured


def test_prepare_final_q_des_is_the_pd_target():
    from controllers.support_transition import apply_stance_hip_roll

    prepare = prepare_hip_roll_command(
        0.0, 0.0, -0.131, 0.15, ramp=0.03, limit=PREPARE_CAPTURE_HIP_LIMIT
    )
    final = apply_stance_hip_roll(
        np.array([-0.15]),
        ["left_hip_roll"],
        "left",
        prepare,
        np.array([-0.43]),
        np.array([0.43]),
    )
    assert abs(float(final[0]) - prepare) < 1e-9
    assert abs(float(final[0]) - 0.18) < 1e-9


def test_cbf_torque_box_is_unchanged():
    from safety.cbf_filter import CBFConfig

    assert CBFConfig().max_torque_deviation == 40.0


def test_reachable_interval_uses_the_leg_ik():
    import mujoco

    from controllers.leg_ik import LegIK

    model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "unitree_h1" / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    ik = LegIK(model)
    ankle = np.asarray(
        data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_link")],
        dtype=float,
    )

    def accepts(y: float) -> bool:
        target = np.array([float(ankle[0]), float(y), float(ankle[2])])
        return bool(ik.solve_lateral(target, data, side="right", max_lateral=1.5).reachable)

    foot_min, foot_max = search_reachable_y(float(ankle[1]), accepts)
    assert foot_min < float(ankle[1]) < foot_max
    assert accepts(foot_min) and accepts(foot_max)
    assert not accepts(foot_min - 0.05)
    low, high = feasible_capture_interval(foot_min, foot_max, margin=0.01)
    assert high > low


def main() -> None:
    test_capture_point_calculation()
    test_capture_point_responds_to_com_velocity()
    test_capture_target_is_bounded()
    test_unreachable_capture_target_is_projected()
    test_lateral_target_uses_existing_3d_ik()
    test_sagittal_trajectory_unchanged_for_lateral_correction_only()
    test_swing_height_is_unchanged()
    test_foot_target_is_smoothly_ramped()
    test_foot_target_freezes_during_final_landing_phase()
    test_touchdown_stops_target_adaptation()
    test_support_foot_switches_only_after_confirmed_touchdown()
    test_predicted_capture_point()
    test_predicted_capture_point_follows_velocity()
    test_trackability_detects_impossible_landing()
    test_trackability_detects_reachable_landing()
    test_target_projection_stays_within_joint_limits()
    test_target_projection_is_smooth()
    test_sagittal_swing_unchanged_by_lateral_override()
    test_swing_height_unchanged_by_lateral_landing()
    test_capture_valid_foot_is_detected()
    test_capture_invalid_foot_is_detected()
    test_impossible_catch_triggers_recovery_instead_of_fall()
    test_prepare_step_exists()
    test_both_feet_stay_planted_during_prepare()
    test_prepare_uses_the_capture_point()
    test_feasible_capture_region_comes_from_foot_reach()
    test_prepare_does_not_finish_while_capture_is_outside()
    test_prepare_finishes_once_capture_is_inside()
    test_swing_does_not_start_before_preparation()
    test_candidate_foot_stays_inside_physical_reach()
    test_reachable_interval_uses_the_leg_ik()
    test_static_reach_is_distinct_from_dynamic_reach()
    test_dynamic_reach_starts_from_current_q()
    test_dynamic_reach_respects_remaining_time_and_ramp()
    test_dynamic_reach_rejects_a_large_move_without_time()
    test_dynamic_reach_accepts_a_small_move()
    test_prepare_does_not_complete_on_static_reach_alone()
    test_prepare_waits_for_dynamic_catchability()
    test_foot_lift_requires_airborne_height()
    test_support_transfer_rejects_force_alone_and_a_falling_impact()
    test_support_transfer_requires_a_stability_dwell()
    test_prepare_target_is_not_zero_for_a_side_foot()
    test_catchable_region_uses_the_sole_width()
    test_prepare_target_stays_inside_the_catchable_interval()
    test_prepare_does_not_move_when_the_capture_is_already_inside()
    test_prepare_moves_toward_the_nearest_feasible_capture()
    test_prepare_target_follows_a_moving_reachable_interval()
    test_a_worsening_gap_drops_the_stale_push()
    test_static_ik_alone_does_not_permit_liftoff()
    test_liftoff_requires_the_predicted_capture_inside_the_margin()
    test_prepare_hip_moves_toward_a_lower_capture_target()
    test_prepare_hip_is_not_the_fixed_weight_shift()
    test_final_q_des_keeps_the_prepare_hip_command()
    test_prepare_target_produces_lateral_com_velocity()
    test_prepare_hip_wins_when_mpc_disagrees()
    test_prepare_cbf_intervention_is_logged()
    test_walking_capture_offset_stays_at_015()
    test_prepare_hip_limit_is_025()
    test_prepare_hip_does_not_jump_from_015_to_025()
    test_prepare_hip_tracks_the_capture_error()
    test_measured_hip_past_the_command_adds_no_extra_offset()
    test_prepare_final_q_des_is_the_pd_target()
    test_cbf_torque_box_is_unchanged()
    print("capture step tests passed")


if __name__ == "__main__":
    main()
