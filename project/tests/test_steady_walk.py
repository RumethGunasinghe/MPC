"""Touchdown, load transfer and the slow walking cycle."""

import numpy as np

from controllers.steady_walk import (
    SteadyWalk,
    SupportLatch,
    clip_landing,
    landing_support_ok,
    point_on_sole,
    slew_xy,
)


def _walker() -> SteadyWalk:
    names = [
        "left_hip_roll",
        "left_hip_pitch",
        "left_knee",
        "left_ankle",
        "right_hip_roll",
        "right_hip_pitch",
        "right_knee",
        "right_ankle",
    ]
    home = np.array([0.0, -0.4, 0.8, -0.4, 0.0, -0.4, 0.8, -0.4])
    return SteadyWalk(names, home, np.full(8, -2.0), np.full(8, 2.0), n_steps=1)


def _identity():
    return np.eye(3)


def test_contact_alone_is_not_support():
    ok = landing_support_ok(
        new_contact=True,
        old_contact=True,
        new_force=5.0,
        old_force=400.0,
        capture_inside=True,
        vx=0.0,
        vy=0.0,
        roll=0.0,
        pitch=0.0,
        roll_rate=0.0,
        pitch_rate=0.0,
    )
    assert ok is False


def test_loaded_quiet_landing_is_support():
    ok = landing_support_ok(
        new_contact=True,
        old_contact=True,
        new_force=120.0,
        old_force=200.0,
        capture_inside=True,
        vx=0.02,
        vy=-0.02,
        roll=0.05,
        pitch=-0.04,
        roll_rate=0.1,
        pitch_rate=-0.1,
    )
    assert ok is True


def test_support_latch_does_not_chatter():
    latch = SupportLatch()
    assert latch.update(10.0, True, 0.02) == "stance"
    assert latch.update(80.0, True, 0.02) == "both"
    assert latch.update(10.0, True, 0.02) == "both"
    assert latch.update(10.0, False, 0.02) == "both"
    assert latch.update(0.0, False, 0.10) == "stance"


def test_com_reference_cannot_jump():
    start = np.array([0.0, 0.20])
    goal = np.array([0.10, -0.20])
    moved = slew_xy(start, goal, speed=0.04, dt=0.02)
    assert float(np.linalg.norm(moved - start)) <= 0.04 * 0.02 + 1e-9
    assert float(np.linalg.norm(moved - goal)) < float(np.linalg.norm(start - goal))


def test_footstep_stays_inside_the_dynamic_window():
    start = np.array([0.04, -0.20, 0.07])
    landing = clip_landing(start, step_length=0.40, capture_y=-0.42, max_step=0.10, max_lateral=0.05)
    assert landing[0] - start[0] <= 0.10 + 1e-9
    assert abs(landing[1] - start[1]) <= 0.05 + 1e-9


def test_capture_uses_the_real_sole():
    ankle = np.array([0.0, 0.20, 0.07])
    rotation = _identity()
    assert point_on_sole(np.array([0.05, 0.20]), ankle, rotation)
    assert not point_on_sole(np.array([0.05, 0.30]), ankle, rotation)


def test_joint_commands_stay_inside_the_actuator_range():
    walker = _walker()
    walker.low[:] = -0.2
    walker.high[:] = 0.2
    walker._ramp_to({"left_hip_roll": 1.5, "right_knee": -3.0}, rate=10.0, dt=1.0)
    assert walker.q[0] <= 0.2
    assert walker.q[6] >= -0.2


def test_touchdown_holds_the_prelanding_pose():
    walker = _walker()
    walker.phase = "touchdown"
    walker.q[0] = -0.25
    walker.hip_hold = -0.25
    walker.sagittal_ref = 0.05
    before = walker.q.copy()
    walker.step_inputs(
        dt=0.02,
        com=np.array([0.05, 0.20, 0.90]),
        vel=np.zeros(3),
        capture=np.array([0.05, 0.20]),
        roll=0.0,
        pitch=0.0,
        roll_rate=0.0,
        pitch_rate=0.0,
        left_ankle=np.array([0.04, 0.20, 0.07]),
        right_ankle=np.array([0.04, -0.20, 0.07]),
        left_rotation=_identity(),
        right_rotation=_identity(),
        left_force=200.0,
        right_force=20.0,
        left_contact=True,
        right_contact=True,
        solved_pose={"left_hip_roll": 0.4, "right_hip_roll": 0.4},
    )
    assert walker.phase == "touchdown"
    assert walker.q[0] == -0.25
    assert abs(float(walker.q[4])) <= 0.02


def test_state_machine_reaches_swing_only_after_a_shift_dwell():
    walker = _walker()
    left = np.array([0.04, 0.20, 0.07])
    right = np.array([0.04, -0.20, 0.07])
    rotation = _identity()
    for _ in range(60):
        walker.step_inputs(
            dt=0.02,
            com=np.array([0.05, 0.0, 0.90]),
            vel=np.zeros(3),
            capture=np.array([0.05, 0.0]),
            roll=0.0,
            pitch=0.0,
            roll_rate=0.0,
            pitch_rate=0.0,
            left_ankle=left,
            right_ankle=right,
            left_rotation=rotation,
            right_rotation=rotation,
            left_force=200.0,
            right_force=200.0,
            left_contact=True,
            right_contact=True,
            solved_pose=None,
        )
    assert walker.phase == "shift"
    pose = {
        "left_hip_roll": -0.2,
        "left_hip_pitch": -0.4,
        "left_knee": 0.8,
        "right_hip_roll": 0.1,
        "right_hip_pitch": -0.4,
        "right_knee": 0.8,
    }
    for _ in range(40):
        walker.step_inputs(
            dt=0.02,
            com=np.array([0.05, 0.20, 0.90]),
            vel=np.array([0.0, 0.0, 0.0]),
            capture=np.array([0.05, 0.20]),
            roll=0.0,
            pitch=0.0,
            roll_rate=0.0,
            pitch_rate=0.0,
            left_ankle=left,
            right_ankle=right,
            left_rotation=rotation,
            right_rotation=rotation,
            left_force=250.0,
            right_force=40.0,
            left_contact=True,
            right_contact=True,
            solved_pose=pose,
        )
    assert walker.walk_state == "RIGHT_SWING"


def test_cbf_support_stays_on_the_stance_sole_until_the_new_foot_is_loaded():
    latch = SupportLatch()
    assert latch.update(400.0, False, 0.02) == "stance"
    assert latch.update(70.0, True, 0.02) == "both"
