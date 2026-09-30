"""Checks for the standing stack and the lateral weight shift.

The shift test does not lower the swing gate. It checks that the centre-of-mass
reference moves toward the measured stance sole, and that the swing foot stays
down while the centre of mass is still outside that sole.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers.footstep_planner import FootstepPlanner, SwingFootTrajectory
from controllers.lateral_shift import LateralWeightShift, support_distance
from controllers.leg_ik import LegIK
from controllers.walk_state_machine import WalkConfig, WalkingStateMachine, WalkState
from main_demo import (
    _ankle_pose,
    build_stack,
    control_step,
    sole_corners,
    stand_reference,
    walk_tick,
)

MODEL = ROOT / "models" / "unitree_h1" / "scene.xml"


def _args() -> argparse.Namespace:
    return argparse.Namespace(model_path=MODEL, control_dt=0.02, horizon=20, seed=0)


def check_planners() -> None:
    planner = FootstepPlanner()
    steps = planner.plan(np.zeros(3), np.zeros(3), n_steps=4, first_foot="left")
    feet = [step.foot for step in steps]
    landings = [round(float(step.target[0]), 5) for step in steps]
    assert feet == ["left", "right", "left", "right"], feet
    assert landings == [0.1, 0.2, 0.3, 0.4], landings
    position, velocity = SwingFootTrajectory(
        np.zeros(3), np.array([0.2, 0.0, 0.0]), 0.8, 0.05
    ).evaluate(0.4)
    assert abs(float(position[2]) - 0.05) < 1e-6
    assert float(np.linalg.norm(velocity)) > 1e-3

    gait = WalkingStateMachine(WalkConfig(stand_duration=0.0, shift_duration=0.2))
    gait.start(np.array([0.0, 0.2, 0.0]), np.array([0.0, -0.2, 0.0]), 0.95, n_steps=1, first_swing="right")
    saw_shift = False
    saw_swing = False
    for _ in range(40):
        sample = gait.update(0.02, True)
        saw_shift = saw_shift or sample.state is WalkState.SHIFT_LEFT
        saw_swing = saw_swing or sample.state is WalkState.RIGHT_SWING
    assert saw_shift and saw_swing


def check_leg_ik(env) -> None:
    ik = LegIK(env.model)
    for side in ("left", "right"):
        ankle, _rotation = _ankle_pose(env, side)
        joints = ik.solve(ankle, env, side=side)
        assert joints.reachable
        assert abs(joints.hip_pitch + 0.4) < 1e-3
        assert abs(joints.knee - 0.8) < 1e-3
        assert abs(joints.ankle_pitch + 0.4) < 1e-3


def check_stand(env, mpc, cbf, pd) -> None:
    env.reset(seed=0)
    reference = stand_reference(env, env.get_state())
    min_z = 10.0
    for _ in range(100):
        row = control_step(env, mpc, cbf, pd, reference)
        min_z = min(min_z, float(row["com"][2]))
    assert min_z > 0.85, min_z
    assert abs(float(row["com"][0]) - float(np.asarray(reference["com"]).reshape(-1)[0])) < 0.05


def check_walk_stays_up(env, mpc, cbf, pd) -> None:
    env.reset(seed=0)
    cbf.config.margin = 0.0
    state = env.get_state()
    gait = WalkingStateMachine(WalkConfig(stand_duration=1.0, shift_duration=1.5))
    gait.start(
        _ankle_pose(env, "left")[0],
        _ankle_pose(env, "right")[0],
        float(state["com"][2]),
        n_steps=1,
        first_swing="right",
    )
    rows: list[dict] = []
    min_z = 10.0
    for _ in range(200):
        row = walk_tick(env, mpc, cbf, pd, gait, LegIK(env.model), state["torso_rotation"], rows)
        rows.append(row)
        min_z = min(min_z, float(row["com"][2]))
    assert min_z > 0.85, min_z
    assert any(row["walk_state"] == "SHIFT_LEFT" for row in rows)
    assert all(row["swing_side"] == "" for row in rows)


def check_target_moves_toward_stance_foot(env, mpc, cbf, pd) -> None:
    """The reference approaches the measured sole, and the swing stays down."""
    env.reset(seed=0)
    cbf.config.margin = 0.0
    state = env.get_state()
    gait = WalkingStateMachine(WalkConfig(stand_duration=1.0, shift_duration=2.0))
    gait.start(
        _ankle_pose(env, "left")[0],
        _ankle_pose(env, "right")[0],
        float(state["com"][2]),
        n_steps=1,
        first_swing="right",
    )
    rows: list[dict] = []
    min_z = 10.0
    for _ in range(350):
        row = walk_tick(env, mpc, cbf, pd, gait, LegIK(env.model), state["torso_rotation"], rows)
        rows.append(row)
        min_z = min(min_z, float(row["com"][2]))
    shift_rows = [row for row in rows if row["walk_state"] == "SHIFT_LEFT"]
    assert shift_rows, "never entered the weight shift"
    start_y = float(shift_rows[0]["com_reference"][1])
    end_y = float(shift_rows[-1]["com_reference"][1])
    target_y = float(gait._lateral_shift.target_xy[1])
    assert end_y - start_y > 0.05, (start_y, end_y, target_y)
    assert abs(target_y - end_y) < abs(target_y - start_y)
    assert all(row["swing_side"] == "" for row in rows)
    assert min_z > 0.85, min_z
    logged = np.asarray(shift_rows[-1]["stance_foot_xy"], dtype=float)
    assert abs(float(logged[1]) - target_y) < 1e-6
    gap = support_distance(shift_rows[-1]["com"][:2], sole_corners(env, "left"))
    assert abs(gap - float(shift_rows[-1]["support_distance"])) < 1e-3


def check_solver_uses_measured_feet(env, mpc) -> None:
    left, _left_rot = _ankle_pose(env, "left")
    right, _right_rot = _ankle_pose(env, "right")
    home = {name: float(q) for name, q in zip(mpc._joint_names, mpc._home_joint_qpos)}
    measured = sole_corners(env, "left").copy()
    measured[:, 1] += 0.03
    shift = LateralWeightShift(env.model)
    shift.begin("left", left, right, env.get_state()["com"], measured, home)
    assert abs(float(shift.target_xy[1]) - float(measured.mean(axis=0)[1])) < 1e-9
    real = sole_corners(env, "left")
    shift.begin("left", left, right, env.get_state()["com"], real, home)
    assert shift.solved
    assert abs(float(shift.target_xy[1]) - float(real.mean(axis=0)[1])) < 1e-9


def main() -> None:
    checks = []
    env, mpc, cbf, pd = build_stack(_args())
    env.reset(seed=0)
    try:
        for name, fn in (
            ("planners", lambda: check_planners()),
            ("leg_ik", lambda: check_leg_ik(env)),
            ("solver", lambda: check_solver_uses_measured_feet(env, mpc)),
            ("stand", lambda: check_stand(env, mpc, cbf, pd)),
            ("walk_stays_up", lambda: check_walk_stays_up(env, mpc, cbf, pd)),
            ("target_moves", lambda: check_target_moves_toward_stance_foot(env, mpc, cbf, pd)),
        ):
            fn()
            checks.append(name)
            print("PASS", name)
    finally:
        env.close()
    print(f"{len(checks)} passed")


if __name__ == "__main__":
    main()
