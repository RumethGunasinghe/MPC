"""Demonstration: MPC, PD and a CBF torque filter on the Unitree H1.

The control stack on every tick is

    state -> WholeBodyMPC -> PDController -> CBFFilter -> joint torques -> MuJoCo

The MPC holds the torso attitude recorded at reset. In walking mode a state
machine shifts the centre of mass onto the measured centre of one sole before
the other foot swings forward. The shift is a blend from the home posture to
the leg pose that reaches that sole with both feet planted. The swing leg
tracks its foot with inverse kinematics, and only after the centre of mass is
inside the stance sole. The CBF keeps the centre of mass inside the soles
that are on the ground, and that region shrinks to the stance foot while a
foot is in the air.

At t = 2 s a horizontal force is applied to the torso when walking mode is
off. COM position, the COM reference and the tracking error are written to
``data/demo_log.npz`` when the window closes.

Usage:
    python main_demo.py
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from controllers.lateral_shift import LateralWeightShift, support_distance
from controllers.leg_ik import LegIK
from controllers.mpc_controller import MPCConfig, WholeBodyMPC
from controllers.pd_controller import PDController
from controllers.walk_state_machine import WalkConfig, WalkingStateMachine, WalkState
from env.industrial_humanoid_env import EnvConfig, IndustrialHumanoidEnv
from safety.cbf_filter import CBFConfig, CBFFilter
from swing_diagnostic import save_support_plot, summarize_single_support
from utils.logger import get_console_logger

LOG = get_console_logger("main_demo")

# A shove in +x at the torso. 300 N held for half a second is several times
# the moment the feet can resist, so the body rotates over the toe. 80 N for
# 0.2 s is enough to make the robot lean and the safety filter intervene,
# and the stance can still bring it back.
PUSH_TIME = 2.0  # seconds
PUSH_FORCE = np.array([80.0, 0.0, 0.0])  # newtons
PUSH_DURATION = 0.2  # seconds
PUSH_BODY_CANDIDATES = ("torso_link", "torso", "pelvis")

# Double-support weight shift used only by ``--no-walk`` is not this constant.
# Walking steps use the state machine below. 0.05 m/s remains the slow
# reference speed of the planted-foot mode.
WALK_SPEED = 0.05  # m/s, world +x

# Sole corners in the ankle frame, metres. The capsules run from 3.5 cm
# behind the ankle to 14 cm in front. Width is the capsule radius: the toe
# bar is wider, but the centre of mass has to sit on the part of the sole
# that is actually under the ankle, or the foot rolls over.
_SOLE_CORNERS = np.array(
    [
        [-0.035, -0.02, 0.0],
        [0.140, -0.02, 0.0],
        [0.140, 0.02, 0.0],
        [-0.035, 0.02, 0.0],
    ]
)
# The middle of that sole, used so the CoM target sits on the foot rather
# than on the ankle joint.
_SOLE_CENTER = np.array([0.0525, 0.0, 0.0])


def build_stack(
    args: argparse.Namespace,
) -> tuple[IndustrialHumanoidEnv, WholeBodyMPC, CBFFilter, PDController]:
    """Load the humanoid and construct the three controllers."""
    env = IndustrialHumanoidEnv(
        EnvConfig(
            model_path=args.model_path,
            control_dt=args.control_dt,
            # The demo owns the viewer; the env must not open a second one.
            render_mode=None,
            reset_joint_noise=0.0,
            episode_seconds=1.0e6,
        )
    )
    mpc = WholeBodyMPC(env.model, MPCConfig(horizon=args.horizon, dt=args.control_dt))
    cbf = CBFFilter(model=env.model, config=CBFConfig())
    pd = PDController(joint_names=env.joint_names)
    return env, mpc, cbf, pd


def sole_center_xy(env: IndustrialHumanoidEnv) -> np.ndarray:
    """Horizontal midpoint of the sole collision geoms, in the world frame.

    The home centre of mass sits almost over the heels. Holding that point
    leaves no room to sway backward, so a push recovery falls off the back
    of the feet. The middle of the soles is the point the stance can support.
    """
    import mujoco

    model, data = env.model, env.data
    ankles = []
    for body_id in range(int(model.nbody)):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        if "ankle" in name:
            ankles.append(np.asarray(data.xpos[body_id], dtype=float))
    if not ankles:
        return np.asarray(env.get_com_position()[:2], dtype=float)
    # The sole runs from about 3.5 cm behind the ankle to 14 cm in front, so
    # its middle is roughly 4 cm ahead of the ankle joint.
    center = np.mean(ankles, axis=0)
    center[0] += 0.04
    return center[:2]


def stand_reference(
    env: IndustrialHumanoidEnv, state: dict[str, np.ndarray]
) -> dict[str, np.ndarray]:
    """Hold the torso attitude, and hold the CoM over the middle of the feet."""
    com = np.asarray(state["com"], dtype=float).copy()
    com[:2] = sole_center_xy(env)
    return {
        "com": com,
        "torso_orientation": np.asarray(state["torso_rotation"], dtype=float).copy(),
    }


def foot_support_x(env: IndustrialHumanoidEnv) -> tuple[float, float, float]:
    """Heel, toe and midpoint along world x, from the ankle positions.

    The H1 sole runs from about 3.5 cm behind the ankle joint to 14 cm in
    front of it. Both feet share that forward axis in the home pose.
    """
    import mujoco

    model, data = env.model, env.data
    ankle_x = []
    for body_id in range(int(model.nbody)):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        if "ankle" in name:
            ankle_x.append(float(data.xpos[body_id][0]))
    if not ankle_x:
        com_x = float(env.get_com_position()[0])
        return com_x, com_x, com_x
    heel = min(ankle_x) - 0.035
    toe = max(ankle_x) + 0.14
    return heel, toe, 0.5 * (heel + toe)


def walking_reference(
    env: IndustrialHumanoidEnv,
    torso: np.ndarray,
    origin_xy: np.ndarray,
    height: float,
    speed: float,
    time_s: float,
    horizon: int,
    dt: float,
    x_limits: tuple[float, float],
) -> dict[str, np.ndarray]:
    """CoM reference that travels forward at ``speed`` with both feet planted.

    Knot ``k`` is the position at ``time_s + k dt``. The x coordinate is held
    at the edge of the sole once the ramp reaches it: past the toe there is
    no contact left to balance on, and this mode does not take a step.
    The velocity reference is the slope of that clipped ramp, so the MPC
    tracks 0.05 m/s in the interior and zero once the weight is fully forward.
    """
    times = float(time_s) + float(dt) * np.arange(horizon)
    x = np.clip(float(origin_xy[0]) + float(speed) * times, x_limits[0], x_limits[1])
    com = np.column_stack(
        (
            x,
            np.full(horizon, float(origin_xy[1])),
            np.full(horizon, float(height)),
        )
    )
    velocity = np.zeros((horizon, 3))
    if horizon > 1 and dt > 0.0:
        velocity[:-1, 0] = np.diff(x) / dt
        velocity[-1, 0] = velocity[-2, 0]
    return {
        "com": com,
        "com_velocity": velocity,
        "torso_orientation": np.asarray(torso, dtype=float).copy(),
    }


def control_step(
    env: IndustrialHumanoidEnv,
    mpc: WholeBodyMPC,
    cbf: CBFFilter,
    pd: PDController,
    reference: dict[str, np.ndarray],
    stance: tuple[str, ...] | None = None,
    joint_pose: dict[str, float] | None = None,
    swing_side: str | None = None,
    walk_state: str = "STAND",
    support_foot: str = "both",
    stance_hip_roll: tuple[str, float] | None = None,
    posture: LateralWeightShift | None = None,
) -> dict[str, np.ndarray | float | str]:
    """One control tick: MPC, PD, CBF, then torque into MuJoCo.

    Returns the quantities the demo logs. ``intervention`` is
    ``||tau_safe - tau_nominal||``; zero means the filter left the command
    unchanged.
    """
    state = env.get_state()
    if stance is not None:
        contacts = state.get("contact_jacobians") or {}
        planted = {
            name: jac
            for name, jac in contacts.items()
            if any(side in name for side in stance)
        }
        # During a swing the moving foot is not a contact. The CoM Jacobian
        # is taken relative to the stance foot only.
        if planted:
            state = dict(state)
            state["contact_jacobians"] = planted
    solution = mpc.solve(state, reference)
    q_des = solution["q_des"]
    if stance_hip_roll is not None:
        roll_side, roll_angle = stance_hip_roll
        for index, name in enumerate(mpc._joint_names):
            if name == f"{roll_side}_hip_roll":
                q_des[index] = float(np.clip(roll_angle, mpc._joint_low[index], mpc._joint_high[index]))
    if posture is not None:
        # The shift posture replaces the leg targets. It is solved from the
        # measured feet; the MPC's fore-aft residual is kept inside apply().
        # Applied before the swing inverse kinematics so the stance lean is
        # held and only the leg that is in the air is overwritten.
        q_des = posture.apply(
            q_des, mpc._joint_names, mpc._home_joint_qpos, mpc._joint_low, mpc._joint_high
        )
    if joint_pose is not None:
        q_des = _compose_step_joints(mpc, q_des, joint_pose, swing_side)
    qd_des = solution["qd_des"]

    # One CBF solve per control tick. The modification ``delta`` is held across
    # the physics substeps; the PD and the gravity term are recomputed on every
    # MuJoCo step so the joints can actually hold the target.
    tau_gravity = np.asarray(env.data.qfrc_bias[env._actuator_dof_adr], dtype=float)
    tau_nominal = pd.compute_torque(
        q_des,
        state["joint_qpos"],
        state["joint_qvel"],
        desired_velocities=qd_des,
        torque_feedforward=tau_gravity,
    )
    tau_safe = cbf.safe_torque(tau_nominal, state["com"], state["com_velocity"])
    delta = tau_safe - tau_nominal

    def torque_at_substep() -> np.ndarray:
        q = env.data.qpos[env._actuator_qpos_adr]
        qd = env.data.qvel[env._actuator_dof_adr]
        gravity = np.asarray(env.data.qfrc_bias[env._actuator_dof_adr], dtype=float)
        torque = pd.compute_torque(
            q_des, q, qd, desired_velocities=qd_des, torque_feedforward=gravity
        )
        return torque + delta

    env.step_feedback(torque_at_substep)

    after = env.get_state()
    ref_com = np.asarray(reference["com"], dtype=float)
    if ref_com.ndim == 2:
        ref_com = ref_com[0]
    ref_com = ref_com.reshape(3)
    return {
        "time": float(np.asarray(after["time"]).reshape(-1)[0]),
        "com": after["com"].copy(),
        "com_reference": ref_com.copy(),
        "com_error": after["com"] - ref_com,
        "com_velocity": after["com_velocity"].copy(),
        "intervention": float(np.linalg.norm(tau_safe - tau_nominal)),
        "cbf_status": cbf.status,
        "tau_nominal": tau_nominal.copy(),
        "tau_safe": tau_safe.copy(),
        "q_des": np.asarray(q_des, dtype=float).copy(),
        "joint_qpos": after["joint_qpos"].copy(),
        "joint_qvel": after["joint_qvel"].copy(),
        "base_rotation": after["base_rotation"].copy(),
        "barriers": cbf.com_barrier_values(after["com"]),
        "ankle_xy": _ankle_positions(env),
        "walk_state": walk_state,
        "support_foot": support_foot,
        "com_acceleration_des": np.asarray(solution["com_acceleration"], dtype=float).copy(),
    }


def _ankle_positions(env: IndustrialHumanoidEnv) -> np.ndarray:
    """World xy of each ankle body, shape ``(n, 2)``, to check the feet stay put."""
    import mujoco

    points = []
    for body_id in range(int(env.model.nbody)):
        name = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        if "ankle" in name:
            points.append(np.asarray(env.data.xpos[body_id][:2], dtype=float))
    if not points:
        return np.zeros((0, 2))
    return np.vstack(points)


def _hold_landed_leg(mpc: WholeBodyMPC, joint_pose: dict[str, float], side: str) -> None:
    """Make the landed swing leg the posture the MPC holds."""
    prefix = f"{side}_"
    for index, name in enumerate(mpc._joint_names):
        if name.startswith(prefix) and name in joint_pose:
            mpc._home_joint_qpos[index] = float(joint_pose[name])


def _compose_step_joints(
    mpc: WholeBodyMPC,
    q_mpc: np.ndarray,
    joint_pose: dict[str, float],
    swing_side: str | None,
) -> np.ndarray:
    """Put the legs on the inverse-kinematics pose, and keep balance on the stance leg.

    The MPC computes joint targets as a deviation from the home pose. After a
    step the feet are no longer at home, so the leg targets are rebuilt on the
    inverse-kinematics pose. The swing leg follows that pose exactly. The
    stance leg keeps the MPC's small balance correction, clipped so a bad
    Jacobian step cannot fold the knee.
    """
    home = np.asarray(mpc._home_joint_qpos, dtype=float)
    q_des = np.asarray(q_mpc, dtype=float).copy()
    for index, name in enumerate(mpc._joint_names):
        side = "left" if name.startswith("left_") else "right" if name.startswith("right_") else ""
        if side == "":
            continue
        if side == swing_side:
            if name in joint_pose:
                q_des[index] = joint_pose[name]
            continue
        if name in joint_pose:
            correction = float(q_mpc[index] - home[index])
            q_des[index] = joint_pose[name] + float(np.clip(correction, -0.2, 0.2))
    return np.clip(q_des, mpc._joint_low, mpc._joint_high)


def _ankle_pose(env: IndustrialHumanoidEnv, side: str) -> tuple[np.ndarray, np.ndarray]:
    """World position and rotation of one ankle body."""
    import mujoco

    body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_link")
    if body_id < 0:
        raise ValueError(f"model has no body {side}_ankle_link")
    position = np.asarray(env.data.xpos[body_id], dtype=float).copy()
    rotation = np.asarray(env.data.xmat[body_id], dtype=float).reshape(3, 3).copy()
    return position, rotation


def sole_corners(env: IndustrialHumanoidEnv, side: str) -> np.ndarray:
    """World xy corners of one sole, shape ``(4, 2)``."""
    position, rotation = _ankle_pose(env, side)
    world = position + (_SOLE_CORNERS @ rotation.T)
    return world[:, :2]


def support_polygon(env: IndustrialHumanoidEnv, stance: tuple[str, ...]) -> np.ndarray:
    """Corners of every sole that is currently carrying weight."""
    return np.vstack([sole_corners(env, side) for side in stance])


def build_step_command(
    env: IndustrialHumanoidEnv,
    gait: WalkingStateMachine,
    ik: LegIK,
    torso: np.ndarray,
    time_s: float,
    horizon: int,
    dt: float,
) -> tuple[dict[str, np.ndarray], np.ndarray, dict[str, float], str | None, object]:
    """MPC reference, support polygon and leg pose for one walking tick.

    The centre-of-mass preview follows the state machine, shifted from the
    ankle onto the middle of the sole. The polygon covers both feet except
    during a swing, when it covers only the stance sole.
    """
    times = float(time_s) + float(dt) * np.arange(horizon)
    samples = [gait.sample(float(tk)) for tk in times]
    com = np.vstack([sample.com_target for sample in samples])
    com[:, 0] += _SOLE_CENTER[0]
    velocity = np.zeros_like(com)
    if horizon > 1 and dt > 0.0:
        velocity[:-1] = np.diff(com, axis=0) / dt
        velocity[-1] = velocity[-2]
    reference = {
        "com": com,
        "com_velocity": velocity,
        "torso_orientation": np.asarray(torso, dtype=float).copy(),
    }
    now = samples[0]
    if now.state is WalkState.RIGHT_SWING:
        stance: tuple[str, ...] = ("left",)
        swing: str | None = "right"
    elif now.state is WalkState.LEFT_SWING:
        stance = ("right",)
        swing = "left"
    else:
        stance = ("left", "right")
        swing = None
    pose: dict[str, float] = {}
    for side, foot in (("left", now.left_foot), ("right", now.right_foot)):
        joints = ik.solve(foot, env, side=side)
        pose[f"{side}_hip_pitch"] = joints.hip_pitch
        pose[f"{side}_knee"] = joints.knee
        pose[f"{side}_ankle"] = joints.ankle_pitch
    return reference, support_polygon(env, stance), pose, swing, now


def _com_over_foot(env: IndustrialHumanoidEnv, side: str) -> bool:
    """True when the centre of mass and its capture point are inside one sole.

    Position alone is not enough. A centre of mass that is inside the sole
    but still moving outward leaves the sole as soon as the other foot lifts.
    The capture point ``c + v/omega`` has to be inside as well.
    """
    sole = support_polygon(env, (side,))
    state = env.get_state()
    com = np.asarray(state["com"], dtype=float).reshape(3)
    vel = np.asarray(state["com_velocity"], dtype=float).reshape(3)
    omega = float(np.sqrt(9.81 / max(float(com[2]), 0.2)))
    capture = com[:2] + vel[:2] / omega
    low = sole.min(axis=0) + 0.01
    high = sole.max(axis=0) - 0.01
    if np.any(high <= low):
        low = sole.min(axis=0)
        high = sole.max(axis=0)

    def _inside(point: np.ndarray) -> bool:
        return bool(np.all(point >= low) and np.all(point <= high))

    # A few centimetres per second is still a fly-through: the sole can
    # contain the capture point only because the foot has rolled with the body.
    slow = abs(float(vel[1])) < 0.03 and abs(float(vel[0])) < 0.08
    return _inside(com[:2]) and _inside(capture) and slow


def walk_tick(
    env: IndustrialHumanoidEnv,
    mpc: WholeBodyMPC,
    cbf: CBFFilter,
    pd: PDController,
    gait: WalkingStateMachine,
    ik: LegIK,
    torso: np.ndarray,
    rows: list[dict],
    swing_ramp: float = 0.03,
    freeze_swing: bool = False,
) -> dict:
    """One stepping tick: shift, swing, polygon update, then MPC-PD-CBF."""
    loaded = gait.loaded_foot
    ready = _com_over_foot(env, loaded) if loaded is not None else True
    command = gait.update(mpc.config.dt, ready)
    horizon = mpc.config.horizon
    com = np.tile(np.asarray(command.com_target, dtype=float), (horizon, 1))
    com[:, 0] += _SOLE_CENTER[0]
    reference = {
        "com": com,
        "com_velocity": np.zeros_like(com),
        "torso_orientation": np.asarray(torso, dtype=float).copy(),
    }
    swing = (
        "right"
        if command.state is WalkState.RIGHT_SWING
        else "left"
        if command.state is WalkState.LEFT_SWING
        else None
    )
    stance = ("left",) if swing == "right" else ("right",) if swing == "left" else None
    polygon = support_polygon(env, stance if stance is not None else ("left", "right"))
    # Do not lift, and do not shrink the polygon to one sole, until the centre
    # of mass is inside that sole. Lifting earlier drops the robot.
    if swing is not None and stance is not None:
        sole = support_polygon(env, stance)
        com_xy = np.asarray(env.get_state()["com"][:2], dtype=float)
        inside = bool(np.all(com_xy >= sole.min(axis=0)) and np.all(com_xy <= sole.max(axis=0)))
        if not inside:
            swing = None
            stance = None
            polygon = support_polygon(env, ("left", "right"))
    cbf.set_support_polygon(polygon)
    pose: dict[str, float] = {}
    for side, foot in (("left", command.left_foot), ("right", command.right_foot)):
        joints = ik.solve(foot, env, side=side)
        pose[f"{side}_hip_pitch"] = joints.hip_pitch
        pose[f"{side}_knee"] = joints.knee
        pose[f"{side}_ankle"] = joints.ankle_pitch
    # Planted legs stay on the MPC posture. Inverse kinematics is applied only
    # to the leg that is in the air. When that foot lands, its joint targets
    # become the new posture so the MPC does not pull the foot back home.
    previous_swing = str(rows[-1]["swing_side"]) if rows else ""
    if swing is None and previous_swing:
        held = getattr(gait, "_swing_command", {})
        # Land on the joints the swing actually reached. The raw inverse-
        # kinematics target can still be ahead of the ramp, and snapping to
        # it at touchdown is a torque step.
        landed_pose = dict(pose)
        for name, angle in held.items():
            if name.startswith(f"{previous_swing}_"):
                landed_pose[name] = float(angle)
        _hold_landed_leg(mpc, landed_pose, previous_swing)
        gait._landed_side = previous_swing
    # The state machine already names the stance ankle. That y coordinate is
    # thrown away by a midline reference, which is why the old shift stopped
    # at a few centimetres. Track the centre of the measured sole instead.
    shift = _weight_shift(gait, env.model)
    state_now = env.get_state()
    loaded = gait.loaded_foot
    if loaded is not None:
        sole = sole_corners(env, loaded)
        if shift.side != loaded:
            left_ankle, _left_rot = _ankle_pose(env, "left")
            right_ankle, _right_rot = _ankle_pose(env, "right")
            home = {name: float(q) for name, q in zip(mpc._joint_names, mpc._home_joint_qpos)}
            shift.begin(loaded, left_ankle, right_ankle, state_now["com"], sole, home)
        ref_xy = shift.advance(
            mpc.config.dt, state_now["com"], state_now["com_velocity"], sole
        )
        com[:, 0] = ref_xy[0]
        com[:, 1] = ref_xy[1]
        reference["com"] = com
        shift.skip_side = ""
        active_posture: LateralWeightShift | None = shift
        stance_sole = sole
        stance_xy = shift.target_xy.copy()
    elif shift.solved and shift.side:
        # The state machine's foot landmark is several centimetres off the
        # body (about +3 cm in x and -3 cm in y on the first landing). That
        # step saturates the MPC, and dropping the posture in the same tick
        # removes the ankle pitch that was holding the rear support edge.
        # Keep the clamped reference and the leaned stance leg. The landed
        # leg stays on the touchdown pose written above.
        sole = sole_corners(env, shift.side)
        shift.skip_side = str(getattr(gait, "_landed_side", ""))
        ref_xy = shift.advance(
            mpc.config.dt, state_now["com"], state_now["com_velocity"], sole
        )
        com[:, 0] = ref_xy[0]
        com[:, 1] = ref_xy[1]
        reference["com"] = com
        active_posture = shift
        both = support_polygon(env, ("left", "right"))
        stance_sole = both
        stance_xy = shift.target_xy.copy()
    else:
        active_posture = None
        both = support_polygon(env, ("left", "right"))
        stance_sole = both
        stance_xy = both.mean(axis=0)
    swing_pose = None
    if swing is not None and not freeze_swing:
        # Only the swinging leg leaves the shift posture. Replacing the
        # stance hip roll with the home angle drops the centre of mass off
        # the sole in one tick. The swing target is approached over the step,
        # not applied in one tick: that step is about 100 N·m, outside the
        # CBF trust region, so the filter fails safe and holds the
        # double-support torque while the foot is supposed to unload.
        measured = {
            name: float(q) for name, q in zip(mpc._joint_names, state_now["joint_qpos"])
        }
        held = getattr(gait, "_swing_command", {})
        swing_pose = {}
        ramp = abs(float(swing_ramp))
        for name, angle in pose.items():
            if not name.startswith(f"{swing}_"):
                continue
            previous = float(held.get(name, measured.get(name, angle)))
            swing_pose[name] = previous + float(np.clip(float(angle) - previous, -ramp, ramp))
        gait._swing_command = dict(swing_pose)
    else:
        gait._swing_command = {}
    row = control_step(
        env,
        mpc,
        cbf,
        pd,
        reference,
        stance=stance,
        joint_pose=swing_pose,
        swing_side=swing,
        walk_state=command.state.value,
        support_foot=command.support_foot,
        posture=active_posture,
    )
    row["swing_side"] = swing or ""
    row["stance_foot_xy"] = np.asarray(stance_xy, dtype=float).reshape(2)
    row["support_distance"] = support_distance(row["com"][:2], stance_sole)
    row["shift_fraction"] = float(shift.fraction)
    row["shift_limit"] = shift.limit
    dt = float(mpc.config.dt)
    if rows:
        row["com_acceleration"] = (np.asarray(row["com_velocity"], dtype=float) - np.asarray(rows[-1]["com_velocity"], dtype=float)) / dt
    else:
        row["com_acceleration"] = np.zeros(3)
    names = list(mpc._joint_names)
    home = np.asarray(mpc._home_joint_qpos, dtype=float)
    for side in ("left", "right"):
        for joint in ("hip_roll", "hip_pitch", "knee", "ankle"):
            index = names.index(f"{side}_{joint}")
            row[f"{side}_{joint}_des"] = float(row["q_des"][index])
            row[f"{side}_{joint}"] = float(row["joint_qpos"][index])
            row[f"{side}_{joint}_torque"] = float(row["tau_safe"][index])
            row[f"{side}_{joint}_nominal"] = float(row["tau_nominal"][index])
        row[f"{side}_knee_vel"] = float(row["joint_qvel"][names.index(f"{side}_knee")])
    stance_side = stance[0] if stance else ""
    if stance_side:
        ankle = names.index(f"{stance_side}_ankle")
        row["stance_ankle_correction"] = float(row["q_des"][ankle] - home[ankle])
    else:
        row["stance_ankle_correction"] = 0.0
    row["shift_target_xy"] = np.asarray(shift.target_xy, dtype=float).reshape(2).copy()
    com_now = np.asarray(row["com"], dtype=float).reshape(3)
    vel_now = np.asarray(row["com_velocity"], dtype=float).reshape(3)
    omega = float(np.sqrt(9.81 / max(float(com_now[2]), 0.2)))
    capture = com_now[:2] + vel_now[:2] / omega
    row["capture_xy"] = capture
    ref_xy = np.asarray(row["com_reference"][:2], dtype=float)
    row["capture_error"] = capture - ref_xy
    barriers = row.get("barriers") or {}
    for name in ("support_x_lower", "support_x_upper", "support_y_lower", "support_y_upper"):
        margin = float(barriers.get(name, np.nan))
        row[name] = margin
        axis = 0 if "x_" in name else 1
        # A lower-face margin is com - edge, so the capture margin is that
        # plus how far the capture point sits ahead of the centre of mass.
        sign = 1.0 if name.endswith("lower") else -1.0
        short = name.removeprefix("support_")
        row[f"capture_margin_{short}"] = margin + sign * float(capture[axis] - com_now[axis])
    rotation = np.asarray(row["base_rotation"], dtype=float).reshape(3, 3)
    row["pelvis_pitch"] = float(np.arctan2(-rotation[2, 0], np.hypot(rotation[2, 1], rotation[2, 2])))
    return row


def _bake_shift_pose(mpc: WholeBodyMPC, shift: LateralWeightShift, skip_side: str = "") -> None:
    """Copy the current leaned leg pose into the MPC home pose."""
    home = mpc._home_joint_qpos
    index = {name: i for i, name in enumerate(mpc._joint_names)}
    fraction = float(shift.fraction)
    for name, goal in shift.pose.items():
        if skip_side and name.startswith(f"{skip_side}_"):
            continue
        i = index.get(name)
        if i is None:
            continue
        home[i] = (1.0 - fraction) * float(home[i]) + fraction * float(goal)
    for side in ("left", "right"):
        if side == skip_side:
            continue
        hip = index.get(f"{side}_hip_pitch")
        knee = index.get(f"{side}_knee")
        ankle = index.get(f"{side}_ankle")
        if hip is None or knee is None or ankle is None:
            continue
        home[ankle] = -(float(home[hip]) + float(home[knee]))


def _weight_shift(gait: WalkingStateMachine, model) -> LateralWeightShift:
    """One shift solver per gait, created on the first tick."""
    shift = getattr(gait, "_lateral_shift", None)
    if shift is None or shift.model is not model:
        shift = LateralWeightShift(model)
        gait._lateral_shift = shift
    return shift


def apply_push(env: IndustrialHumanoidEnv, force: np.ndarray) -> str:
    """Apply ``force`` (N, world frame) to the torso for ``PUSH_DURATION``.

    Tries the usual H1 body names and falls back to the floating-base root
    when the scene uses a different one.
    """
    for candidate in PUSH_BODY_CANDIDATES:
        try:
            env.apply_external_force(force, body_name=candidate, duration=PUSH_DURATION)
        except ValueError:
            continue
        else:
            return candidate
    env.apply_external_force(force, body_name=None, duration=PUSH_DURATION)
    return "root"


def save_log(path: Path, rows: list[dict]) -> None:
    """Write COM position, the COM reference and the tracking error to ``path``."""
    path.parent.mkdir(parents=True, exist_ok=True)
    time_s = np.array([row["time"] for row in rows])
    com = np.vstack([row["com"] for row in rows])
    com_reference = np.vstack([row["com_reference"] for row in rows])
    com_error = np.vstack([row["com_error"] for row in rows])
    com_velocity = np.vstack([row["com_velocity"] for row in rows])
    intervention = np.array([row["intervention"] for row in rows])
    status = np.array([row["cbf_status"] for row in rows])
    ankle_xy = np.stack([row["ankle_xy"] for row in rows])
    walk_state = np.array([row.get("walk_state", "STAND") for row in rows])
    support_foot = np.array([row.get("support_foot", "both") for row in rows])
    stance_foot_xy = np.vstack(
        [np.asarray(row.get("stance_foot_xy", [0.0, 0.0]), dtype=float).reshape(2) for row in rows]
    )
    support_gap = np.array([float(row.get("support_distance", 0.0)) for row in rows])
    np.savez(
        path,
        time=time_s,
        com=com,
        com_reference=com_reference,
        com_error=com_error,
        com_velocity=com_velocity,
        intervention=intervention,
        cbf_status=status,
        ankle_xy=ankle_xy,
        walk_state=walk_state,
        support_foot=support_foot,
        stance_foot_xy=stance_foot_xy,
        support_distance=support_gap,
    )
    csv_path = path.with_suffix(".csv")
    header = (
        "time,com_x,com_y,com_z,ref_x,ref_y,ref_z,err_x,err_y,err_z,"
        "com_vx,com_vy,com_vz,intervention,cbf_status,walk_state,support_foot,"
        "stance_x,stance_y,support_distance"
    )
    lines = [header]
    for i in range(time_s.size):
        lines.append(
            f"{time_s[i]:.4f},{com[i, 0]:.6f},{com[i, 1]:.6f},{com[i, 2]:.6f},"
            f"{com_reference[i, 0]:.6f},{com_reference[i, 1]:.6f},{com_reference[i, 2]:.6f},"
            f"{com_error[i, 0]:.6f},{com_error[i, 1]:.6f},{com_error[i, 2]:.6f},"
            f"{com_velocity[i, 0]:.6f},{com_velocity[i, 1]:.6f},{com_velocity[i, 2]:.6f},"
            f"{intervention[i]:.6f},{status[i]},{walk_state[i]},{support_foot[i]},"
            f"{stance_foot_xy[i, 0]:.6f},{stance_foot_xy[i, 1]:.6f},{support_gap[i]:.6f}"
        )
    csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _finish_log(args: argparse.Namespace, env: IndustrialHumanoidEnv, log_path: Path, rows: list[dict]) -> None:
    """Write the demo log, and a swing summary when this run was walking."""
    try:
        if not rows:
            return
        save_log(log_path, rows)
        interventions = np.array([row["intervention"] for row in rows])
        LOG.info(
            "closed after %.2f s, %d steps. peak intervention %.2f Nm. log: %s",
            rows[-1]["time"],
            len(rows),
            float(interventions.max()),
            log_path,
        )
        if args.walk:
            report = summarize_single_support(rows)
            print(report)
            plot_path = log_path.with_name(log_path.stem + "_support.png")
            save_support_plot(rows, plot_path)
            LOG.info("support plot: %s", plot_path)
    finally:
        env.close()


def run_demo(args: argparse.Namespace) -> Path:
    """Open the MuJoCo viewer and run until the user closes it."""
    import mujoco.viewer

    env, mpc, cbf, pd = build_stack(args)
    env.reset(seed=args.seed)
    state0 = env.get_state()
    fixed_reference = stand_reference(env, state0)
    torso = np.asarray(state0["torso_rotation"], dtype=float).copy()
    height = float(state0["com"][2])
    gait: WalkingStateMachine | None = None
    ik: LegIK | None = None
    if args.walk:
        # The shift is slower than the swing so the centre of mass is over
        # the stance sole before the other foot leaves the ground.
        gait = WalkingStateMachine(
            WalkConfig(
                step_length=float(getattr(args, "step_length", 0.1)),
                step_height=0.05,
                step_duration=float(getattr(args, "step_duration", 0.8)),
                shift_duration=1.5,
                double_support_duration=0.4,
                stand_duration=1.5,
            )
        )
        ik = LegIK(env.model)
        left_ankle, _left_rot = _ankle_pose(env, "left")
        right_ankle, _right_rot = _ankle_pose(env, "right")
        gait.start(
            left_ankle,
            right_ankle,
            height,
            n_steps=int(args.steps),
            t0=0.0,
            first_swing="right",
        )
        # The home centre of mass sits a couple of centimetres ahead of the
        # heels. The standing margin of 2 cm would put that point on the
        # rear face and the filter would throw the robot over.
        cbf.config.margin = 0.0
        cbf.set_support_polygon(support_polygon(env, ("left", "right")))
    log_path = Path(args.log_path)
    rows: list[dict] = []
    pushed = False
    next_report = 1.0

    if args.walk:
        LOG.info(
            "%s %d forward steps, centre of mass shifts onto the stance foot before each swing.",
            "headless run." if getattr(args, "headless", False) else "viewer open — close the window to stop.",
            int(args.steps),
        )
    else:
        LOG.info(
            "%s Push of %.0f N at t = %.1f s.",
            "headless run." if getattr(args, "headless", False) else "viewer open — close the window to stop.",
            float(np.linalg.norm(args.push_force)),
            PUSH_TIME,
        )

    def one_tick(sim_time: float) -> dict:
        nonlocal pushed
        if args.walk:
            assert gait is not None and ik is not None
            return walk_tick(
                env,
                mpc,
                cbf,
                pd,
                gait,
                ik,
                torso,
                rows,
                swing_ramp=float(getattr(args, "swing_ramp", 0.03)),
                freeze_swing=bool(getattr(args, "freeze_swing", False)),
            )
        reference = fixed_reference
        if not pushed and sim_time >= PUSH_TIME:
            body = apply_push(env, np.asarray(args.push_force, dtype=float))
            pushed = True
            LOG.info(
                "t = %.2f s: applied %s N for %.2f s on %s",
                sim_time,
                np.array2string(np.asarray(args.push_force), precision=0),
                PUSH_DURATION,
                body,
            )
        return control_step(env, mpc, cbf, pd, reference)

    if bool(getattr(args, "headless", False)):
        duration = float(getattr(args, "duration", 24.0))
        try:
            while float(env.data.time) < duration:
                row = one_tick(float(env.data.time))
                rows.append(row)
                if float(row["com"][2]) < 0.72:
                    LOG.info("stopped: pelvis height %.3f m", float(row["com"][2]))
                    break
        finally:
            _finish_log(args, env, log_path, rows)
        return log_path

    # launch_passive returns immediately. The loop below is what keeps the
    # window alive; is_running() goes false when the user closes it.
    viewer = mujoco.viewer.launch_passive(env.model, env.data)
    try:
        while viewer.is_running():
            tick = time.perf_counter()
            sim_time = float(env.data.time)
            row = one_tick(sim_time)
            rows.append(row)
            viewer.sync()

            if row["time"] >= next_report:
                com = row["com"]
                vel = row["com_velocity"]
                err = row["com_error"]
                LOG.info(
                    "t = %5.2f s | %s support %s | COM [%.3f %.3f %.3f] m | "
                    "ref [%.3f %.3f] m | stance [%.3f %.3f] m | dist %.3f m | "
                    "error [%.3f %.3f %.3f] m | |v| = %.3f m/s | intervention %.2f Nm (%s)",
                    row["time"],
                    row["walk_state"],
                    row["support_foot"],
                    com[0],
                    com[1],
                    com[2],
                    row["com_reference"][0],
                    row["com_reference"][1],
                    float(np.asarray(row.get("stance_foot_xy", [0.0, 0.0]))[0]),
                    float(np.asarray(row.get("stance_foot_xy", [0.0, 0.0]))[1]),
                    float(row.get("support_distance", 0.0)),
                    err[0],
                    err[1],
                    err[2],
                    float(np.linalg.norm(vel)),
                    row["intervention"],
                    row["cbf_status"],
                )
                next_report += 1.0

            # Pace the loop to the control period so the viewer is watchable.
            remaining = args.control_dt - (time.perf_counter() - tick)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        viewer.close()
        _finish_log(args, env, log_path, rows)
    return log_path


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="MPC + CBF demo for the Unitree H1")
    parser.add_argument("--horizon", type=int, default=20, help="MPC horizon length")
    parser.add_argument("--control-dt", type=float, default=0.02, help="control period in seconds")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--walk",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="take alternating forward steps; --no-walk stands and takes the push",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=4,
        help="number of swing steps in walking mode",
    )
    parser.add_argument(
        "--walk-speed",
        type=float,
        default=WALK_SPEED,
        help="forward CoM reference speed in m/s (unused by the stepping gait)",
    )
    parser.add_argument(
        "--push-force",
        type=float,
        nargs=3,
        default=PUSH_FORCE.tolist(),
        metavar=("FX", "FY", "FZ"),
        help="world-frame force (N) applied at t = 2 s",
    )
    parser.add_argument(
        "--model-path",
        type=Path,
        default=root / "models" / "unitree_h1" / "scene.xml",
        help="MJCF scene for the H1",
    )
    parser.add_argument(
        "--log-path",
        type=Path,
        default=root / "data" / "demo_log.npz",
        help="where COM and intervention traces are written",
    )
    parser.add_argument("--step-length", type=float, default=0.1, help="forward step length in metres")
    parser.add_argument("--step-duration", type=float, default=0.8, help="swing duration in seconds")
    parser.add_argument(
        "--swing-ramp",
        type=float,
        default=0.03,
        help="maximum swing-joint target change per control tick, in radians",
    )
    parser.add_argument(
        "--freeze-swing",
        action="store_true",
        help="enter single support but do not move the swing leg (diagnostic only)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without the viewer, for a fixed --duration, then print the swing diagnostic",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=24.0,
        help="simulated seconds when --headless is set",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_demo(parse_args())
