"""Demonstration: MPC, PD and a CBF torque filter on the Unitree H1.

The control stack on every tick is

    state -> WholeBodyMPC -> PDController -> CBFFilter -> joint torques -> MuJoCo

The MPC holds the torso attitude recorded at reset. In walking mode a state
machine shifts the centre of mass onto the measured centre of one sole before
the other foot swings forward. During the swing that reference is replaced
by a capture-point velocity toward the planned landing sole, still through
the same MPC. The shift is a blend from the home posture to
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

from controllers.adaptive_foot_placement import combined_y_margin
from controllers.capture_point_controller import CapturePreparation
from controllers.support_transition import (
    TRANSITION_Y,
    FallRelease,
    apply_stance_hip_roll,
    capture_box_margin,
    combined_support_active,
    fall_blend,
    fall_safe_box,
    stance_hip_roll_capture_target,
    stance_inner_edge_y,
    support_phase,
)
from controllers.lateral_shift import LateralWeightShift, support_distance
from controllers.load_transfer import (
    LoadTransferTracker,
    final_window,
    load_sample_ok,
    plan_load_transfer,
)
from controllers.leg_ik import LegIK
from controllers.mpc_controller import MPCConfig, WholeBodyMPC
from controllers.pd_controller import PDController
from controllers.walk_state_machine import (
    WalkConfig,
    WalkingStateMachine,
    WalkState,
    contact_qualified_corners,
    recovery_conditions_met,
    support_box_center,
    touchdown_sample_ok,
)
from env.industrial_humanoid_env import EnvConfig, IndustrialHumanoidEnv
from safety.cbf_filter import CBFConfig, CBFFilter
from swing_diagnostic import save_support_plot, summarize_single_support
from utils.experiment_logger import write_experiment_log
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


# Walking stance-hip tracking only. The standing leg law stays kp 400 / kd 40.
# On the planted hip, about 30 N·m holds the pose, so kp 400 settles with
# ~0.07 rad of lag and the 0.15 rad fall does not finish inside the swing.
# The v1 nominal torque there was 30–75 N·m. The motor is a gear-1 torque
# source with ctrlrange ±200 N·m, joint damping 1 and no stiffness, so the
# actuator was not the limit, and once the inner face was open the CBF
# passed that torque unchanged.
# kp 1000 and kp 700 both closed that gap inside the swing. The body then
# kept falling for about a second while the right foot was still in the
# air, and the pelvis carried that foot out to about -0.38 m before
# contact. The extra stiffness is therefore only used while the swing is
# in progress, so the hip moves during the 0.8 s swing and the existing
# leg gains hold the catch afterwards. Damping scales with sqrt(kp).
# Torque still goes through PDController, the ±200 N·m clip, and the CBF.
_WALK_STANCE_HIP_ROLL_KP = 700.0
_WALK_STANCE_HIP_ROLL_KD = 53.0
_WALK_STANCE_HIP_ROLL_STATES = ("RIGHT_SWING", "LEFT_SWING")


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
    *,
    landed_side: str = "",
    landed_ankle_capture: bool = True,
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
    solve_started = time.perf_counter()
    solution = mpc.solve(
        state,
        reference,
        landed_side=landed_side,
        landed_ankle_capture=landed_ankle_capture,
    )
    mpc_solve_s = float(time.perf_counter() - solve_started)
    q_des = np.asarray(solution["q_des"], dtype=float).copy()
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
    # Snapshot before the fall write. Swing IK only fills swing-side joints,
    # so this is the shift blend on the stance hip.
    q_des_before_walking_override = np.asarray(q_des, dtype=float).copy()
    # Last write on the stance hip. The shift, the swing IK and the recovery
    # capture term have already run. This is the target the PD receives.
    if stance_hip_roll is not None:
        roll_side, roll_angle = stance_hip_roll
        q_des = apply_stance_hip_roll(
            q_des,
            list(mpc._joint_names),
            roll_side,
            float(roll_angle),
            mpc._joint_low,
            mpc._joint_high,
        )
    q_des = np.asarray(q_des, dtype=float).copy()
    qd_des = np.asarray(solution["qd_des"], dtype=float)

    names = list(mpc._joint_names)
    hip_index = names.index("left_hip_roll")
    saved_hip_gains = None
    if stance_hip_roll is not None and walk_state in _WALK_STANCE_HIP_ROLL_STATES:
        stance_index = names.index(f"{stance_hip_roll[0]}_hip_roll")
        saved_hip_gains = (
            stance_index,
            float(pd.kp[stance_index]),
            float(pd.kd[stance_index]),
        )
        pd.kp[stance_index] = _WALK_STANCE_HIP_ROLL_KP
        pd.kd[stance_index] = _WALK_STANCE_HIP_ROLL_KD

    q_meas = np.asarray(state["joint_qpos"], dtype=float)
    qd_meas = np.asarray(state["joint_qvel"], dtype=float)
    position_error = q_des - q_meas
    velocity_error = qd_des - qd_meas
    tau_position = pd.kp * position_error
    tau_velocity = pd.kd * velocity_error
    # One CBF solve per control tick. The modification ``delta`` is held across
    # the physics substeps; the PD and the gravity term are recomputed on every
    # MuJoCo step so the joints can actually hold the target.
    tau_gravity = np.asarray(env.data.qfrc_bias[env._actuator_dof_adr], dtype=float)
    tau_pd = tau_position + tau_velocity
    try:
        tau_nominal = pd.compute_torque(
            q_des,
            q_meas,
            qd_meas,
            desired_velocities=qd_des,
            torque_feedforward=tau_gravity,
        )
        tau_safe = cbf.safe_torque(tau_nominal, state["com"], state["com_velocity"])
        delta = tau_safe - tau_nominal
        first_ctrl: dict[str, np.ndarray] = {}

        def torque_at_substep() -> np.ndarray:
            q = env.data.qpos[env._actuator_qpos_adr]
            qd = env.data.qvel[env._actuator_dof_adr]
            gravity = np.asarray(env.data.qfrc_bias[env._actuator_dof_adr], dtype=float)
            torque = pd.compute_torque(
                q_des, q, qd, desired_velocities=qd_des, torque_feedforward=gravity
            )
            commanded = torque + delta
            if "first" not in first_ctrl:
                first_ctrl["first"] = np.asarray(commanded, dtype=float).copy()
            return commanded

        env.step_feedback(torque_at_substep)
        hip_trace = _left_hip_roll_trace(
            env,
            hip_index,
            q_des_before_walking_override,
            q_des,
            q_meas,
            qd_meas,
            position_error,
            tau_position,
            tau_velocity,
            tau_pd,
            tau_gravity,
            tau_nominal,
            tau_safe,
            first_ctrl.get("first", tau_safe),
            float(pd.kp[hip_index]),
            float(pd.kd[hip_index]),
        )
    finally:
        if saved_hip_gains is not None:
            stance_index, kp_standing, kd_standing = saved_hip_gains
            pd.kp[stance_index] = kp_standing
            pd.kd[stance_index] = kd_standing

    after = env.get_state()
    ref_com = np.asarray(reference["com"], dtype=float)
    if ref_com.ndim == 2:
        ref_com = ref_com[0]
    ref_com = ref_com.reshape(3)
    torso_ref = reference.get("torso_orientation")
    left_foot, left_vel = _body_pose_and_velocity(env, "left_ankle_link")
    right_foot, right_vel = _body_pose_and_velocity(env, "right_ankle_link")
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
        "qd_des": np.asarray(qd_des, dtype=float).copy(),
        "qpos": after["qpos"].copy(),
        "qvel": after["qvel"].copy(),
        "joint_qpos": after["joint_qpos"].copy(),
        "joint_qvel": after["joint_qvel"].copy(),
        "base_rotation": after["base_rotation"].copy(),
        "pelvis_rpy": _rpy(after["base_rotation"]),
        "torso_rpy": _rpy(after["torso_rotation"]),
        "torso_rpy_ref": _rpy(torso_ref) if torso_ref is not None else np.full(3, np.nan),
        "left_foot_xyz": left_foot,
        "right_foot_xyz": right_foot,
        "left_foot_vel": left_vel,
        "right_foot_vel": right_vel,
        "mpc_solve_s": mpc_solve_s,
        "barriers": cbf.com_barrier_values(after["com"]),
        "ankle_xy": _ankle_positions(env),
        "walk_state": walk_state,
        "support_foot": support_foot,
        "com_acceleration_des": np.asarray(solution["com_acceleration"], dtype=float).copy(),
        "sagittal_capture_offset": float(solution.get("sagittal_capture_offset", 0.0)),
        "landed_ankle_capture_suppressed": float(
            solution.get("landed_ankle_capture_suppressed", 0.0)
        ),
        **hip_trace,
    }


def _left_hip_roll_trace(
    env: IndustrialHumanoidEnv,
    index: int,
    q_des_before: np.ndarray,
    q_des: np.ndarray,
    q_meas: np.ndarray,
    qd_meas: np.ndarray,
    position_error: np.ndarray,
    tau_position: np.ndarray,
    tau_velocity: np.ndarray,
    tau_pd: np.ndarray,
    tau_gravity: np.ndarray,
    tau_nominal: np.ndarray,
    tau_safe: np.ndarray,
    first_ctrl: np.ndarray,
    kp: float,
    kd: float,
) -> dict[str, float]:
    """Left hip-roll command, PD terms, and the torque MuJoCo received.

    ``q_des`` here is the vector immediately before the PD. ``first_ctrl`` is
    the torque passed into the first physics substep, before ``ctrlrange``
    clipping. ``left_hip_roll_ctrl`` is the actuator command after that clip
    on the last substep.
    """
    import mujoco

    actuator = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_hip_roll")
    joint = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, "left_hip_roll")
    dof = int(env.model.jnt_dofadr[joint])
    mass = np.zeros((env.model.nv, env.model.nv), dtype=float)
    mujoco.mj_fullM(env.model, env.data, mass)
    inertia = float(mass[dof, dof])
    return {
        "q_des_before_walking_override": float(q_des_before[index]),
        "q_des_after_walking_override": float(q_des[index]),
        "q_des_immediately_before_PD": float(q_des[index]),
        "measured_q": float(q_meas[index]),
        "measured_qvel": float(qd_meas[index]),
        "position_error": float(position_error[index]),
        "tau_position": float(tau_position[index]),
        "tau_velocity": float(tau_velocity[index]),
        "tau_PD": float(tau_pd[index]),
        "qfrc_bias": float(tau_gravity[index]),
        "tau_gravity": float(tau_gravity[index]),
        "left_hip_roll_tau_unclipped": float(
            tau_position[index] + tau_velocity[index] + tau_gravity[index]
        ),
        "left_hip_roll_cbf_intervention": float(tau_nominal[index] - tau_safe[index]),
        "left_hip_roll_ctrl_command": float(first_ctrl[index]),
        "left_hip_roll_ctrl": float(env.data.ctrl[actuator]),
        "left_hip_roll_actuator_force": float(env.data.actuator_force[actuator]),
        "left_hip_roll_kp": float(kp),
        "left_hip_roll_kd": float(kd),
        "left_hip_roll_gear": float(env.model.actuator_gear[actuator, 0]),
        "left_hip_roll_ctrl_low": float(env.model.actuator_ctrlrange[actuator, 0]),
        "left_hip_roll_ctrl_high": float(env.model.actuator_ctrlrange[actuator, 1]),
        "left_hip_roll_force_low": float(env.model.actuator_forcerange[actuator, 0]),
        "left_hip_roll_force_high": float(env.model.actuator_forcerange[actuator, 1]),
        "left_hip_roll_damping": float(env.model.dof_damping[dof]),
        "left_hip_roll_armature": float(env.model.dof_armature[dof]),
        "left_hip_roll_stiffness": float(env.model.jnt_stiffness[joint]),
        "left_hip_roll_limit_low": float(env.model.jnt_range[joint, 0]),
        "left_hip_roll_limit_high": float(env.model.jnt_range[joint, 1]),
        "left_hip_roll_inertia": inertia,
    }


def _rpy(rotation: np.ndarray) -> np.ndarray:
    """Roll, pitch and yaw in radians from a body-to-world rotation."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.ndim == 3:
        matrix = matrix[0]
    matrix = matrix.reshape(3, 3)
    roll = float(np.arctan2(matrix[2, 1], matrix[2, 2]))
    pitch = float(np.arctan2(-matrix[2, 0], np.hypot(matrix[2, 1], matrix[2, 2])))
    yaw = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
    return np.array([roll, pitch, yaw], dtype=float)


def _body_pose_and_velocity(env: IndustrialHumanoidEnv, body_name: str) -> tuple[np.ndarray, np.ndarray]:
    """World position and linear velocity of a body. Reads MuJoCo; writes nothing."""
    import mujoco

    body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return np.full(3, np.nan), np.full(3, np.nan)
    position = np.asarray(env.data.xpos[body_id], dtype=float).copy()
    spatial = np.zeros(6)
    mujoco.mj_objectVelocity(env.model, env.data, mujoco.mjtObj.mjOBJ_BODY, body_id, spatial, 0)
    return position, np.asarray(spatial[3:], dtype=float).copy()


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
    """True when a swing may start on ``side``.

    The foot has to be down and carrying load, the body upright, and both
    the centre of mass and the capture point inside that sole, with the
    horizontal speed still bounded. The capture point does not have to sit
    in the middle of the sole. After liftoff the step is allowed to carry
    it toward the inner edge; this gate only stops an early uncontrolled lift.
    """
    sole = support_polygon(env, (side,))
    state = env.get_state()
    com = np.asarray(state["com"], dtype=float).reshape(3)
    vel = np.asarray(state["com_velocity"], dtype=float).reshape(3)
    omega = float(np.sqrt(9.81 / max(float(com[2]), 0.2)))
    capture = com[:2] + vel[:2] / omega
    low = sole.min(axis=0)
    high = sole.max(axis=0)

    def _inside(point: np.ndarray) -> bool:
        return bool(np.all(point >= low) and np.all(point <= high))

    slow = abs(float(vel[1])) < 0.05 and abs(float(vel[0])) < 0.08
    roll, pitch, _yaw = _rpy(np.asarray(state["base_rotation"], dtype=float))
    upright = abs(float(roll)) < 0.35 and abs(float(pitch)) < 0.35
    loaded = _foot_in_contact(env, side) and _foot_normal_force(env, side) > 30.0
    return _inside(com[:2]) and _inside(capture) and slow and upright and loaded


def _empty_lateral_log() -> dict[str, float | str]:
    """Lateral-IK fields for a tick that did not command a swing foot."""
    return {
        "desired_foot_x": np.nan,
        "desired_foot_y": np.nan,
        "desired_foot_z": np.nan,
        "current_foot_x": np.nan,
        "current_foot_y": np.nan,
        "current_foot_z": np.nan,
        "applied_foot_y": np.nan,
        "lateral_ik_correction": 0.0,
        "ik_reachable": np.nan,
        "ik_failure_reason": "",
        "hip_roll_target": np.nan,
        "hip_yaw_target": np.nan,
        "ankle_roll_target": np.nan,
    }


_PREPARATION_STATES = (
    WalkState.RIGHT_SWING,
    WalkState.LEFT_SWING,
    WalkState.TOUCHDOWN_PREP,
    WalkState.TOUCHDOWN,
)
def _arm_phase_support(
    cbf: CBFFilter,
    env: IndustrialHumanoidEnv,
    gait: WalkingStateMachine,
    phase: WalkState,
    swing_side: str | None,
    stance: tuple[str, ...] | None,
    polygon: np.ndarray,
    capture_y: float,
    both_in_contact: bool,
) -> None:
    """Point the CBF at the support box for this walking phase.

    Standing, the weight shift and early swing keep the geometric sole.
    Late swing moves only the inner face into the gap. After both feet are
    down the box is their measured combined polygon. The capture reference
    is the transition line once the fall has started.
    """
    duration = float(gait.config.step_duration)
    progress = float(np.clip(gait.swing_elapsed / duration, 0.0, 1.0)) if duration > 0.0 else 0.0
    phase_name = support_phase(phase.value, progress, late_start=0.30)
    opening = fall_blend(progress, phase.value)
    gait._fall_progress = progress
    gait._fall_opening = opening
    transitioning = (
        swing_side in ("left", "right")
        and phase in _PREPARATION_STATES + (WalkState.DOUBLE_SUPPORT_RECOVERY,)
    )
    edge = float("nan")
    target_y = float("nan")
    if not transitioning:
        cbf.set_support_polygon(polygon)
    else:
        stance_foot = "left" if swing_side == "right" else "right"
        if stance is not None and len(stance) == 1:
            stance_foot = stance[0]
        stance_corners = sole_corners(env, stance_foot)
        edge = stance_inner_edge_y(stance_corners, swing_side)
        confirmed = combined_support_active(phase.value, both_in_contact)
        if confirmed:
            cbf.set_support_polygon(polygon)
        elif opening <= 1e-9:
            cbf.set_support_polygon(stance_corners)
        else:
            box = fall_safe_box(stance_corners, swing_side, opening)
            cbf.set_support_bounds(box[0], box[1])
        if opening > 1e-9:
            target_y = float(TRANSITION_Y)
    low, high = cbf.support_bounds
    if swing_side == "left":
        inner_y = float(high[1])
        outer_y = float(low[1])
    elif swing_side == "right":
        inner_y = float(low[1])
        outer_y = float(high[1])
    else:
        inner_y = float(low[1])
        outer_y = float(high[1])
    gait._transition_capture_y = target_y
    gait._support_log = {
        "support_phase": phase_name,
        "cbf_support_region_min_y": float(low[1]),
        "cbf_support_region_max_y": float(high[1]),
        "stance_inner_edge_y": float(edge),
        "support_inner_edge_original": float(edge),
        "support_inner_edge_active": inner_y,
        "support_outer_edge": outer_y,
        "support_toe_edge": float(high[0]),
        "support_heel_edge": float(low[0]),
    }


def _support_accelerations(cbf: CBFFilter, row: dict) -> tuple[float, float]:
    """Lateral CoM acceleration of the nominal torque and of the safe torque.

    These are the accelerations the support barrier sees. The capture-point
    controller does not produce them.
    """
    matrix = np.asarray(cbf.control_matrix, dtype=float)
    if matrix.ndim != 2 or matrix.shape[0] < 2 or matrix.shape[1] == 0:
        return float("nan"), float("nan")
    width = int(matrix.shape[1])
    nominal = np.asarray(row.get("tau_nominal", np.zeros(width)), dtype=float).reshape(-1)
    safe = np.asarray(row.get("tau_safe", np.zeros(width)), dtype=float).reshape(-1)
    if nominal.size < width or safe.size < width:
        return float("nan"), float("nan")
    bias = np.asarray(getattr(cbf, "_a0", np.zeros(2)), dtype=float).reshape(-1)
    nominal_accel = matrix @ nominal[:width]
    safe_accel = matrix @ safe[:width]
    if bias.size >= 2:
        nominal_accel = nominal_accel + bias[: matrix.shape[0]]
        safe_accel = safe_accel + bias[: matrix.shape[0]]
    return float(nominal_accel[1]), float(safe_accel[1])


def _capture_controller(gait: WalkingStateMachine) -> CapturePreparation:
    controller = getattr(gait, "_capture_preparation", None)
    if controller is None:
        controller = CapturePreparation(max_forward_velocity=WALK_SPEED)
        gait._capture_preparation = controller
    return controller


def _apply_capture_preparation(
    gait: WalkingStateMachine,
    env: IndustrialHumanoidEnv,
    shift: LateralWeightShift,
    reference: dict[str, np.ndarray],
    state_now: dict[str, np.ndarray],
    phase: WalkState,
    swing_side: str | None,
    dt: float,
    capture_target: np.ndarray | None = None,
) -> None:
    """Replace the lateral CoM reference with the capture-point velocity.

    The target is chosen by the support phase: the stance inner edge during
    late swing, then the middle of the combined soles after contact. The
    forward reference stays on the weight shift. This function does not
    write a torque. The CBF remains the last filter on the way to MuJoCo.
    """
    preparing = (
        capture_target is not None
        and phase in _PREPARATION_STATES + (WalkState.DOUBLE_SUPPORT_RECOVERY,)
    )
    command = _capture_controller(gait).step(
        state_now["com"],
        state_now["com_velocity"],
        None,
        None,
        1.0,
        preparing,
        dt,
        capture_target=capture_target,
    )
    gait._capture_log = {
        "capture_preparation_active": 1.0 if command.active else 0.0,
        "capture_preparation_blend": float(command.blend),
        "current_capture_x": float(command.current_capture[0]),
        "current_capture_y": float(command.current_capture[1]),
        "desired_capture_x": float(command.desired_capture[0]),
        "desired_capture_y": float(command.desired_capture[1]),
        "capture_error_x": float(command.capture_error[0]),
        "capture_error_y": float(command.capture_error[1]),
        "desired_com_vx": float(command.desired_com_velocity[0]),
        "desired_com_vy": float(command.desired_com_velocity[1]),
        "applied_com_vx": 0.0,
        "applied_com_vy": 0.0,
    }
    if not _fall_holds_lean(gait, phase, swing_side):
        shift.preparation_hold = bool(command.active)
    if not command.active:
        gait._applied_com_velocity = np.zeros(2)
        return
    measured = np.asarray(state_now["com"], dtype=float).reshape(3)
    velocity = np.asarray(command.desired_com_velocity, dtype=float).reshape(2)
    gait._applied_com_velocity = velocity.copy()
    gait._capture_log["applied_com_vx"] = float(velocity[0])
    gait._capture_log["applied_com_vy"] = float(velocity[1])
    com_ref = np.asarray(reference["com"], dtype=float).copy()
    if com_ref.ndim == 1:
        com_ref = com_ref.reshape(1, 3)
    for knot in range(com_ref.shape[0]):
        com_ref[knot, 1] = float(measured[1]) + float(velocity[1]) * (knot + 1) * float(dt)
    reference["com"] = com_ref
    vel_ref = np.asarray(reference.get("com_velocity", np.zeros_like(com_ref)), dtype=float).copy()
    if vel_ref.shape != com_ref.shape:
        vel_ref = np.zeros_like(com_ref)
    vel_ref[:, 1] = float(velocity[1])
    reference["com_velocity"] = vel_ref


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
    late_swing_capture: bool = False,
    landing_handoff: bool = False,
    touchdown_agent=None,
) -> dict:
    """One stepping tick: shift, swing, polygon update, then MPC-PD-CBF."""
    loaded = gait.loaded_foot
    ready = _com_over_foot(env, loaded) if loaded is not None else True
    contacts = {side: _foot_in_contact(env, side) for side in ("left", "right")}
    normal_force = {side: _foot_normal_force(env, side) for side in ("left", "right")}
    foot_ok = False
    recovery_ok = False
    if gait.state is WalkState.TOUCHDOWN and gait.active_swing in ("left", "right"):
        foot_ok = _settled_touchdown(env, gait.active_swing, gait.config)
    if gait.state is WalkState.DOUBLE_SUPPORT_RECOVERY:
        recovery_ok = _recovery_sample_ok(env, contacts, gait.config)
    # Same capture point the rest of the stack uses: c + v/omega, with
    # omega = sqrt(g / z). This is the pre-step state. The logged capture_xy
    # remains the post-step value.
    state_pre = env.get_state()
    com_pre = np.asarray(state_pre["com"], dtype=float).reshape(3)
    vel_pre = np.asarray(state_pre["com_velocity"], dtype=float).reshape(3)
    omega_pre = float(np.sqrt(9.81 / max(float(com_pre[2]), 0.2)))
    capture_pre = com_pre[:2] + vel_pre[:2] / omega_pre
    stance_name = gait.loaded_foot if gait.loaded_foot in ("left", "right") else ""
    stance_y = None
    stance_inner = None
    if stance_name:
        stance_ankle, _stance_rot = _ankle_pose(env, stance_name)
        stance_y = float(stance_ankle[1])
        swing_y = gait.original_swing_target_y
        sole_y = sole_corners(env, stance_name)[:, 1]
        if np.isfinite(swing_y):
            stance_inner = float(sole_y.min()) if float(swing_y) <= stance_y else float(sole_y.max())
    placement_reachable = True
    if gait.placement_check_due(mpc.config.dt):
        proposed = gait.full_adaptive_target(capture_pre, stance_y, stance_inner)
        swing_side = gait.active_swing
        original_y = gait.original_swing_target_y
        if proposed is not None and swing_side in ("left", "right") and np.isfinite(original_y):
            # The check is the ankle the existing pitch solution would reach,
            # plus the full lateral correction. The planner's absolute y is
            # not a point the current hip roll is tracking.
            landing = np.asarray(proposed, dtype=float).copy()
            correction = float(landing[1] - original_y)
            landing[1] = float(original_y)
            sagittal = ik.solve(landing, env, side=swing_side)
            reached = ik.ankle_position(env, swing_side, sagittal)
            desired = reached.copy()
            desired[1] += correction
            placement_reachable = bool(
                ik.solve_lateral(
                    desired,
                    env,
                    side=swing_side,
                    max_lateral=float(gait.config.max_lateral_step_adjustment) + 0.02,
                ).reachable
            )
    command = gait.update(
        mpc.config.dt,
        ready,
        foot_sample_ok=foot_ok,
        recovery_sample_ok=recovery_ok,
        capture_xy=capture_pre,
        placement_reachable=placement_reachable,
        stance_foot_y=stance_y,
        stance_inner_y=stance_inner,
    )
    horizon = mpc.config.horizon
    com = np.tile(np.asarray(command.com_target, dtype=float), (horizon, 1))
    com[:, 0] += _SOLE_CENTER[0]
    reference = {
        "com": com,
        "com_velocity": np.zeros_like(com),
        "torso_orientation": np.asarray(torso, dtype=float).copy(),
    }
    phase = command.state
    active = gait.active_swing
    # The cubic is still running through touchdown prep. TOUCHDOWN holds the
    # landing target and waits for contact, so that foot is not a swing command.
    swing = (
        active
        if phase in (WalkState.RIGHT_SWING, WalkState.LEFT_SWING, WalkState.TOUCHDOWN_PREP)
        and active in ("left", "right")
        else None
    )
    single_support = phase in (
        WalkState.RIGHT_SWING,
        WalkState.LEFT_SWING,
        WalkState.TOUCHDOWN_PREP,
        WalkState.TOUCHDOWN,
    )
    if single_support and active == "right":
        stance: tuple[str, ...] | None = ("left",)
    elif single_support and active == "left":
        stance = ("right",)
    else:
        stance = None
    if phase is WalkState.DOUBLE_SUPPORT_RECOVERY:
        soles = {side: sole_corners(env, side) for side in ("left", "right")}
        qualified = contact_qualified_corners(soles, contacts)
        if qualified.shape[0] == 0:
            polygon = support_polygon(env, ("left", "right"))
            stance = None
            cbf_support_feet = "left+right"
        else:
            polygon = qualified
            planted = tuple(side for side in ("left", "right") if contacts[side])
            stance = planted if planted else None
            cbf_support_feet = "+".join(planted) if planted else "none"
    else:
        polygon = support_polygon(env, stance if stance is not None else ("left", "right"))
        cbf_support_feet = "+".join(stance) if stance else "left+right"
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
                cbf_support_feet = "left+right"
    # None means every contact Jacobian the model reported. A named tuple is
    # the only set the MPC treats as planted.
    mpc_support_feet = "+".join(stance) if stance else "all"
    _arm_phase_support(
        cbf,
        env,
        gait,
        phase,
        active,
        stance,
        polygon,
        float(capture_pre[1]),
        bool(contacts["left"] and contacts["right"]),
    )
    pose: dict[str, float] = {}
    lateral_log = _empty_lateral_log()
    placed_side = str(getattr(gait, "_placed_hip_roll_side", ""))
    lateral_correction = float(gait.adaptive_lateral_correction)
    if not gait.config.adaptive_foot_placement:
        placed_side = ""
    elif swing is not None and abs(lateral_correction) <= 1e-6 and swing != placed_side:
        placed_side = ""
    for side, foot in (("left", command.left_foot), ("right", command.right_foot)):
        target = np.asarray(foot, dtype=float).reshape(3).copy()
        use_lateral = (
            bool(gait.config.adaptive_foot_placement)
            and swing == side
            and abs(lateral_correction) > 1e-6
        )
        if use_lateral:
            original_y = gait.original_swing_target_y
            sagittal_target = target.copy()
            if np.isfinite(original_y):
                sagittal_target[1] = float(original_y)
            # The offset is measured from the hip roll the weight shift would
            # command. The measured roll already contains an earlier lateral
            # command, and adding the correction to that rolls the target again.
            reference_roll = _shift_hip_roll(_weight_shift(gait, env.model), mpc, side)
            if reference_roll is None:
                sagittal = ik.solve(sagittal_target, env, side=side)
                sagittal_ankle = ik.ankle_position(env, side, sagittal)
            else:
                sagittal, sagittal_ankle = ik.ankle_at_hip_roll(
                    sagittal_target, env, side, reference_roll
                )
            desired = sagittal_ankle.copy()
            desired[1] += lateral_correction
            # The 4 cm clamp is already applied to ``lateral_correction``,
            # measured from the weight-shift ankle. The pitch plane of the
            # measured leg can sit a few millimetres away from that ankle.
            # The solver's own gate has to cover that gap, or it rejects the
            # clamped target and the foot never leaves the uncorrected path.
            plane = ik.ankle_position(env, side, ik.solve(sagittal_target, env, side=side))
            needed = abs(float(desired[1] - plane[1]))
            limit = float(gait.config.max_lateral_step_adjustment)
            solved = ik.solve_lateral(
                desired,
                env,
                side=side,
                max_lateral=max(limit, min(needed, limit + 0.02)),
            )
            pose[f"{side}_hip_pitch"] = sagittal.hip_pitch
            pose[f"{side}_knee"] = sagittal.knee
            pose[f"{side}_ankle"] = sagittal.ankle_pitch
            # The existing pitch solver runs again in the new hip-roll plane
            # so the ankle keeps the sagittal x and z. A later miss holds the
            # last hip-roll command so the joint does not jump back to the
            # shift blend.
            if solved.reachable and solved.hip_roll is not None:
                pose[f"{side}_hip_pitch"] = solved.hip_pitch
                pose[f"{side}_knee"] = solved.knee
                pose[f"{side}_ankle"] = solved.ankle_pitch
                pose[f"{side}_hip_roll"] = float(solved.hip_roll)
                placed_side = side
                applied_y = float(desired[1])
            else:
                held_roll = getattr(gait, "_swing_command", {}).get(f"{side}_hip_roll")
                if held_roll is not None:
                    pose[f"{side}_hip_roll"] = float(held_roll)
                    placed_side = side
                applied_y = float(sagittal_ankle[1])
            measured, _rotation = _ankle_pose(env, side)
            lateral_log = {
                "desired_foot_x": float(desired[0]),
                "desired_foot_y": float(desired[1]),
                "desired_foot_z": float(desired[2]),
                "current_foot_x": float(measured[0]),
                "current_foot_y": float(measured[1]),
                "current_foot_z": float(measured[2]),
                "applied_foot_y": applied_y,
                "lateral_ik_correction": float(solved.lateral_correction),
                "ik_reachable": 1.0 if solved.reachable else 0.0,
                "ik_failure_reason": solved.failure_reason,
                "hip_roll_target": float(pose.get(f"{side}_hip_roll", np.nan)),
                "hip_yaw_target": float(solved.hip_yaw) if solved.hip_yaw is not None else np.nan,
                "ankle_roll_target": np.nan,
            }
        else:
            joints = ik.solve(target, env, side=side)
            pose[f"{side}_hip_pitch"] = joints.hip_pitch
            pose[f"{side}_knee"] = joints.knee
            pose[f"{side}_ankle"] = joints.ankle_pitch
            if swing == side:
                measured, _rotation = _ankle_pose(env, side)
                lateral_log = {
                    "desired_foot_x": float(target[0]),
                    "desired_foot_y": float(target[1]),
                    "desired_foot_z": float(target[2]),
                    "current_foot_x": float(measured[0]),
                    "current_foot_y": float(measured[1]),
                    "current_foot_z": float(measured[2]),
                    "applied_foot_y": float(target[1]),
                    "lateral_ik_correction": 0.0,
                    "ik_reachable": np.nan,
                    "ik_failure_reason": "",
                    "hip_roll_target": np.nan,
                    "hip_yaw_target": np.nan,
                    "ankle_roll_target": np.nan,
                }
    gait._placed_hip_roll_side = placed_side
    gait._lateral_ik_log = lateral_log
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
    shift.reference_speed = float(gait.config.recovery_reference_speed)
    if phase in (WalkState.TOUCHDOWN_PREP, WalkState.TOUCHDOWN):
        shift.mode = "prep"
        shift.capture_trim = False
    elif phase is WalkState.DOUBLE_SUPPORT_RECOVERY:
        shift.mode = "recovery"
        shift.capture_trim = False
        shift.recovery_target = support_box_center(polygon, float(gait.config.capture_margin))
    else:
        shift.mode = "shift"
    old_side = shift.side if shift.side in ("left", "right") else ""
    new_side = active if active in ("left", "right") else ""
    if (
        phase is WalkState.DOUBLE_SUPPORT_RECOVERY
        and old_side
        and new_side
        and old_side != new_side
    ):
        old_y = float(np.mean(sole_corners(env, old_side)[:, 1]))
        new_y = float(np.mean(sole_corners(env, new_side)[:, 1]))
        transfer = plan_load_transfer(
            state=phase.value,
            time_since=float(gait._recover_time),
            duration=float(gait.config.load_transfer_duration),
            old_y=old_y,
            new_y=new_y,
            reference_x=float(shift.recovery_target[0]),
            reference_y=float(shift.recovery_target[1]),
            max_com_offset=float(gait.config.load_transfer_max_com_offset),
            max_hip_roll=float(gait.config.load_transfer_max_hip_roll),
        )
        # Keep the forward component from the support-box centre. Only the
        # lateral component follows the load schedule.
        shift.recovery_target[1] = transfer.target_y
    else:
        transfer = plan_load_transfer(
            state="",
            time_since=0.0,
            duration=float(gait.config.load_transfer_duration),
            old_y=0.0,
            new_y=0.0,
            reference_x=float(shift.recovery_target[0]) if shift.recovery_target is not None else 0.0,
            reference_y=float(shift.recovery_target[1]) if shift.recovery_target is not None else 0.0,
            max_com_offset=float(gait.config.load_transfer_max_com_offset),
            max_hip_roll=float(gait.config.load_transfer_max_hip_roll),
        )
    shift.load_transfer_hip_roll = transfer.hip_roll
    shift.load_transfer_hip_roll_limit = float(gait.config.load_transfer_max_hip_roll)
    if loaded is not None:
        sole = sole_corners(env, loaded)
        if shift.side != loaded:
            left_ankle, _left_rot = _ankle_pose(env, "left")
            right_ankle, _right_rot = _ankle_pose(env, "right")
            home = {name: float(q) for name, q in zip(mpc._joint_names, mpc._home_joint_qpos)}
            shift.begin(loaded, left_ankle, right_ankle, state_now["com"], sole, home)
        # The foot that just finished its trajectory is held on the command
        # the ramp reached. Clearing skip_side here lets the shift ankle law
        # rewrite that foot on the first TOUCHDOWN tick.
        if phase is WalkState.TOUCHDOWN and active in ("left", "right"):
            shift.skip_side = active
        else:
            shift.skip_side = ""
        shift.hold_landed = False
        # Armed only while a foot is in the air. advance() still ignores it
        # until the capture point is inside 5 cm of the outer sole edge.
        shift.capture_trim = bool(late_swing_capture) and swing is not None and shift.mode != "prep"
        shift.preparation_hold = _fall_holds_lean(gait, phase, active)
        ref_xy = shift.advance(
            mpc.config.dt, state_now["com"], state_now["com_velocity"], sole
        )
        com[:, 0] = ref_xy[0]
        com[:, 1] = ref_xy[1]
        reference["com"] = com
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
        shift.capture_trim = False
        if shift.mode == "recovery":
            shift.hold_landed = False
            shift.preparation_hold = _fall_holds_lean(gait, phase, active)
            ref_xy = shift.advance(
                mpc.config.dt, state_now["com"], state_now["com_velocity"], sole
            )
        elif landing_handoff:
            # Keep the reference that was active at touchdown. Calling
            # advance() again would clip it onto the outward-moving centre
            # of mass, which is how the held lean walked off the sole.
            # The landed pitch, knee and ankle stay on the command the swing
            # actually reached; the capture offset is not added on top.
            shift.hold_landed = True
            if shift.handoff_xy is None:
                ref_xy = shift.advance(
                    mpc.config.dt, state_now["com"], state_now["com_velocity"], sole
                )
                shift.handoff_xy = np.asarray(ref_xy, dtype=float).copy()
            else:
                shift._com = np.asarray(state_now["com"], dtype=float).copy()
                ref_xy = np.asarray(shift.handoff_xy, dtype=float).copy()
            speed = np.asarray(state_now["com_velocity"], dtype=float).reshape(-1)
            if abs(float(speed[0])) < 0.02 and abs(float(speed[1])) < 0.02:
                left_xy, _left_rot = _ankle_pose(env, "left")
                right_xy, _right_rot = _ankle_pose(env, "right")
                mid_y = 0.5 * (float(left_xy[1]) + float(right_xy[1]))
                shift.handoff_xy[1] += float(
                    np.clip(mid_y - float(shift.handoff_xy[1]), -0.002, 0.002)
                )
                ref_xy = np.asarray(shift.handoff_xy, dtype=float).copy()
        else:
            shift.hold_landed = False
            ref_xy = shift.advance(
                mpc.config.dt, state_now["com"], state_now["com_velocity"], sole
            )
        com[:, 0] = ref_xy[0]
        com[:, 1] = ref_xy[1]
        reference["com"] = com
        active_posture = shift
        if shift.mode == "recovery":
            stance_sole = np.asarray(polygon, dtype=float)
        else:
            stance_sole = support_polygon(env, ("left", "right"))
        stance_xy = shift.target_xy.copy()
    else:
        active_posture = None
        both = support_polygon(env, ("left", "right"))
        stance_sole = both
        stance_xy = both.mean(axis=0)
    target_y = float(getattr(gait, "_transition_capture_y", np.nan))
    capture_target = None
    if np.isfinite(target_y):
        capture_target = np.array([float(state_now["com"][0]), target_y], dtype=float)
    _apply_capture_preparation(
        gait, env, shift, reference, state_now, phase, active, mpc.config.dt, capture_target
    )
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
        if f"{swing}_hip_roll" in swing_pose:
            gait._lateral_ik_log["swing_pose_hip_roll"] = float(swing_pose[f"{swing}_hip_roll"])
    else:
        gait._swing_command = {}
    # The touchdown policy, when one is loaded, may add a clipped centre-of-mass
    # residual during late swing. With no agent the reference is unchanged.
    # The copy below is the reference before that residual, kept for the log.
    deterministic_com = np.asarray(reference["com"], dtype=float)
    if deterministic_com.ndim == 2:
        deterministic_com = deterministic_com[0]
    deterministic_com = deterministic_com.reshape(3).copy()
    rl_log = {
        "rl_active": 0.0,
        "rl_action": np.zeros(2),
        "rl_residual": np.zeros(2),
        "rl_inference_s": 0.0,
        "touchdown_detected": 0.0,
        "stable_double_support": 0.0,
    }
    if touchdown_agent is not None:
        reference, rl_log = touchdown_agent.correct(
            env, reference, gait, cbf, command, swing, state_now, rows, polygon
        )
    landed_side = ""
    if phase in (WalkState.TOUCHDOWN, WalkState.DOUBLE_SUPPORT_RECOVERY) and active in (
        "left",
        "right",
    ):
        landed_side = active
    shift.hold_touchdown_hip_roll = bool(gait.config.hold_touchdown_hip_roll)
    shift.placed_hip_roll_side = str(getattr(gait, "_placed_hip_roll_side", ""))
    fall_hip = _fall_hip_command(
        gait, shift, mpc, env, phase, active, capture_pre, float(mpc.config.dt)
    )
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
        landed_side=landed_side,
        landed_ankle_capture=bool(gait.config.landed_ankle_capture),
        stance_hip_roll=fall_hip,
    )
    row["swing_side"] = swing or ""
    row["stance_foot_xy"] = np.asarray(stance_xy, dtype=float).reshape(2)
    row["support_distance"] = support_distance(row["com"][:2], stance_sole)
    row["shift_fraction"] = float(shift.fraction)
    row["shift_limit"] = shift.limit
    row["posture_active"] = bool(active_posture is not None)
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
    stance_side = stance[0] if stance else (shift.side or "")
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
    row["pelvis_roll"] = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
    row["com_reference_deterministic"] = deterministic_com
    row["loaded_foot"] = gait.loaded_foot or ""
    duration = float(gait.config.step_duration)
    if (
        command.state in (
            WalkState.RIGHT_SWING,
            WalkState.LEFT_SWING,
            WalkState.TOUCHDOWN_PREP,
            WalkState.TOUCHDOWN,
        )
        and duration > 0.0
    ):
        row["swing_progress"] = float(np.clip(gait.swing_elapsed / duration, 0.0, 1.0))
    else:
        row["swing_progress"] = 0.0
    for key, value in getattr(gait, "_lateral_ik_log", _empty_lateral_log()).items():
        row[key] = value
    for key, value in getattr(gait, "_capture_log", {}).items():
        row[key] = value
    for key, value in getattr(gait, "_support_log", {}).items():
        row[key] = value
    for key, value in getattr(gait, "_fall_log", _empty_fall_log()).items():
        row[key] = value
    fall_side = str(getattr(gait, "_fall_side", ""))
    if fall_side in ("left", "right"):
        row["stance_hip_roll_measured"] = float(row[f"{fall_side}_hip_roll"])
    else:
        row["stance_hip_roll_measured"] = float("nan")
    if active == "right":
        inner_margin = float(row.get("support_y_lower", np.nan))
    elif active == "left":
        inner_margin = float(row.get("support_y_upper", np.nan))
    else:
        inner_margin = float(
            min(row.get("support_y_lower", np.nan), row.get("support_y_upper", np.nan))
        )
    nominal_accel, safe_accel = _support_accelerations(cbf, row)
    row["cbf_support_margin"] = inner_margin
    row["cbf_nominal_capture_accel_y"] = float(nominal_accel)
    row["cbf_safe_capture_accel_y"] = float(safe_accel)
    row["cbf_nominal_acceleration_y"] = float(nominal_accel)
    row["cbf_safe_acceleration_y"] = float(safe_accel)
    row["cbf_intervention"] = float(row.get("intervention", 0.0))
    row["right_normal_force"] = float(normal_force["right"])
    row["left_normal_force"] = float(normal_force["left"])
    row["capture_point_y"] = float(capture[1])
    row["desired_capture_point_y"] = float(row.get("desired_capture_y", np.nan))
    row["capture_point_target_y"] = float(row.get("capture_point_target_y", np.nan))
    row["com_y"] = float(row["com"][1])
    row["com_velocity_y"] = float(row["com_velocity"][1])
    row["com_acceleration_y"] = float(np.asarray(row["com_acceleration"], dtype=float).reshape(3)[1])
    row["com_vy"] = float(row["com_velocity"][1])
    row["desired_com_y"] = float(np.asarray(row["com_reference"], dtype=float).reshape(3)[1])
    row["desired_com_velocity_y"] = float(row.get("desired_com_vy", 0.0))
    row["stance_foot"] = str(command.support_foot)
    row["swing_foot"] = active or ""
    row["original_swing_target_y"] = float(gait.original_swing_target_y)
    row["adaptive_target_y"] = float(gait.adaptive_target_y)
    row["adaptive_lateral_correction"] = float(gait.adaptive_lateral_correction)
    row["adaptive_requested_correction"] = float(gait.adaptive_requested_correction)
    row["adaptive_placement_fallback"] = float(gait.adaptive_placement_fallback)
    row["requested_lateral_correction"] = float(gait.adaptive_requested_correction)
    row["applied_lateral_correction"] = float(gait.adaptive_lateral_correction)
    track_side = swing if swing in ("left", "right") else ""
    if (
        not track_side
        and active in ("left", "right")
        and command.state in (WalkState.RIGHT_SWING, WalkState.LEFT_SWING, WalkState.TOUCHDOWN_PREP)
    ):
        track_side = active
    if track_side in ("left", "right"):
        measured_y = float(row[f"{track_side}_foot_xyz"][1])
        ik_y = float(row.get("desired_foot_y", np.nan))
        row["ik_foot_y"] = ik_y
        row["measured_foot_y"] = measured_y
        row["lateral_tracking_error"] = float(ik_y - measured_y) if np.isfinite(ik_y) else np.nan
        row["q_des_hip_roll"] = float(row[f"{track_side}_hip_roll_des"])
        row["actual_hip_roll"] = float(row[f"{track_side}_hip_roll"])
        row["q_des_hip_pitch"] = float(row[f"{track_side}_hip_pitch_des"])
        row["q_des_knee"] = float(row[f"{track_side}_knee_des"])
        row["q_des_ankle"] = float(row[f"{track_side}_ankle_des"])
    else:
        row["ik_foot_y"] = np.nan
        row["measured_foot_y"] = np.nan
        row["lateral_tracking_error"] = np.nan
        row["q_des_hip_roll"] = np.nan
        row["actual_hip_roll"] = np.nan
        row["q_des_hip_pitch"] = np.nan
        row["q_des_knee"] = np.nan
        row["q_des_ankle"] = np.nan
    if command.support_foot == "left":
        row["stance_foot_y"] = float(row["left_foot_xyz"][1])
    elif command.support_foot == "right":
        row["stance_foot_y"] = float(row["right_foot_xyz"][1])
    else:
        row["stance_foot_y"] = float("nan")
    if active == "right":
        row["swing_foot_y"] = float(row["right_foot_xyz"][1])
    elif active == "left":
        row["swing_foot_y"] = float(row["left_foot_xyz"][1])
    else:
        row["swing_foot_y"] = float("nan")
    sole_y = np.concatenate(
        (sole_corners(env, "left")[:, 1], sole_corners(env, "right")[:, 1])
    )
    combined_margin = combined_y_margin(float(row["capture_xy"][1]), sole_y)
    row["combined_capture_margin"] = float(combined_margin)
    if row["walk_state"] == WalkState.TOUCHDOWN_PREP.value:
        gait._capture_margin_before = float(combined_margin)
    previous_phase = str(rows[-1]["walk_state"]) if rows else ""
    if (
        row["walk_state"] == WalkState.TOUCHDOWN.value
        and previous_phase == WalkState.TOUCHDOWN_PREP.value
    ):
        gait._capture_margin_after = float(combined_margin)
    row["capture_margin_before_touchdown"] = float(getattr(gait, "_capture_margin_before", np.nan))
    row["capture_margin_after_touchdown"] = float(getattr(gait, "_capture_margin_after", np.nan))
    row["swing_height_ref"] = float(gait.config.step_height)
    row["left_foot_des"] = np.asarray(command.left_foot, dtype=float).reshape(3).copy()
    row["right_foot_des"] = np.asarray(command.right_foot, dtype=float).reshape(3).copy()
    if swing == "right":
        swing_target = command.right_foot
    elif swing == "left":
        swing_target = command.left_foot
    else:
        swing_target = np.full(3, np.nan)
    row["swing_target"] = np.asarray(swing_target, dtype=float).reshape(3).copy()
    for joint_name, field in (
        ("hip_pitch", "swing_ik_hip_pitch"),
        ("knee", "swing_ik_knee"),
        ("ankle", "swing_ik_ankle"),
    ):
        if swing:
            row[field] = float(pose.get(f"{swing}_{joint_name}", np.nan))
        else:
            row[field] = np.nan
    previous_state = str(rows[-1]["walk_state"]) if rows else ""
    detected_now = float(
        row["walk_state"] == WalkState.DOUBLE_SUPPORT_RECOVERY.value
        and previous_state == WalkState.TOUCHDOWN.value
    )
    row["touchdown"] = detected_now
    margin_side = swing and ("left" if swing == "right" else "right")
    if not margin_side:
        margin_side = shift.side or ""
    if margin_side == "left":
        row["outer_support_margin"] = float(row.get("support_y_upper", np.nan))
        row["inner_support_margin"] = float(row.get("support_y_lower", np.nan))
        row["outer_capture_margin"] = float(row.get("capture_margin_y_upper", np.nan))
        row["inner_capture_margin"] = float(row.get("capture_margin_y_lower", np.nan))
    elif margin_side == "right":
        row["outer_support_margin"] = float(row.get("support_y_lower", np.nan))
        row["inner_support_margin"] = float(row.get("support_y_upper", np.nan))
        row["outer_capture_margin"] = float(row.get("capture_margin_y_lower", np.nan))
        row["inner_capture_margin"] = float(row.get("capture_margin_y_upper", np.nan))
    else:
        row["outer_support_margin"] = np.nan
        row["inner_support_margin"] = np.nan
        row["outer_capture_margin"] = np.nan
        row["inner_capture_margin"] = np.nan
    row["rear_capture_margin"] = float(row.get("capture_margin_x_lower", np.nan))
    row["front_capture_margin"] = float(row.get("capture_margin_x_upper", np.nan))
    stance_xy_now = np.asarray(row["stance_foot_xy"], dtype=float).reshape(2)
    row["com_to_stance"] = float(np.linalg.norm(np.asarray(row["com"][:2], dtype=float) - stance_xy_now))
    row["push_active"] = 0.0
    row["push_force"] = np.zeros(3)
    row["touchdown_confirm_count"] = float(gait.touchdown_confirm_count)
    if landed_side:
        ankle_index = names.index(f"{landed_side}_ankle")
        row["landing_ankle_before_capture"] = float(mpc._home_joint_qpos[ankle_index])
        row["landed_ankle_target"] = float(row["q_des"][ankle_index])
    else:
        row["landing_ankle_before_capture"] = np.nan
        row["landed_ankle_target"] = np.nan
    row["sagittal_capture_offset"] = float(row.get("sagittal_capture_offset", 0.0))
    row["landed_ankle_capture_suppressed"] = float(row.get("landed_ankle_capture_suppressed", 0.0))
    support_points = np.asarray(polygon, dtype=float).reshape(-1, 2)
    row["support_low"] = support_points.min(axis=0)
    row["support_high"] = support_points.max(axis=0)
    row["left_contact"] = float(contacts["left"])
    row["right_contact"] = float(contacts["right"])
    row["left_foot_normal_force"] = float(normal_force["left"])
    row["right_foot_normal_force"] = float(normal_force["right"])
    row["mpc_support_feet"] = mpc_support_feet
    row["cbf_support_feet"] = cbf_support_feet
    row["landed_foot"] = landed_side
    if landed_side:
        row["landed_foot_z"] = float(row[f"{landed_side}_foot_xyz"][2])
        row["landed_foot_vz"] = float(row[f"{landed_side}_foot_vel"][2])
        row["landed_foot_normal_force"] = float(normal_force[landed_side])
    else:
        row["landed_foot_z"] = np.nan
        row["landed_foot_vz"] = np.nan
        row["landed_foot_normal_force"] = np.nan
    captures = getattr(shift, "hip_roll_capture", {})
    available = getattr(shift, "hip_roll_capture_available", {})
    row["left_hip_roll_capture"] = float(captures.get("left_hip_roll", 0.0))
    row["right_hip_roll_capture"] = float(captures.get("right_hip_roll", 0.0))
    row["left_hip_roll_capture_available"] = float(available.get("left_hip_roll", 0.0))
    row["right_hip_roll_capture_available"] = float(available.get("right_hip_roll", 0.0))
    row["hold_touchdown_hip_roll"] = float(getattr(shift, "hold_touchdown_hip_roll", False))
    row["old_stance_foot"] = old_side if transfer.active else ""
    row["new_stance_foot"] = new_side if transfer.active else ""
    if transfer.active and old_side and new_side:
        row["old_foot_contact"] = float(contacts[old_side])
        row["new_foot_contact"] = float(contacts[new_side])
        row["old_normal_force"] = float(normal_force[old_side])
        row["new_normal_force"] = float(normal_force[new_side])
    else:
        row["old_foot_contact"] = 0.0
        row["new_foot_contact"] = 0.0
        row["old_normal_force"] = np.nan
        row["new_normal_force"] = np.nan
    row["load_transfer_alpha"] = float(transfer.alpha)
    row["desired_new_load_fraction"] = float(transfer.new_fraction)
    row["desired_old_load_fraction"] = float(transfer.old_fraction)
    row["target_com_x"] = float(transfer.target_x)
    row["target_com_y"] = float(transfer.target_y)
    row["load_transfer_hip_roll"] = float(transfer.hip_roll)
    row["load_transfer_complete"] = float(transfer.complete)
    low = support_points.min(axis=0) + float(gait.config.capture_margin)
    high = support_points.max(axis=0) - float(gait.config.capture_margin)
    capture_now = np.asarray(row["capture_xy"], dtype=float).reshape(2)
    capture_inside = bool(
        np.all(high > low) and np.all(capture_now >= low) and np.all(capture_now <= high)
    )
    sample_ok = load_sample_ok(
        new_force=float(row["new_normal_force"]) if np.isfinite(row["new_normal_force"]) else 0.0,
        old_force=float(row["old_normal_force"]) if np.isfinite(row["old_normal_force"]) else 0.0,
        both_contact=bool(row["old_foot_contact"] and row["new_foot_contact"]),
        com_z=float(row["com"][2]),
        vy=float(row["com_velocity"][1]),
        capture_inside=capture_inside,
        cbf_status=str(row["cbf_status"]),
        min_fraction=float(gait.config.load_transfer_success_fraction),
        max_vy=float(gait.config.max_abs_vy),
    )
    tracker = getattr(gait, "_load_transfer", None)
    if tracker is None:
        tracker = LoadTransferTracker()
        gait._load_transfer = tracker
    tracker.observe(
        active=transfer.active,
        time=float(row["time"]),
        sample_ok=sample_ok,
        in_final_window=final_window(float(gait._recover_time), float(gait.config.load_transfer_duration)),
        dwell=float(gait.config.load_transfer_success_dwell),
        dt=float(mpc.config.dt),
    )
    row["load_transfer_start_time"] = float(tracker.start_time)
    row["load_transfer_success"] = float(tracker.success)
    row["load_transfer_sample_ok"] = float(sample_ok)
    row["left_foot_vz"] = float(row["left_foot_vel"][2])
    row["right_foot_vz"] = float(row["right_foot_vel"][2])
    if row["walk_state"] == WalkState.DOUBLE_SUPPORT_RECOVERY.value:
        row["recovery_com_reference"] = np.asarray(row["com_reference"], dtype=float).reshape(3).copy()
    else:
        row["recovery_com_reference"] = np.full(3, np.nan)
    row.update(rl_log)
    if touchdown_agent is not None:
        parts = touchdown_agent.update_after_step(row)
        row["rl_reward"] = float(parts.get("total", 0.0))
        row["rl_reward_components"] = parts
        row["stable_double_support"] = float(touchdown_agent.stable)
        row["episode_failure_reason"] = touchdown_agent.reason
    # The gait's contact confirmation is the touchdown record. An optional
    # residual policy does not replace it.
    row["touchdown_detected"] = detected_now
    if touchdown_agent is None:
        row["stable_double_support"] = float(transfer.active and sample_ok and tracker.success)
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


def _foot_normal_force(env: IndustrialHumanoidEnv, side: str) -> float:
    """Sum of MuJoCo contact-frame normal forces on ``side``.

    The H1 sole capsules are unnamed children of the ankle body, so a contact
    counts when its geom name starts with the side or its body is that ankle.
    The value is the compressive normal reported by ``mj_contactForce``.
    """
    import mujoco

    body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_link")
    total = 0.0
    wrench = np.zeros(6)
    for index in range(int(env.data.ncon)):
        contact = env.data.contact[index]
        if float(contact.dist) > 1e-3:
            continue
        geoms = (int(contact.geom1), int(contact.geom2))
        names = [
            mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_GEOM, geom) or "" for geom in geoms
        ]
        bodies = [int(env.model.geom_bodyid[geom]) for geom in geoms]
        named = [name.startswith(f"{side}_") for name in names]
        owned = [body == body_id for body in bodies] if body_id >= 0 else [False, False]
        own = [name_hit or body_hit for name_hit, body_hit in zip(named, owned)]
        if any(own) and not all(own):
            mujoco.mj_contactForce(env.model, env.data, index, wrench)
            total += float(wrench[0])
    return total


def _foot_in_contact(env: IndustrialHumanoidEnv, side: str) -> bool:
    """True when ``side`` is touching something that is not itself.

    ``foot_in_contact`` matches geoms whose names start with the foot side.
    The H1 sole capsules are unnamed children of the ankle body, so a contact
    on that body counts as well.
    """
    from rl.touchdown import foot_in_contact

    if foot_in_contact(env, side):
        return True
    import mujoco

    body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_link")
    if body_id < 0:
        return False
    for index in range(int(env.data.ncon)):
        contact = env.data.contact[index]
        if float(contact.dist) > 1e-3:
            continue
        bodies = [int(env.model.geom_bodyid[int(geom)]) for geom in (contact.geom1, contact.geom2)]
        own = [body == body_id for body in bodies]
        if any(own) and not all(own):
            return True
    return False


def _settled_touchdown(env: IndustrialHumanoidEnv, side: str, config: WalkConfig) -> bool:
    """Contact, ankle height, and vertical speed for one confirmation sample."""
    position, velocity = _body_pose_and_velocity(env, f"{side}_ankle_link")
    return touchdown_sample_ok(
        in_contact=_foot_in_contact(env, side),
        foot_z=float(position[2]),
        foot_vz=float(velocity[2]),
        height_limit=float(config.touchdown_height),
        vz_min=float(config.touchdown_vz_min),
    )


def _recovery_sample_ok(
    env: IndustrialHumanoidEnv,
    contacts: dict[str, bool],
    config: WalkConfig,
) -> bool:
    """One tick of the recovery exit conditions, using contact-qualified soles."""
    soles = {side: sole_corners(env, side) for side in ("left", "right")}
    corners = contact_qualified_corners(soles, contacts)
    if corners.shape[0] == 0:
        return False
    state = env.get_state()
    com = np.asarray(state["com"], dtype=float).reshape(3)
    vel = np.asarray(state["com_velocity"], dtype=float).reshape(3)
    omega = float(np.sqrt(9.81 / max(float(com[2]), 0.2)))
    capture = com[:2] + vel[:2] / omega
    roll, pitch, _yaw = _rpy(state["base_rotation"])
    return recovery_conditions_met(
        both_contact=bool(contacts.get("left") and contacts.get("right")),
        com_xy=com[:2],
        capture_xy=capture,
        box_low=corners.min(axis=0),
        box_high=corners.max(axis=0),
        vx=float(vel[0]),
        vy=float(vel[1]),
        roll=float(roll),
        pitch=float(pitch),
        margin=float(config.capture_margin),
        max_vx=float(config.max_abs_vx),
        max_vy=float(config.max_abs_vy),
        max_roll=float(config.max_abs_roll),
        max_pitch=float(config.max_abs_pitch),
    )


def _empty_fall_log() -> dict[str, float]:
    return {
        "stance_hip_roll_base": float("nan"),
        "stance_hip_roll_capture_target": float("nan"),
        "stance_hip_roll_final_target": float("nan"),
        "capture_point_target_y": float("nan"),
        "double_support_capture_margin": float("nan"),
        "release_capture_correction": 0.0,
    }


def _fall_holds_lean(gait: WalkingStateMachine, phase: WalkState, swing_side: str | None) -> bool:
    """True while the weight-shift lean must stay latched for the fall."""
    in_fall = (
        swing_side in ("left", "right")
        and phase in _PREPARATION_STATES + (WalkState.DOUBLE_SUPPORT_RECOVERY,)
    )
    release = getattr(gait, "_fall_release", None)
    released = release is not None and release.started and release.blend >= 1.0
    latched = str(getattr(gait, "_fall_base_side", "")) in ("left", "right")
    return bool((in_fall or latched) and not released)


def _fall_hip_command(
    gait: WalkingStateMachine,
    shift: LateralWeightShift,
    mpc: WholeBodyMPC,
    env: IndustrialHumanoidEnv,
    phase: WalkState,
    swing_side: str | None,
    capture_xy: np.ndarray,
    dt: float,
) -> tuple[str, float] | None:
    """Stance hip target for this tick, or None outside a step.

    The value is written onto q_des after the shift and the swing IK, so it
    is the last stance-hip command before the PD.
    """
    release = getattr(gait, "_fall_release", None)
    if not isinstance(release, FallRelease):
        release = FallRelease()
        gait._fall_release = release
    fall_side = "left" if swing_side == "right" else "right" if swing_side == "left" else ""
    in_fall = fall_side in ("left", "right") and phase in _PREPARATION_STATES + (
        WalkState.DOUBLE_SUPPORT_RECOVERY,
    )
    # Recovery can return to stand before the new foot is loaded. The inward
    # target stays until the release conditions are met. Handing the hip back
    # to the weight-shift blend at that moment puts the old lean on again.
    latched = str(getattr(gait, "_fall_base_side", "")) in ("left", "right")
    if not in_fall and not latched:
        release.update(in_recovery=False, capture_margin=0.0, new_foot_force=0.0, dt=dt)
        gait._fall_base_side = ""
        gait._fall_side = ""
        gait._fall_seen_recovery = False
        gait._fall_log = _empty_fall_log()
        return None
    if not in_fall:
        fall_side = str(gait._fall_base_side)
    if getattr(gait, "_fall_base_side", "") != fall_side:
        base = _shift_hip_roll(shift, mpc, fall_side)
        if base is None:
            gait._fall_log = _empty_fall_log()
            return None
        gait._fall_hip_base = float(base)
        gait._fall_hip_home = float(
            mpc._home_joint_qpos[list(mpc._joint_names).index(f"{fall_side}_hip_roll")]
        )
        gait._fall_base_side = fall_side
        gait._fall_new_foot = "right" if fall_side == "left" else "left"
    if phase is WalkState.DOUBLE_SUPPORT_RECOVERY:
        gait._fall_seen_recovery = True
    both = _foot_in_contact(env, "left") and _foot_in_contact(env, "right")
    if both:
        margin = capture_box_margin(capture_xy, support_polygon(env, ("left", "right")))
    else:
        margin = -1.0
    new_foot = swing_side if swing_side in ("left", "right") else str(getattr(gait, "_fall_new_foot", ""))
    force = _foot_normal_force(env, new_foot) if new_foot in ("left", "right") else 0.0
    blend = release.update(
        in_recovery=phase is WalkState.DOUBLE_SUPPORT_RECOVERY or bool(getattr(gait, "_fall_seen_recovery", False)),
        capture_margin=margin,
        new_foot_force=force,
        dt=dt,
    )
    weight = float(getattr(gait, "_fall_opening", 0.0)) if in_fall else 1.0
    angle = stance_hip_roll_capture_target(
        float(gait._fall_hip_base),
        float(capture_xy[1]),
        TRANSITION_Y,
        weight,
        gain=float(mpc.config.capture_gain),
        home=float(gait._fall_hip_home),
        release=blend,
    )
    full = stance_hip_roll_capture_target(
        float(gait._fall_hip_base),
        float(capture_xy[1]),
        TRANSITION_Y,
        1.0,
        gain=float(mpc.config.capture_gain),
    )
    gait._fall_log = {
        "stance_hip_roll_base": float(gait._fall_hip_base),
        "stance_hip_roll_capture_target": float(full),
        "stance_hip_roll_final_target": float(angle),
        "capture_point_target_y": float(TRANSITION_Y) if weight > 0.0 else float("nan"),
        "double_support_capture_margin": float(margin) if both else float("nan"),
        "release_capture_correction": float(blend),
    }
    gait._fall_side = fall_side
    return fall_side, float(angle)


def _shift_hip_roll(shift: LateralWeightShift, mpc: WholeBodyMPC, side: str) -> float | None:
    """Hip roll the weight shift would command for ``side``, before lateral IK.

    This is the blend ``apply`` writes during the shift and during swing.
    ``None`` when that posture has not been solved yet.
    """
    name = f"{side}_hip_roll"
    if not shift.solved or shift.fraction <= 0.0 or name not in shift.pose:
        return None
    index = list(mpc._joint_names).index(name)
    home = float(mpc._home_joint_qpos[index])
    fraction = float(shift.fraction)
    return (1.0 - fraction) * home + fraction * float(shift.pose[name])


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


def save_log(path: Path, rows: list[dict], metadata: dict | None = None) -> None:
    """Write the rollout. The NPZ keeps every numeric field on the rows."""
    path.parent.mkdir(parents=True, exist_ok=True)
    write_experiment_log(path, rows, metadata)
    time_s = np.array([row["time"] for row in rows])
    com = np.vstack([row["com"] for row in rows])
    com_reference = np.vstack([row["com_reference"] for row in rows])
    com_error = np.vstack([row["com_error"] for row in rows])
    com_velocity = np.vstack([row["com_velocity"] for row in rows])
    intervention = np.array([row["intervention"] for row in rows])
    status = np.array([row["cbf_status"] for row in rows])
    walk_state = np.array([row.get("walk_state", "STAND") for row in rows])
    support_foot = np.array([row.get("support_foot", "both") for row in rows])
    stance_foot_xy = np.vstack(
        [np.asarray(row.get("stance_foot_xy", [0.0, 0.0]), dtype=float).reshape(2) for row in rows]
    )
    support_gap = np.array([float(row.get("support_distance", 0.0)) for row in rows])
    rl_active = np.array([float(row.get("rl_active", 0.0)) for row in rows])
    rl_residual = np.vstack(
        [np.asarray(row.get("rl_residual", [0.0, 0.0]), dtype=float).reshape(2) for row in rows]
    )
    csv_path = path.with_suffix(".csv")
    header = (
        "time,com_x,com_y,com_z,ref_x,ref_y,ref_z,err_x,err_y,err_z,"
        "com_vx,com_vy,com_vz,intervention,cbf_status,walk_state,support_foot,"
        "stance_x,stance_y,support_distance,rl_active,rl_dx,rl_dy"
    )
    lines = [header]
    for i in range(time_s.size):
        lines.append(
            f"{time_s[i]:.4f},{com[i, 0]:.6f},{com[i, 1]:.6f},{com[i, 2]:.6f},"
            f"{com_reference[i, 0]:.6f},{com_reference[i, 1]:.6f},{com_reference[i, 2]:.6f},"
            f"{com_error[i, 0]:.6f},{com_error[i, 1]:.6f},{com_error[i, 2]:.6f},"
            f"{com_velocity[i, 0]:.6f},{com_velocity[i, 1]:.6f},{com_velocity[i, 2]:.6f},"
            f"{intervention[i]:.6f},{status[i]},{walk_state[i]},{support_foot[i]},"
            f"{stance_foot_xy[i, 0]:.6f},{stance_foot_xy[i, 1]:.6f},{support_gap[i]:.6f},"
            f"{rl_active[i]:.0f},{rl_residual[i, 0]:.6f},{rl_residual[i, 1]:.6f}"
        )
    csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _run_metadata(args: argparse.Namespace, mpc: WholeBodyMPC, touchdown_agent) -> dict:
    """Identifiers and constants for the log header. Not a control input."""
    if bool(getattr(args, "rl_touchdown", False)):
        name = "touchdown_rl"
    elif not args.walk:
        name = "standing_push"
    elif int(args.steps) <= 1:
        name = "walking_single_step"
    else:
        name = "walking_multi_step"
    metadata = {
        "experiment": name,
        "seed": int(args.seed),
        "control_dt": float(args.control_dt),
        "horizon": int(args.horizon),
        "walk": bool(args.walk),
        "n_steps": int(args.steps) if args.walk else 0,
        "step_length": float(getattr(args, "step_length", 0.1)) if args.walk else None,
        "step_duration": float(getattr(args, "step_duration", 0.8)) if args.walk else None,
        "step_height": 0.05 if args.walk else None,
        "first_swing": "right" if args.walk else "",
        "push_force_command": np.asarray(args.push_force, dtype=float).tolist() if not args.walk else [0.0, 0.0, 0.0],
        "push_window_s": [PUSH_TIME, PUSH_TIME + PUSH_DURATION] if not args.walk else None,
        "joint_names": list(mpc._joint_names),
        "joint_low": np.asarray(mpc._joint_low, dtype=float),
        "joint_high": np.asarray(mpc._joint_high, dtype=float),
        "fall_height_m": 0.72,
        "adaptive_foot_placement": bool(getattr(args, "adaptive_foot_placement", False)),
        "capture_footstep_gain": float(getattr(args, "capture_footstep_gain", 0.5)),
        "max_lateral_step_adjustment": float(getattr(args, "max_lateral_step_adjustment", 0.04)),
    }
    if touchdown_agent is not None:
        metadata["residual_bound"] = np.asarray(touchdown_agent.config.bounds(), dtype=float)
        metadata["rl_model"] = str(getattr(args, "rl_model", ""))
    return metadata


def _finish_log(
    args: argparse.Namespace,
    env: IndustrialHumanoidEnv,
    log_path: Path,
    rows: list[dict],
    metadata: dict | None = None,
) -> None:
    """Write the demo log, and a swing summary when this run was walking."""
    try:
        if not rows:
            return
        save_log(log_path, rows, metadata)
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
        adaptive_placement = bool(getattr(args, "adaptive_foot_placement", False))
        # Support transition owns the step into double support. The separate
        # load-transfer offsets stay at zero for this walk.
        gait = WalkingStateMachine(
            WalkConfig(
                step_length=float(getattr(args, "step_length", 0.1)),
                step_height=0.05,
                step_duration=float(getattr(args, "step_duration", 0.8)),
                shift_duration=1.5,
                double_support_duration=0.4,
                stand_duration=1.5,
                hold_touchdown_hip_roll=bool(getattr(args, "hold_touchdown_hip_roll", False)),
                load_transfer_duration=float(getattr(args, "load_transfer_duration", 0.40)),
                load_transfer_max_com_offset=0.0,
                load_transfer_max_hip_roll=0.0,
                adaptive_foot_placement=adaptive_placement,
                capture_footstep_gain=float(getattr(args, "capture_footstep_gain", 0.5)),
                max_lateral_step_adjustment=float(getattr(args, "max_lateral_step_adjustment", 0.04)),
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
    touchdown_agent = None
    if bool(getattr(args, "rl_touchdown", False)):
        model_path = str(getattr(args, "rl_model", "") or "")
        if not model_path:
            raise SystemExit("--rl-touchdown requires --rl-model PATH")
        from rl.touchdown import TouchdownAgent

        touchdown_agent = TouchdownAgent(deterministic=True)
        touchdown_agent.load(Path(model_path))
        LOG.info("touchdown policy loaded from %s", model_path)
    log_path = Path(args.log_path)
    log_meta = _run_metadata(args, mpc, touchdown_agent)
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
                late_swing_capture=bool(getattr(args, "late_swing_capture", False)),
                landing_handoff=bool(getattr(args, "landing_handoff", False)),
                touchdown_agent=touchdown_agent,
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
        row = control_step(env, mpc, cbf, pd, reference)
        active = PUSH_TIME <= float(sim_time) < PUSH_TIME + PUSH_DURATION
        row["push_active"] = float(active)
        row["push_force"] = np.asarray(args.push_force, dtype=float).copy() if active else np.zeros(3)
        return row

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
            _finish_log(args, env, log_path, rows, log_meta)
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
        _finish_log(args, env, log_path, rows, log_meta)
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
    parser.add_argument(
        "--hold-touchdown-hip-roll",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "experiment: keep the pre-touchdown hip-roll blend through recovery. "
            "The load-transfer offset is still added. Default leaves the lateral "
            "capture term on"
        ),
    )
    parser.add_argument(
        "--load-transfer-duration",
        type=float,
        default=0.40,
        help="nominal seconds to move the lateral reference toward the new foot",
    )
    parser.add_argument(
        "--load-transfer-max-com-offset",
        type=float,
        default=0.04,
        help="largest lateral recovery-reference move toward the new sole, in metres",
    )
    parser.add_argument(
        "--load-transfer-max-hip-roll",
        type=float,
        default=0.08,
        help="largest load-transfer hip-roll offset, in radians",
    )
    parser.add_argument(
        "--adaptive-foot-placement",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "late-swing lateral landing adjustment from the capture point. "
            "Default leaves the footstep landing unchanged. Enabling it also "
            "holds the load-transfer offsets at zero"
        ),
    )
    parser.add_argument(
        "--capture-footstep-gain",
        type=float,
        default=0.5,
        help="fraction of the sole-geometry lateral correction to apply",
    )
    parser.add_argument(
        "--max-lateral-step-adjustment",
        type=float,
        default=0.04,
        help="largest lateral change of the swing landing, in metres",
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
        "--late-swing-capture",
        action="store_true",
        help="ease the lean during swing only while the capture point closes on the outer sole edge",
    )
    parser.add_argument(
        "--landing-handoff",
        action="store_true",
        help="hold the touchdown reference and landed-leg command through early double support",
    )
    parser.add_argument(
        "--rl-touchdown",
        action="store_true",
        help="add the trained late-swing touchdown residual; requires --rl-model",
    )
    parser.add_argument(
        "--rl-model",
        type=str,
        default="",
        help="touchdown checkpoint (.pt) used with --rl-touchdown",
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
