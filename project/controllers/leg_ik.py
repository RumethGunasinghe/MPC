"""Analytical inverse kinematics for one Unitree H1 leg.

The H1 leg, from the MJCF, is

    hip yaw  ->  hip roll  ->  hip pitch  ->  knee  ->  ankle pitch

with the thigh and the shank each 0.4 m long. At zero joint angles both
links lie along the parent frame's -z axis, and every pitch joint rotates
about +y. This solver closes the three pitch joints. Hip yaw and hip roll
are left at whatever the current state already has: they set the plane the
pitch joints move in, and a foot target off that plane cannot be reached
by pitch alone.

Geometry
--------
In the hip-pitch parent frame (after the floating base, hip yaw and hip
roll), the ankle position produced by hip pitch ``q_h`` and knee ``q_k`` is

    x = -L1 sin(q_h) - L2 sin(q_h + q_k)
    z = -L1 cos(q_h) - L2 cos(q_h + q_k)

``L1`` is the thigh and ``L2`` the shank. Squaring and adding gives the
law of cosines

    cos(q_k) = (r^2 - L1^2 - L2^2) / (2 L1 L2),    r^2 = x^2 + z^2.

``q_k > 0`` bends the knee forward, which is the H1 standing posture
(home is hip pitch -0.4 rad, knee +0.8 rad). The opposite sign folds the
knee backward and is used only when the current knee is already on that
side of straight. Substituting ``q_k`` back into the two equations is a
linear system for ``sin(q_h)`` and ``cos(q_h)``:

    x cos(q_h) - z sin(q_h) = -L2 sin(q_k)
    x sin(q_h) + z cos(q_h) = -L1 - L2 cos(q_k).

Ankle pitch does not move the ankle point. It is set so the sole stays
level in the world:

    q_a = q_level - q_h - q_k

where ``q_level = atan2(u_x, u_z)`` and ``u`` is world +z written in the
hip-pitch parent frame. With an upright pelvis this is ``q_a = -(q_h+q_k)``,
which is the home ankle of -0.4 rad.

Lateral placement
-----------------
The H1 leg has hip yaw and hip roll. It has no ankle-roll joint. A lateral
ankle move is a hip-roll change that puts the target in the pitch plane,
followed by the same pitch solution. Hip yaw is left at the measured angle
so the foot is not given an extra yaw. Sole roll changes with the hip roll;
there is no ankle roll to cancel it. :meth:`LegIK.solve` is unchanged.
:meth:`LegIK.solve_lateral` is the only path that commands hip roll.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

_SIDES = ("left", "right")


@dataclass(frozen=True)
class LegJoints:
    """Pitch-joint solution for one leg, in radians.

    ``hip_roll`` and ``hip_yaw`` are set by :meth:`LegIK.solve_lateral`.
    ``solve`` leaves them as ``None`` and does not command them. ``ankle_roll``
    stays ``None``: the H1 model has no ankle-roll joint.
    """

    hip_pitch: float
    knee: float
    ankle_pitch: float
    reachable: bool
    hip_roll: float | None = None
    hip_yaw: float | None = None
    ankle_roll: float | None = None
    failure_reason: str = ""
    lateral_correction: float = 0.0


@dataclass(frozen=True)
class _LegModel:
    """Body and joint ids, link lengths and limits for one side."""

    side: str
    hip_pitch_body: int
    hip_parent_body: int
    hip_pitch_joint: int
    knee_joint: int
    ankle_joint: int
    thigh_length: float
    shank_length: float
    hip_range: tuple[float, float]
    knee_range: tuple[float, float]
    ankle_range: tuple[float, float]
    hip_yaw_body: int
    hip_roll_body: int
    ankle_body: int
    hip_yaw_joint: int
    hip_roll_joint: int
    hip_roll_range: tuple[float, float]
    pitch_offset_y: float
    hip_yaw_qadr: int
    hip_roll_qadr: int
    hip_pitch_qadr: int
    knee_qadr: int
    ankle_qadr: int


class LegIK:
    """Solve H1 hip pitch, knee and ankle pitch for a desired ankle position.

    Args:
        model: Compiled MuJoCo model of the Unitree H1. Link lengths and
            joint limits are read from it.
    """

    def __init__(self, model) -> None:
        import mujoco

        self._model = model
        self._legs = {side: self._index_leg(mujoco, model, side) for side in _SIDES}
        self._scratch = None

    def solve(
        self,
        desired_foot_position: np.ndarray,
        state,
        side: str = "left",
    ) -> LegJoints:
        """Joint angles that put this leg's ankle on ``desired_foot_position``.

        Args:
            desired_foot_position: World position of the ankle joint, in
                metres, shape (3,). The sole sits below this point; ankle
                pitch levels the sole but does not move the ankle.
            state: Current robot state. Either the env (``env.data`` is
                used) or a MuJoCo ``MjData`` after ``mj_forward``. Hip yaw,
                hip roll and the pelvis pose are taken from this state and
                are not modified.
            side: ``"left"`` or ``"right"``.

        Returns:
            Hip pitch, knee and ankle pitch in radians. ``reachable`` is
            false when the target is off the pitch plane, outside the
            leg's reach, or outside a joint limit. The returned angles are
            still the closest pose inside the limits.
        """
        if side not in self._legs:
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        target = np.asarray(desired_foot_position, dtype=float).reshape(3)
        data = _as_mjdata(state)
        leg = self._legs[side]

        parent = np.asarray(data.xmat[leg.hip_parent_body], dtype=float).reshape(3, 3)
        origin = np.asarray(data.xpos[leg.hip_pitch_body], dtype=float)
        local = parent.T @ (target - origin)
        # The knee and the ankle only move in the parent xz plane. ``local[1]``
        # is the lateral miss that hip roll would have to cover.
        lateral = float(local[1])
        x, z, on_reach = _clamp_reach(
            float(local[0]),
            float(local[2]),
            leg.thigh_length,
            leg.shank_length,
            leg.knee_range,
        )
        current_knee = _joint_position(self._model, data, leg.knee_joint)
        hip, knee, in_limits = _pitch_and_knee(
            x,
            z,
            leg.thigh_length,
            leg.shank_length,
            current_knee,
            leg.hip_range,
            leg.knee_range,
        )
        # World up, expressed in the hip-pitch parent frame. The angle about
        # +y that aligns the foot +z with this direction levels the sole.
        world_up = parent[2, :]
        level = float(np.arctan2(world_up[0], world_up[2]))
        raw_ankle = level - hip - knee
        ankle = _clip(raw_ankle, leg.ankle_range)
        ankle_in_limits = abs(ankle - raw_ankle) <= 1e-6
        reachable = on_reach and in_limits and ankle_in_limits and abs(lateral) <= 1e-2
        return LegJoints(
            hip_pitch=float(hip),
            knee=float(knee),
            ankle_pitch=float(ankle),
            reachable=bool(reachable),
        )

    def solve_lateral(
        self,
        desired_foot_position: np.ndarray,
        state,
        side: str = "left",
        *,
        max_lateral: float = 0.04,
    ) -> LegJoints:
        """Place the ankle at ``desired_foot_position``, including world y.

        The pitch, knee and ankle-pitch solution is the existing one, solved
        in the plane of the hip roll that puts the target there. Hip yaw is
        held at the measured angle. A request whose world-y change from the
        current pitch-plane ankle exceeds ``max_lateral`` is not executed:
        the returned angles are the current-roll solution and ``reachable``
        is false.

        Args:
            desired_foot_position: World ankle position, shape (3,).
            state: Env or MuJoCo ``MjData``, as for :meth:`solve`.
            side: ``"left"`` or ``"right"``.
            max_lateral: Largest accepted world-y displacement, in metres.

        Returns:
            Joints, with ``failure_reason`` empty when the target was reached.
            Reasons are ``lateral_limit``, ``hip_roll_limit``,
            ``no_solution``, ``sagittal_unreachable`` and ``residual``.
        """
        if side not in self._legs:
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        desired = np.asarray(desired_foot_position, dtype=float).reshape(3).copy()
        data = _as_mjdata(state)
        leg = self._legs[side]
        current_roll = _joint_position(self._model, data, leg.hip_roll_joint)
        current_yaw = _joint_position(self._model, data, leg.hip_yaw_joint)
        sagittal = self.solve(desired, data, side)
        current_ankle = np.asarray(data.xpos[leg.ankle_body], dtype=float).copy()
        # The existing solver drops the out-of-plane part. Its forward
        # position is the ankle the current hip roll would actually reach.
        reference = self._ankle_at(
            data, leg, current_roll, sagittal.hip_pitch, sagittal.knee, sagittal.ankle_pitch
        )
        # A target that is the measured ankle, or already the pitch-plane
        # solution, keeps the existing joints and the measured hip roll.
        if float(np.linalg.norm(desired - current_ankle)) <= 1e-4 or float(
            np.linalg.norm(desired - reference)
        ) <= 1e-4:
            return self._with_roll(sagittal, current_roll, current_yaw, "", 0.0, sagittal.reachable)
        lateral = float(desired[1] - reference[1])
        # One millimetre of slack: the in-plane projection of an off-plane
        # target is not exactly the caller's reference when the hip is rolled.
        if abs(lateral) > abs(float(max_lateral)) + 1e-3:
            return self._with_roll(sagittal, current_roll, current_yaw, "lateral_limit", 0.0, False)
        roll, reason = self._hip_roll_for_ankle(data, leg, desired, current_roll)
        if reason:
            return self._with_roll(sagittal, current_roll, current_yaw, reason, 0.0, False)
        pitched = self._solve_at_roll(data, leg, desired, roll, side)
        if not pitched.reachable:
            return self._with_roll(
                sagittal, current_roll, current_yaw, "sagittal_unreachable", 0.0, False
            )
        achieved = self._ankle_at(data, leg, roll, pitched.hip_pitch, pitched.knee, pitched.ankle_pitch)
        if float(np.linalg.norm(achieved - desired)) > 1e-3:
            return self._with_roll(sagittal, current_roll, current_yaw, "residual", 0.0, False)
        return self._with_roll(pitched, roll, current_yaw, "", lateral, True)

    def ankle_at_hip_roll(
        self,
        foot_target: np.ndarray,
        state,
        side: str,
        hip_roll: float,
    ) -> tuple[LegJoints, np.ndarray]:
        """Existing pitch solution and its ankle position at one hip roll.

        ``hip_roll`` is the roll the sagittal swing would use. The measured
        roll is not used, so a lateral offset is not added on top of itself.
        """
        if side not in self._legs:
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        data = _as_mjdata(state)
        leg = self._legs[side]
        target = np.asarray(foot_target, dtype=float).reshape(3)
        joints = self._solve_at_roll(data, leg, target, float(hip_roll), side)
        position = self._ankle_at(
            data, leg, float(hip_roll), joints.hip_pitch, joints.knee, joints.ankle_pitch
        )
        return joints, position

    def ankle_position(self, state, side: str, joints: LegJoints, hip_roll: float | None = None) -> np.ndarray:
        """World ankle position for ``joints`` at the given or measured hip roll.

        Hip yaw and the rest of the robot stay as they are in ``state``.
        """
        if side not in self._legs:
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        data = _as_mjdata(state)
        leg = self._legs[side]
        roll = (
            float(hip_roll)
            if hip_roll is not None
            else _joint_position(self._model, data, leg.hip_roll_joint)
        )
        return self._ankle_at(data, leg, roll, joints.hip_pitch, joints.knee, joints.ankle_pitch)

    def _with_roll(
        self,
        joints: LegJoints,
        hip_roll: float,
        hip_yaw: float,
        reason: str,
        lateral: float,
        reachable: bool,
    ) -> LegJoints:
        return LegJoints(
            hip_pitch=float(joints.hip_pitch),
            knee=float(joints.knee),
            ankle_pitch=float(joints.ankle_pitch),
            reachable=bool(reachable),
            hip_roll=float(hip_roll),
            hip_yaw=float(hip_yaw),
            ankle_roll=None,
            failure_reason=str(reason),
            lateral_correction=float(lateral),
        )

    def _solve_at_roll(self, data, leg: _LegModel, target: np.ndarray, hip_roll: float, side: str) -> LegJoints:
        """Existing pitch solution with the hip roll joint set to ``hip_roll``."""
        import mujoco

        scratch = self._scratch_data()
        scratch.qpos[:] = np.asarray(data.qpos, dtype=float)
        scratch.qpos[leg.hip_roll_qadr] = float(hip_roll)
        mujoco.mj_forward(self._model, scratch)
        return self.solve(target, scratch, side)

    def _hip_roll_for_ankle(
        self, data, leg: _LegModel, target: np.ndarray, current_roll: float
    ) -> tuple[float, str]:
        """Hip roll that puts ``target`` in the pitch plane. Yaw is not changed.

        The roll joint axis is +x in the hip-yaw frame. The hip-pitch origin
        sits at ``(0, pitch_offset_y, 0)`` in the roll frame, so the in-plane
        condition is ``v_y cos q + v_z sin q = pitch_offset_y``.
        """
        yaw = np.asarray(data.xmat[leg.hip_yaw_body], dtype=float).reshape(3, 3)
        anchor = np.asarray(data.xpos[leg.hip_roll_body], dtype=float)
        local = yaw.T @ (np.asarray(target, dtype=float).reshape(3) - anchor)
        horizontal = float(local[1])
        vertical = float(local[2])
        radius = float(np.hypot(horizontal, vertical))
        offset = float(leg.pitch_offset_y)
        if radius < 1e-8 or abs(offset) > radius + 1e-9:
            return float(current_roll), "no_solution"
        alpha = float(np.arctan2(vertical, horizontal))
        delta = float(np.arccos(float(np.clip(offset / radius, -1.0, 1.0))))
        candidates = (alpha + delta, alpha - delta)
        chosen = min(candidates, key=lambda angle: abs(_wrap(angle - current_roll)))
        chosen = _wrap(chosen)
        if not _inside(chosen, leg.hip_roll_range):
            return float(current_roll), "hip_roll_limit"
        return float(chosen), ""

    def _ankle_at(
        self,
        data,
        leg: _LegModel,
        hip_roll: float,
        hip_pitch: float,
        knee: float,
        ankle: float,
    ) -> np.ndarray:
        """World ankle position with those leg joints and the rest of ``data``."""
        import mujoco

        scratch = self._scratch_data()
        scratch.qpos[:] = np.asarray(data.qpos, dtype=float)
        scratch.qpos[leg.hip_roll_qadr] = float(hip_roll)
        scratch.qpos[leg.hip_pitch_qadr] = float(hip_pitch)
        scratch.qpos[leg.knee_qadr] = float(knee)
        scratch.qpos[leg.ankle_qadr] = float(ankle)
        mujoco.mj_forward(self._model, scratch)
        return np.asarray(scratch.xpos[leg.ankle_body], dtype=float).copy()

    def _scratch_data(self):
        if self._scratch is None:
            import mujoco

            self._scratch = mujoco.MjData(self._model)
        return self._scratch

    @staticmethod
    def _index_leg(mujoco, model, side: str) -> _LegModel:
        hip_body = _body_id(mujoco, model, f"{side}_hip_pitch_link")
        knee_body = _body_id(mujoco, model, f"{side}_knee_link")
        ankle_body = _body_id(mujoco, model, f"{side}_ankle_link")
        hip_joint = _joint_id(mujoco, model, f"{side}_hip_pitch")
        knee_joint = _joint_id(mujoco, model, f"{side}_knee")
        ankle_joint = _joint_id(mujoco, model, f"{side}_ankle")
        yaw_body = _body_id(mujoco, model, f"{side}_hip_yaw_link")
        roll_body = _body_id(mujoco, model, f"{side}_hip_roll_link")
        yaw_joint = _joint_id(mujoco, model, f"{side}_hip_yaw")
        roll_joint = _joint_id(mujoco, model, f"{side}_hip_roll")
        thigh = np.asarray(model.body_pos[knee_body], dtype=float)
        shank = np.asarray(model.body_pos[ankle_body], dtype=float)
        return _LegModel(
            side=side,
            hip_pitch_body=hip_body,
            hip_parent_body=int(model.body_parentid[hip_body]),
            hip_pitch_joint=hip_joint,
            knee_joint=knee_joint,
            ankle_joint=ankle_joint,
            thigh_length=float(np.linalg.norm(thigh)),
            shank_length=float(np.linalg.norm(shank)),
            hip_range=_joint_range(model, hip_joint),
            knee_range=_joint_range(model, knee_joint),
            ankle_range=_joint_range(model, ankle_joint),
            hip_yaw_body=yaw_body,
            hip_roll_body=roll_body,
            ankle_body=ankle_body,
            hip_yaw_joint=yaw_joint,
            hip_roll_joint=roll_joint,
            hip_roll_range=_joint_range(model, roll_joint),
            pitch_offset_y=float(model.body_pos[hip_body][1]),
            hip_yaw_qadr=int(model.jnt_qposadr[yaw_joint]),
            hip_roll_qadr=int(model.jnt_qposadr[roll_joint]),
            hip_pitch_qadr=int(model.jnt_qposadr[hip_joint]),
            knee_qadr=int(model.jnt_qposadr[knee_joint]),
            ankle_qadr=int(model.jnt_qposadr[ankle_joint]),
        )


def _pitch_and_knee(
    x: float,
    z: float,
    thigh: float,
    shank: float,
    current_knee: float,
    hip_range: tuple[float, float],
    knee_range: tuple[float, float],
) -> tuple[float, float, bool]:
    """Hip pitch and knee for an in-plane ankle target ``(x, z)``.

    Returns the angles and whether both lie inside their joint ranges
    before clipping.
    """
    r2 = x * x + z * z
    cos_knee = (r2 - thigh * thigh - shank * shank) / (2.0 * thigh * shank)
    cos_knee = float(np.clip(cos_knee, -1.0, 1.0))
    magnitude = float(np.arccos(cos_knee))
    candidates = []
    for knee in (magnitude, -magnitude):
        if not _inside(knee, knee_range):
            continue
        hip = _hip_pitch(x, z, knee, thigh, shank)
        inside = _inside(hip, hip_range)
        candidates.append((abs(knee - current_knee), inside, hip, knee))
    if not candidates:
        # Neither bend fits in the knee range. Use the bend nearest the
        # current knee and clip both joints.
        knee = magnitude if abs(magnitude - current_knee) <= abs(-magnitude - current_knee) else -magnitude
        hip = _hip_pitch(x, z, knee, thigh, shank)
        return _clip(hip, hip_range), _clip(knee, knee_range), False
    # Prefer a solution already inside the hip limits, then the one closest
    # to the knee the robot is holding.
    candidates.sort(key=lambda item: (not item[1], item[0]))
    _, inside, hip, knee = candidates[0]
    if inside:
        return hip, knee, True
    return _clip(hip, hip_range), knee, False


def _hip_pitch(x: float, z: float, knee: float, thigh: float, shank: float) -> float:
    """``q_h`` from the 2x2 system in the module docstring."""
    r2 = x * x + z * z
    if r2 < 1e-12:
        return 0.0
    sin_knee = float(np.sin(knee))
    cos_knee = float(np.cos(knee))
    cos_hip = (-x * shank * sin_knee - z * (thigh + shank * cos_knee)) / r2
    sin_hip = (-x * (thigh + shank * cos_knee) + z * shank * sin_knee) / r2
    return float(np.arctan2(sin_hip, cos_hip))


def _clamp_reach(
    x: float,
    z: float,
    thigh: float,
    shank: float,
    knee_range: tuple[float, float],
) -> tuple[float, float, bool]:
    """Pull ``(x, z)`` onto the annulus the knee limits can reach.

    The last flag is true when the point was already inside that annulus.
    """
    radius = float(np.hypot(x, z))
    # Reach is longest with the knee straight and shortest at the knee angle
    # farthest from zero. The limits are the extrema over the allowed range.
    radii = [
        _reach_radius(thigh, shank, knee_range[0]),
        _reach_radius(thigh, shank, knee_range[1]),
    ]
    if knee_range[0] <= 0.0 <= knee_range[1]:
        radii.append(thigh + shank)
    r_min = min(radii)
    r_max = max(radii)
    if radius < 1e-8:
        clamped = float(np.clip(r_min, r_min, r_max))
        return 0.0, -clamped, False
    clamped = float(np.clip(radius, r_min, r_max))
    scale = clamped / radius
    return x * scale, z * scale, abs(radius - clamped) <= 1e-4


def _reach_radius(thigh: float, shank: float, knee: float) -> float:
    """Hip-to-ankle distance at one knee angle."""
    cos_knee = float(np.cos(knee))
    return float(np.sqrt(max(0.0, thigh * thigh + shank * shank + 2.0 * thigh * shank * cos_knee)))


def _joint_position(model, data, joint_id: int) -> float:
    address = int(model.jnt_qposadr[joint_id])
    return float(data.qpos[address])


def _joint_range(model, joint_id: int) -> tuple[float, float]:
    low, high = np.asarray(model.jnt_range[joint_id], dtype=float)
    return float(low), float(high)


def _inside(value: float, limits: tuple[float, float], tol: float = 1e-6) -> bool:
    return limits[0] - tol <= value <= limits[1] + tol


def _clip(value: float, limits: tuple[float, float]) -> float:
    return float(np.clip(value, limits[0], limits[1]))


def _wrap(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _body_id(mujoco, model, name: str) -> int:
    body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
    if body_id < 0:
        raise ValueError(f"model has no body {name!r}")
    return body_id


def _joint_id(mujoco, model, name: str) -> int:
    joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
    if joint_id < 0:
        raise ValueError(f"model has no joint {name!r}")
    return joint_id


def _as_mjdata(state):
    """MjData from a MuJoCo state or from the humanoid env."""
    if hasattr(state, "xpos") and hasattr(state, "xmat") and hasattr(state, "qpos"):
        return state
    data = getattr(state, "data", None)
    if data is not None and hasattr(data, "xpos"):
        return data
    raise TypeError("state must be the humanoid env or a MuJoCo MjData")
