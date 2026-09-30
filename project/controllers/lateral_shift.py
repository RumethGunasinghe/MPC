"""Lateral weight shift onto a measured stance sole.

The walking tick used to hold the centre-of-mass reference on the midline and
overwrite one hip roll with a fixed angle. Both hip-roll axes point the same
way, and the capture law on the other hip is clipped to ±0.15 rad, which is
only enough to move the centre of mass a few centimetres. The stance foot is
planted, so that hip-roll command cannot slide the sole under the body. The
two commands cancel and the shift stops at about 3 cm, short of a sole whose
ankle is whatever distance MuJoCo reports (about 20 cm on the H1 home pose).

This module does not hard-code that separation. It reads the ankle positions,
puts the centre-of-mass target at the centre of the measured stance sole, and
solves the leg posture that reaches that target with both ankles kept at the
positions they have now. The posture is a blend from the home pose. The blend
advances only while the capture point is still inside the sole, so the lean
does not run off the foot. The swing is still someone else's decision: this
class never reports that the foot may lift.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

_LEG_JOINTS = (
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
)


class LateralWeightShift:
    """Move the centre of mass from double support onto one measured sole.

    ``begin`` is called when a weight shift starts. ``advance`` then rate-limits
    the blend, and ``apply`` writes the blended leg targets over the MPC's
    joint command. The MPC, the PD and the CBF are unchanged; this only
    replaces the leg position targets for the duration of the shift.
    """

    def __init__(self, model) -> None:
        self.model = model
        self.side: str | None = None
        self.fraction = 0.0
        self.origin_xy = np.zeros(2)
        self.target_xy = np.zeros(2)
        self.sole_low = np.zeros(2)
        self.sole_high = np.zeros(2)
        self.pose: dict[str, float] = {}
        self.solved = False
        self.solve_info: dict[str, float] = {}
        # Why the blend did not advance on the last call. "moving" means it did.
        self.limit = "idle"

    def begin(
        self,
        side: str,
        left_ankle: np.ndarray,
        right_ankle: np.ndarray,
        com: np.ndarray,
        sole_corners: np.ndarray,
        home: dict[str, float],
    ) -> None:
        """Solve the planted-foot posture for ``side`` and start the blend.

        ``left_ankle`` and ``right_ankle`` are the measured world positions.
        ``sole_corners`` is the measured stance sole, shape ``(n, 2)``. The
        target is the centre of that sole, not a fixed offset from the midline.
        """
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        corners = np.asarray(sole_corners, dtype=float).reshape(-1, 2)
        self.side = side
        self.fraction = 0.0
        self.origin_xy = np.asarray(com[:2], dtype=float).copy()
        self.target_xy = corners.mean(axis=0)
        self.sole_low = corners.min(axis=0)
        self.sole_high = corners.max(axis=0)
        self.pose, self.solved, self.solve_info = _shift_posture(
            self.model,
            np.asarray(left_ankle, dtype=float).reshape(3),
            np.asarray(right_ankle, dtype=float).reshape(3),
            self.target_xy,
            home,
        )

    def advance(self, dt: float, com: np.ndarray, com_velocity: np.ndarray) -> np.ndarray:
        """Step the blend and return the centre-of-mass xy reference.

        The reference is the straight line from the double-support centre of
        mass to the stance-sole centre. The fraction along that line grows
        only while the capture point is still inside the sole. Past the outer
        edge the fraction decreases, which brings the centre of pressure back
        before the body can fall off the capsule.
        """
        if self.side is None:
            return np.asarray(com[:2], dtype=float).copy()
        com = np.asarray(com, dtype=float).reshape(-1)
        vel = np.asarray(com_velocity, dtype=float).reshape(-1)
        omega = float(np.sqrt(9.81 / max(float(com[2]), 0.2)))
        capture_y = float(com[1] + vel[1] / omega)
        if self.side == "left":
            outside = capture_y > float(self.sole_high[1]) - 0.015
            short = float(com[1]) < float(self.target_xy[1]) - 0.005
        else:
            outside = capture_y < float(self.sole_low[1]) + 0.015
            short = float(com[1]) > float(self.target_xy[1]) + 0.005
        if outside:
            self.limit = "capture_outside_sole"
            self.fraction = max(0.0, self.fraction - 2.0 * float(dt))
        elif float(com[2]) < 0.86:
            self.limit = "height"
            self.fraction = max(0.0, self.fraction - 2.0 * float(dt))
        elif not self.solved:
            self.limit = "posture_unsolved"
        elif not short:
            self.limit = "com_at_target"
        elif abs(float(vel[1])) >= 0.04:
            self.limit = "lateral_speed"
        elif self.fraction >= 1.0:
            self.limit = "fraction"
        else:
            self.limit = "moving"
            self.fraction = min(1.0, self.fraction + 0.06 * float(dt))
        return (1.0 - self.fraction) * self.origin_xy + self.fraction * self.target_xy

    def apply(
        self,
        q_mpc: np.ndarray,
        joint_names: list[str],
        home: np.ndarray,
        low: np.ndarray,
        high: np.ndarray,
    ) -> np.ndarray:
        """Leg targets for the current blend, with the MPC's sagittal residual kept.

        Hip roll follows the solved posture alone: the MPC's lateral capture
        offset is the ±0.15 rad clip that stalled the old shift, and adding it
        again would fight the posture. Hip pitch and knee keep a small piece
        of the MPC command so the ankles can still balance fore-aft. Ankle
        pitch is then set so the sole stays level, plus that same fore-aft
        residual.
        """
        if not self.solved or self.side is None or self.fraction <= 0.0:
            return np.asarray(q_mpc, dtype=float).copy()
        q_des = np.asarray(q_mpc, dtype=float).copy()
        home = np.asarray(home, dtype=float)
        index = {name: i for i, name in enumerate(joint_names)}
        fraction = float(self.fraction)
        for name, goal in self.pose.items():
            i = index.get(name)
            if i is None:
                continue
            blended = (1.0 - fraction) * float(home[i]) + fraction * float(goal)
            if "hip_roll" in name:
                q_des[i] = blended
            else:
                residual = float(np.clip(q_des[i] - home[i], -0.10, 0.10))
                q_des[i] = blended + residual
        for side in ("left", "right"):
            hip = index.get(f"{side}_hip_pitch")
            knee = index.get(f"{side}_knee")
            ankle = index.get(f"{side}_ankle")
            if hip is None or knee is None or ankle is None:
                continue
            residual = float(np.clip(q_mpc[ankle] - home[ankle], -0.15, 0.15))
            q_des[ankle] = -(q_des[hip] + q_des[knee]) + residual
        return np.clip(q_des, low, high)


def support_distance(com_xy: np.ndarray, sole_corners: np.ndarray) -> float:
    """Distance from ``com_xy`` to the sole rectangle. Zero when it is inside."""
    corners = np.asarray(sole_corners, dtype=float).reshape(-1, 2)
    point = np.asarray(com_xy, dtype=float).reshape(2)
    low = corners.min(axis=0)
    high = corners.max(axis=0)
    outside = np.maximum(low - point, point - high)
    outside = np.maximum(outside, 0.0)
    return float(np.linalg.norm(outside))


def _shift_posture(
    model,
    left_ankle: np.ndarray,
    right_ankle: np.ndarray,
    target_xy: np.ndarray,
    home: dict[str, float],
) -> tuple[dict[str, float], bool, dict[str, float]]:
    """Joint angles that put the centre of mass on ``target_xy`` with both feet planted.

    The floating base is free in the solve so the pelvis can move over the
    stance foot. The ankles are constrained to the measured positions, so the
    solution is a weight shift, not a step. Returns ``(pose, accepted)``.
    A solve that cannot keep the feet where they were is rejected and the
    caller leaves the joints to the MPC.
    """
    import mujoco

    data = mujoco.MjData(model)
    key = 0 if int(model.nkey) > 0 else -1
    if key == 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    def qadr(name: str) -> int:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"model has no joint {name}")
        return int(model.jnt_qposadr[joint_id])

    def body_id(name: str) -> int:
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))

    root = body_id("pelvis")
    addresses = [qadr(name) for name in _LEG_JOINTS]
    home_q = np.array([float(home.get(name, data.qpos[adr])) for name, adr in zip(_LEG_JOINTS, addresses)])

    def quat_from_rpy(rpy: np.ndarray) -> np.ndarray:
        roll, pitch, yaw = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
        cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
        cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
        cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
        return np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ]
        )

    def fun(x: np.ndarray) -> np.ndarray:
        data.qpos[:3] = x[:3]
        data.qpos[3:7] = quat_from_rpy(x[3:6])
        for address, value in zip(addresses, x[6:]):
            data.qpos[address] = value
        mujoco.mj_forward(model, data)
        left = data.xpos[body_id("left_ankle_link")]
        right = data.xpos[body_id("right_ankle_link")]
        com = data.subtree_com[root]
        return np.concatenate(
            (
                (left - left_ankle) * 10.0,
                (right - right_ankle) * 10.0,
                (com[:2] - target_xy) * 8.0,
                x[3:6] * 0.3,
            )
        )

    guess = np.concatenate((np.array([0.0, 0.0, 0.98, 0.0, 0.0, 0.0]), home_q))
    lower = np.array([-0.4, -0.4, 0.75, -0.35, -0.35, -0.35, -0.43, -1.2, -0.2, -0.43, -1.2, -0.2])
    upper = np.array([0.4, 0.4, 1.15, 0.35, 0.35, 0.35, 0.43, 0.4, 2.05, 0.43, 0.4, 2.05])
    result = least_squares(fun, guess, bounds=(lower, upper), max_nfev=60)
    pose = {name: float(value) for name, value in zip(_LEG_JOINTS, result.x[6:])}
    # Re-evaluate the foot error. A pose that moves the ankles is a step, not a shift.
    fun(result.x)
    left_error = float(np.linalg.norm(data.xpos[body_id("left_ankle_link")] - left_ankle))
    right_error = float(np.linalg.norm(data.xpos[body_id("right_ankle_link")] - right_ankle))
    accepted = left_error < 0.02 and right_error < 0.02 and float(result.cost) < 0.05
    pose_error = {"left_ankle_m": left_error, "right_ankle_m": right_error, "cost": float(result.cost)}
    return pose, accepted, pose_error
