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
slows down once the body is already moving onto the sole, and it is held once
the centre of mass is inside and slow. The swing is still someone else's
decision: this class never reports that the foot may lift.
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
        self.pelvis_rpy = np.zeros(3)
        self.solved = False
        self.solve_info: dict[str, float] = {}
        # Why the blend did not advance on the last call. "moving" means it did.
        self.limit = "idle"
        self._on_sole = False
        self._com = np.zeros(3)
        # Leg left on its landed pose. apply() must not pull it back to the
        # pre-swing shift solution.
        self.skip_side = ""
        # Late-swing trim. The walking tick turns this on only for a swing,
        # and advance() uses it only while the capture point is closing on
        # the outer sole edge.
        self.capture_trim = False
        self._trim_anchor: float | None = None
        # Touchdown reference. Held so the 8 mm clamp cannot chase the body
        # off the sole after the swing.
        self.handoff_xy: np.ndarray | None = None
        self.hold_landed = False

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
        self.skip_side = ""
        self._trim_anchor = None
        self.handoff_xy = None
        self.hold_landed = False
        self._on_sole = False
        self.origin_xy = np.asarray(com[:2], dtype=float).copy()
        self.target_xy = corners.mean(axis=0)
        self.sole_low = corners.min(axis=0)
        self.sole_high = corners.max(axis=0)
        self.pose, self.pelvis_rpy, self.solved, self.solve_info = _shift_posture(
            self.model,
            np.asarray(left_ankle, dtype=float).reshape(3),
            np.asarray(right_ankle, dtype=float).reshape(3),
            self.target_xy,
            home,
        )

    def advance(
        self,
        dt: float,
        com: np.ndarray,
        com_velocity: np.ndarray,
        sole_corners: np.ndarray | None = None,
    ) -> np.ndarray:
        """Step the blend and return the centre-of-mass xy reference.

        The blend is a slow lean toward the measured sole. Near the sole the
        inverted pendulum runs away if that lean keeps growing: the centre of
        mass gets ahead of the reference, and the MPC then saturates its
        lateral acceleration at -5 m/s^2. Hip roll is owned by this posture,
        so that saturated command never reaches the joint and the body keeps
        accelerating. The lean therefore stops, and then eases off, once the
        capture point is near the sole or the lateral speed is already a few
        centimetres per second. The reference stays within 8 mm of the body
        so the acceleration command remains inside its box.
        """
        if self.side is None:
            return np.asarray(com[:2], dtype=float).copy()
        com = np.asarray(com, dtype=float).reshape(-1)
        vel = np.asarray(com_velocity, dtype=float).reshape(-1)
        if sole_corners is not None:
            corners = np.asarray(sole_corners, dtype=float).reshape(-1, 2)
            self.sole_low = corners.min(axis=0)
            self.sole_high = corners.max(axis=0)
        omega = float(np.sqrt(9.81 / max(float(com[2]), 0.2)))
        capture_y = float(com[1] + vel[1] / omega)
        sign = 1.0 if self.side == "left" else -1.0
        # Stop just inside the live sole. Chasing the sole centre rolls the
        # capsule, and the contact creeps further out.
        if self.side == "left":
            hold_y = min(float(self.sole_low[1]) + 0.015, float(self.target_xy[1]))
            gap = float(self.sole_low[1] - com[1])
            capture_in = float(self.sole_low[1]) <= capture_y <= float(self.sole_high[1])
            short = capture_y < hold_y - 0.005
        else:
            hold_y = max(float(self.sole_high[1]) - 0.015, float(self.target_xy[1]))
            gap = float(com[1] - self.sole_high[1])
            capture_in = float(self.sole_low[1]) <= capture_y <= float(self.sole_high[1])
            short = capture_y > hold_y + 0.005
        inside = bool(
            self.sole_low[0] <= com[0] <= self.sole_high[0]
            and self.sole_low[1] <= com[1] <= self.sole_high[1]
        )
        toward = float(vel[1]) * sign
        close = gap < 0.05
        braking = (close and toward > 0.02) or capture_in or toward > 0.04
        if inside and abs(toward) < 0.03:
            # Latched only after a slow arrival. A fly-through used to set
            # this while still moving, and holding that lean threw the swing.
            self._on_sole = True
        if not self.solved:
            self.limit = "posture_unsolved"
            self.fraction = 0.0
        elif float(com[2]) < 0.80 and not inside:
            self.limit = "height"
            self.fraction = max(0.0, self.fraction - 2.0 * float(dt))
        elif self._on_sole:
            self.limit = "com_inside"
            # The lean is latched for the swing. It is eased only while the
            # capture point is still inside and closing on the outer edge,
            # and only by a few hundredths of the blend. Unwinding the same
            # hip roll after touchdown did not pull the body back.
            if (
                self.capture_trim
                and not self.skip_side
                and inside
                and toward > 0.0
            ):
                outer = (
                    float(self.sole_high[1]) - capture_y
                    if self.side == "left"
                    else capture_y - float(self.sole_low[1])
                )
                if outer < 0.05:
                    if self._trim_anchor is None:
                        self._trim_anchor = float(self.fraction)
                    floor = max(0.0, float(self._trim_anchor) - 0.04)
                    self.fraction = max(floor, self.fraction - 0.15 * float(dt))
                    self.limit = "late_capture"
        elif braking and toward > 0.015:
            self.limit = "lateral_brake"
            self.fraction = max(0.0, self.fraction - 0.20 * float(dt))
        elif short and self.fraction < 1.0:
            self.limit = "moving"
            self.fraction = min(1.0, self.fraction + 0.05 * float(dt))
        else:
            self.limit = "capture_at_target"
        reference = (1.0 - self.fraction) * self.origin_xy + self.fraction * self.target_xy
        if self.side == "left":
            reference[1] = min(float(reference[1]), hold_y)
        else:
            reference[1] = max(float(reference[1]), hold_y)
        # Keep the lateral error under a centimetre. A larger lag saturates
        # the MPC acceleration, and the posture has already replaced the hip
        # roll that acceleration would have used.
        if not self._on_sole and braking and toward > 0.015:
            reference[1] = float(com[1]) - 0.008 * sign
        reference[1] = float(np.clip(reference[1], float(com[1]) - 0.008, float(com[1]) + 0.008))
        # Sagittal reference stays at the double-support position. Pulling it
        # toward the sole centre walks the body onto the toes.
        reference[0] = float(np.clip(self.origin_xy[0], float(com[0]) - 0.008, float(com[0]) + 0.008))
        self._com = com.copy()
        return reference

    def apply(
        self,
        q_mpc: np.ndarray,
        joint_names: list[str],
        home: np.ndarray,
        low: np.ndarray,
        high: np.ndarray,
    ) -> np.ndarray:
        """Leg targets for the current blend, with the MPC's sagittal residual kept.

        Hip roll follows the solved posture alone. The capture offset on that
        joint is the ±0.15 rad clip that stalled the old shift, and adding it
        also breaks the pelvis-roll pair that keeps the soles flat. Hip pitch
        and knee keep a small piece of the MPC command so the ankles can still
        balance fore-aft. Ankle pitch then levels the sole, plus that residual.
        """
        if not self.solved or self.side is None or self.fraction <= 0.0:
            return np.asarray(q_mpc, dtype=float).copy()
        q_des = np.asarray(q_mpc, dtype=float).copy()
        home = np.asarray(home, dtype=float)
        index = {name: i for i, name in enumerate(joint_names)}
        fraction = float(self.fraction)
        skip = self.skip_side
        for name, goal in self.pose.items():
            i = index.get(name)
            if i is None:
                continue
            blended = (1.0 - fraction) * float(home[i]) + fraction * float(goal)
            if skip and name.startswith(f"{skip}_"):
                # Hip roll stays on the lean. Pitch and knee stay on the
                # touchdown command written into home: q_mpc would add the
                # capture offset on the landing tick, about 0.13 rad on the
                # ankle in the baseline hand-off.
                if "hip_roll" in name:
                    q_des[i] = blended
                elif self.hold_landed:
                    q_des[i] = float(home[i])
                continue
            if "hip_roll" in name:
                q_des[i] = blended
            else:
                residual = float(np.clip(q_des[i] - home[i], -0.10, 0.10))
                q_des[i] = blended + residual
        for side in ("left", "right"):
            if side == skip:
                if self.hold_landed:
                    ankle = index.get(f"{side}_ankle")
                    if ankle is not None:
                        q_des[ankle] = float(home[ankle])
                continue
            hip = index.get(f"{side}_hip_pitch")
            knee = index.get(f"{side}_knee")
            ankle = index.get(f"{side}_ankle")
            if hip is None or knee is None or ankle is None:
                continue
            residual = float(np.clip(q_mpc[ankle] - home[ankle], -0.15, 0.15))
            # The knee blend pitches the pelvis back a few millimetres per
            # second. The capture residual above is already at its global
            # clip once that error exists, and it loses. Extra ankle pitch,
            # proportional to how far the centre of mass has slid behind the
            # double-support position, is what keeps the heel barrier open.
            back = float(self.origin_xy[0] - self._com[0])
            heel = float(np.clip(-4.0 * back, -0.2, 0.05))
            q_des[ankle] = -(q_des[hip] + q_des[knee]) + residual + heel
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
) -> tuple[dict[str, float], np.ndarray, bool, dict[str, float]]:
    """Joint angles that put the centre of mass on ``target_xy`` with both feet planted.

    The floating base is free in the solve so the pelvis can move over the
    stance foot. The ankles are constrained to the measured positions, so the
    solution is a weight shift, not a step. Returns the leg joints and the
    pelvis roll, pitch and yaw of that pose. A solve that cannot keep the
    feet where they were is rejected and the caller leaves the joints to the MPC.
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
    return pose, np.asarray(result.x[3:6], dtype=float).copy(), accepted, pose_error
