"""Slow walking cycle for the H1.

The robot has no ankle-roll joint. Lateral balance is a hip-roll pose that
puts the centre of mass on one sole, a short swing, and a paused load
transfer onto the foot that just landed. Joint targets are ramped. The
capture-fall hip term and the recovery reference are not used.

Touchdown is not "the foot touched". The new foot has to carry load, the
capture point has to sit inside the soles that are actually down, and the
pelvis has to be slow, for a dwell, before the next swing is allowed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from controllers.lateral_shift import _shift_posture

SOLE_X = (-0.035, 0.140)
SOLE_Y = 0.040
LEG_JOINTS = (
    "hip_roll",
    "hip_pitch",
    "knee",
    "ankle",
)
_NEW_FORCE_N = 60.0
_KEEP_FORCE_N = 25.0
_DWELL_S = 0.40
_RELEASE_S = 0.10


def slew_xy(current: np.ndarray, goal: np.ndarray, speed: float, dt: float) -> np.ndarray:
    """Move ``current`` toward ``goal`` by at most ``speed * dt`` metres."""
    now = np.asarray(current, dtype=float).reshape(2)
    target = np.asarray(goal, dtype=float).reshape(2)
    delta = target - now
    distance = float(np.linalg.norm(delta))
    step = max(0.0, float(speed)) * max(0.0, float(dt))
    if distance <= step or distance <= 1e-9:
        return target.copy()
    return now + delta / distance * step


def ramp_scalar(current: float, goal: float, rate: float, dt: float) -> float:
    step = abs(float(rate)) * max(0.0, float(dt))
    return float(current + np.clip(float(goal) - float(current), -step, step))


def clip_landing(
    start: np.ndarray,
    step_length: float,
    capture_y: float,
    *,
    max_step: float = 0.10,
    max_lateral: float = 0.05,
) -> np.ndarray:
    """Foot target the swing can still reach.

    Forward motion is capped. Lateral motion only trims toward the capture
    point and cannot ask for a step the hip cannot make in the remaining swing.
    """
    origin = np.asarray(start, dtype=float).reshape(3).copy()
    forward = float(np.clip(step_length, 0.0, max_step))
    lateral = float(np.clip(float(capture_y) - origin[1], -max_lateral, max_lateral))
    origin[0] = origin[0] + forward
    origin[1] = origin[1] + lateral
    return origin


def sole_local(point_xy: np.ndarray, ankle_xyz: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    offset = np.array(
        [float(point_xy[0]) - float(ankle_xyz[0]), float(point_xy[1]) - float(ankle_xyz[1]), 0.0]
    )
    return np.asarray(rotation, dtype=float).reshape(3, 3).T @ offset


def point_on_sole(point_xy: np.ndarray, ankle_xyz: np.ndarray, rotation: np.ndarray) -> bool:
    local = sole_local(point_xy, ankle_xyz, rotation)
    return bool(SOLE_X[0] <= local[0] <= SOLE_X[1] and abs(local[1]) <= SOLE_Y)


def landing_support_ok(
    *,
    new_contact: bool,
    old_contact: bool,
    new_force: float,
    old_force: float,
    capture_inside: bool,
    vx: float,
    vy: float,
    roll: float,
    pitch: float,
    roll_rate: float,
    pitch_rate: float,
) -> bool:
    """True when the new foot is actually holding the robot.

    Contact by itself is not enough. The new foot has to share the load,
    the capture point has to be over a sole, and the pelvis has to be quiet.
    """
    sharing = float(new_force) >= 40.0
    return bool(
        new_contact
        and old_contact
        and sharing
        and capture_inside
        and abs(float(vx)) < 0.08
        and abs(float(vy)) < 0.06
        and abs(float(roll)) < 0.30
        and abs(float(pitch)) < 0.30
        and abs(float(roll_rate)) < 0.80
        and abs(float(pitch_rate)) < 0.80
    )


@dataclass
class SupportLatch:
    """Both-foot support, with hysteresis so one light sample cannot drop it."""

    both: bool = False
    weak_time: float = 0.0

    def update(self, new_force: float, both_contact: bool, dt: float) -> str:
        if not self.both:
            if both_contact and float(new_force) >= _NEW_FORCE_N:
                self.both = True
                self.weak_time = 0.0
            return "both" if self.both else "stance"
        if both_contact and float(new_force) >= _KEEP_FORCE_N:
            self.weak_time = 0.0
        else:
            self.weak_time += max(0.0, float(dt))
            if self.weak_time >= _RELEASE_S:
                self.both = False
                self.weak_time = 0.0
        return "both" if self.both else "stance"


def _rpy(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    roll = float(np.arctan2(matrix[2, 1], matrix[2, 2]))
    pitch = float(np.arctan2(-matrix[2, 0], np.hypot(matrix[2, 1], matrix[2, 2])))
    yaw = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
    return np.array([roll, pitch, yaw], dtype=float)


def _swing_xyz(start: np.ndarray, landing: np.ndarray, height: float, progress: float) -> np.ndarray:
    blend = float(np.clip(progress, 0.0, 1.0))
    smooth = blend * blend * (3.0 - 2.0 * blend)
    point = (1.0 - smooth) * np.asarray(start, dtype=float) + smooth * np.asarray(landing, dtype=float)
    point[2] = point[2] + float(height) * np.sin(np.pi * blend)
    return point


class SteadyWalk:
    """One slow step cycle. ``step`` returns the joint goal for this tick."""

    def __init__(
        self,
        joint_names: list[str],
        home_q: np.ndarray,
        joint_low: np.ndarray,
        joint_high: np.ndarray,
        *,
        n_steps: int = 1,
        first_swing: str = "right",
    ) -> None:
        self.names = list(joint_names)
        self.index = {name: i for i, name in enumerate(self.names)}
        self.home = np.asarray(home_q, dtype=float).copy()
        self.low = np.asarray(joint_low, dtype=float).copy()
        self.high = np.asarray(joint_high, dtype=float).copy()
        self.n_steps = int(n_steps)
        self.swing_side = first_swing if first_swing in ("left", "right") else "right"
        self.phase = "stand"
        self.phase_time = 0.0
        self.dwell = 0.0
        self.steps_done = 0
        self.q = self.home.copy()
        self.pose: dict[str, float] = {}
        self.origin: dict[str, float] = {}
        self.fraction = 0.0
        self.hip_hold = 0.0
        self.home_feet: dict[str, np.ndarray] = {}
        self.swing_start = np.zeros(3)
        self.landing = np.zeros(3)
        self.waypoint = np.zeros(2)
        self._solve_wait = 0.0
        self.support = SupportLatch()
        self.touchdown_time = float("nan")
        self.stable_time = float("nan")
        self.sagittal_ref: float | None = None
        self.ankle_bias = 0.0
        self.ankle_nominal = {
            "left": float(self.home[self.index["left_ankle"]]) if "left_ankle" in self.index else 0.0,
            "right": float(self.home[self.index["right_ankle"]]) if "right_ankle" in self.index else 0.0,
        }
        self.log: dict[str, float | str] = {}

    @property
    def stance_side(self) -> str:
        return "left" if self.swing_side == "right" else "right"

    @property
    def walk_state(self) -> str:
        if self.phase in ("stand", "finished"):
            return "STAND"
        if self.phase == "shift":
            return "SHIFT_TO_STANCE"
        if self.phase == "swing":
            return "RIGHT_SWING" if self.swing_side == "right" else "LEFT_SWING"
        if self.phase == "touchdown":
            return "TOUCHDOWN"
        if self.phase == "transfer":
            return "LOAD_TRANSFER"
        return "STABLE_DOUBLE_SUPPORT"

    def _leg(self, side: str, joint: str) -> str:
        return f"{side}_{joint}"

    def _set_leg(self, goal: dict[str, float], side: str, joint: str, value: float) -> None:
        goal[self._leg(side, joint)] = float(value)

    def _home_dict(self) -> dict[str, float]:
        return {name: float(self.home[self.index[name]]) for name in self.names if name in self.index}

    def _ramp_to(self, goal: dict[str, float], rate: float, dt: float) -> None:
        for name, value in goal.items():
            if name.endswith("_ankle"):
                side = name.split("_", 1)[0]
                self.ankle_nominal[side] = ramp_scalar(self.ankle_nominal.get(side, 0.0), float(value), rate, dt)
                continue
            slot = self.index.get(name)
            if slot is None:
                continue
            commanded = ramp_scalar(float(self.q[slot]), float(value), rate, dt)
            self.q[slot] = float(np.clip(commanded, self.low[slot], self.high[slot]))

    def _servo_stance_hip(self, capture: np.ndarray, ankle: np.ndarray, dt: float) -> None:
        """Pull the capture point back onto the stance sole with hip roll.

        The offset is clipped to ±0.15 rad around the pose that was holding
        the body at liftoff, and it is ramped. There is no ankle-roll joint.
        """
        name = f"{self.stance_side}_hip_roll"
        slot = self.index.get(name)
        if slot is None:
            return
        offset = float(np.clip(12.0 * (float(capture[1]) - float(ankle[1])), -0.15, 0.15))
        goal = self.hip_hold + offset
        commanded = ramp_scalar(float(self.q[slot]), goal, 1.5, dt)
        self.q[slot] = float(np.clip(commanded, self.low[slot], self.high[slot]))

    def _commit_ankles(self, com: np.ndarray, vel: np.ndarray, dt: float) -> None:
        """Pitch both ankles so the centre of pressure follows the capture point.

        The H1 has no ankle roll. This is the sagittal correction the standing
        pose already uses. It is an offset on the leveled ankle, not a new pose.
        """
        if self.sagittal_ref is None:
            self.sagittal_ref = float(com[0])
        omega = float(np.sqrt(9.81 / max(float(com[2]), 0.2)))
        error_x = (float(com[0]) - float(self.sagittal_ref)) + float(vel[0]) / omega
        # Negative ankle pitch moves this robot's centre of mass forward.
        target = float(np.clip(12.0 * error_x, -0.15, 0.15))
        self.ankle_bias = ramp_scalar(self.ankle_bias, target, 3.0, dt)
        for side in ("left", "right"):
            slot = self.index.get(f"{side}_ankle")
            if slot is None:
                continue
            commanded = self.ankle_nominal.get(side, float(self.home[slot])) + self.ankle_bias
            self.q[slot] = float(np.clip(commanded, self.low[slot], self.high[slot]))

    def _hold_pose_dict(self) -> dict[str, float]:
        return {name: float(self.q[self.index[name]]) for name in self.names}

    def _flat_ankles(self, pose: dict[str, float], sides: tuple[str, ...]) -> dict[str, float]:
        """Keep a planted sole level. The H1 cannot roll the ankle."""
        goal = dict(pose)
        for side in sides:
            hip = goal.get(f"{side}_hip_pitch")
            knee = goal.get(f"{side}_knee")
            if hip is None or knee is None:
                continue
            goal[f"{side}_ankle"] = float(-(float(hip) + float(knee)))
        return goal

    def step_inputs(
        self,
        *,
        dt: float,
        com: np.ndarray,
        vel: np.ndarray,
        capture: np.ndarray,
        roll: float,
        pitch: float,
        roll_rate: float,
        pitch_rate: float,
        left_ankle: np.ndarray,
        right_ankle: np.ndarray,
        left_rotation: np.ndarray,
        right_rotation: np.ndarray,
        left_force: float,
        right_force: float,
        left_contact: bool,
        right_contact: bool,
        solved_pose: dict[str, float] | None,
    ) -> None:
        """Advance the cycle from one measured sample. No physics here."""
        self.phase_time += float(dt)
        if self.sagittal_ref is None:
            self.sagittal_ref = 0.5 * (float(left_ankle[0]) + float(right_ankle[0])) + 0.045
        if not self.home_feet:
            self.home_feet = {
                "left": np.asarray(left_ankle, dtype=float).copy(),
                "right": np.asarray(right_ankle, dtype=float).copy(),
            }
        stance = self.stance_side
        swing = self.swing_side
        ankles = {"left": left_ankle, "right": right_ankle}
        rotations = {"left": left_rotation, "right": right_rotation}
        forces = {"left": left_force, "right": right_force}
        contacts = {"left": left_contact, "right": right_contact}
        on_stance = point_on_sole(com[:2], ankles[stance], rotations[stance])
        on_swing = point_on_sole(com[:2], ankles[swing], rotations[swing])
        capture_on_stance = point_on_sole(capture[:2], ankles[stance], rotations[stance])
        capture_on_either = capture_on_stance or point_on_sole(capture[:2], ankles[swing], rotations[swing])
        slow = abs(float(vel[0])) < 0.06 and abs(float(vel[1])) < 0.05
        upright = abs(float(roll)) < 0.30 and abs(float(pitch)) < 0.30
        both_contact = bool(left_contact and right_contact)

        if self.phase == "stand":
            if self.steps_done == 0:
                self._ramp_to(self._home_dict(), 0.4, dt)
            if self.phase_time >= 1.0 and self.steps_done < self.n_steps:
                self._enter("shift")
        elif self.phase == "finished":
            self._commit_ankles(com, vel, dt)
            return
        elif self.phase == "shift" and self.steps_done > 0:
            target_y = float(ankles[stance][1])
            want = 1.0 if target_y > float(com[1]) else -1.0
            # The same sign on both hips moves the body sideways. Continue
            # from the pose that finished the previous transfer.
            delta = 0.55 if want < 0.0 else -0.55
            wrong_way = float(vel[1]) * want < 0.0 and abs(float(vel[1])) > 0.06
            for side in ("left", "right"):
                name = f"{side}_hip_roll"
                slot = self.index.get(name)
                if slot is None:
                    continue
                base = float(self.origin.get(name, self.q[slot]))
                goal = base if abs(float(pitch)) > 0.16 or wrong_way else base + delta
                commanded = ramp_scalar(float(self.q[slot]), goal, 0.12, dt)
                self.q[slot] = float(np.clip(commanded, self.low[slot], self.high[slot]))
            ready = (
                abs(float(com[1]) - target_y) < 0.045
                and slow
                and abs(float(pitch)) < 0.16
                and forces[stance] > 40.0
            )
            self.dwell = self.dwell + dt if ready else 0.0
            if self.dwell >= 0.40:
                self.swing_start = np.asarray(ankles[swing], dtype=float).copy()
                planted = self.home_feet.get(swing, self.swing_start)
                self.landing = np.asarray(planted, dtype=float).copy()
                self.landing[0] = float(self.swing_start[0]) + 0.08
                self.landing[2] = float(planted[2])
                hold_name = f"{stance}_hip_roll"
                if hold_name in self.index:
                    self.hip_hold = float(self.q[self.index[hold_name]])
                self._enter("swing")
        elif self.phase == "shift":
            if solved_pose:
                self.pose = dict(solved_pose)
            if self.pose:
                direction = 1.0 if stance == "left" else -1.0
                toward = float(vel[1]) * direction
                gap = max(0.0, abs(float(ankles[stance][1]) - float(com[1])))
                approach = min(0.06, 0.6 * gap)
                braking = toward > approach + 0.015 or (capture_on_stance and toward > 0.03)
                if braking:
                    self.fraction = max(0.0, self.fraction - 0.20 * dt)
                else:
                    self.fraction = min(1.0, self.fraction + 0.15 * dt)
                blended: dict[str, float] = {}
                for name, value in self.pose.items():
                    base = float(self.origin.get(name, self.home[self.index[name]] if name in self.index else value))
                    blended[name] = (1.0 - self.fraction) * base + self.fraction * float(value)
                self._ramp_to(self._flat_ankles(blended, ("left", "right")), 1.5, dt)
            ready = on_stance and capture_on_stance and slow and upright and forces[stance] > 40.0
            self.dwell = self.dwell + dt if ready else 0.0
            if self.dwell >= 0.50:
                self.swing_start = np.asarray(ankles[swing], dtype=float).copy()
                planted = self.home_feet.get(swing, self.swing_start)
                self.landing = np.asarray(planted, dtype=float).copy()
                self.landing[0] = float(self.swing_start[0]) + 0.08
                self.landing[2] = float(planted[2])
                hold_name = f"{stance}_hip_roll"
                self.hip_hold = float(self.q[self.index[hold_name]]) if hold_name in self.index else 0.0
                self._enter("swing")
        elif self.phase == "swing":
            progress = self.phase_time / 0.50
            target = _swing_xyz(self.swing_start, self.landing, 0.05, min(progress, 1.0))
            self.log["swing_target_x"] = float(target[0])
            self.log["swing_target_y"] = float(target[1])
            self.log["swing_target_z"] = float(target[2])
            if solved_pose:
                self._ramp_to(solved_pose, 1.2, dt)
            self._servo_stance_hip(capture, ankles[stance], dt)
            landed = progress >= 0.90 and contacts[swing] and forces[swing] >= _NEW_FORCE_N
            if landed:
                self.touchdown_time = self.phase_time
                self.waypoint = np.asarray(com[:2], dtype=float).copy()
                self._enter("touchdown")
        elif self.phase == "touchdown":
            # Keep the joints that were holding the robot up. Moving the
            # swing leg again here is what throws the body off the sole.
            self._servo_stance_hip(capture, ankles[stance], dt)
            self.support.update(forces[swing], both_contact, dt)
            quiet = (
                abs(float(vel[0])) < 0.04
                and abs(float(vel[1])) < 0.03
                and abs(float(pitch)) < 0.18
                and abs(float(roll)) < 0.25
                and contacts[swing]
                and contacts[stance]
                and forces[swing] >= 5.0
                and self.phase_time > 0.40
            )
            self.dwell = self.dwell + dt if quiet else 0.0
            if self.dwell >= 0.30:
                self.waypoint = np.asarray(com[:2], dtype=float).copy()
                self.sagittal_ref = 0.5 * (float(left_ankle[0]) + float(right_ankle[0])) + 0.045
                hold_name = f"{stance}_hip_roll"
                if hold_name in self.index:
                    self.hip_hold = float(self.q[self.index[hold_name]])
                self._enter("transfer")
        elif self.phase == "transfer":
            # Move only the stance hip, away from the pose that is already
            # balancing. A freshly solved crouch here throws the body.
            new_y = float(ankles[swing][1])
            want = -1.0 if new_y < float(com[1]) else 1.0
            sign = 1.0 if stance == "left" else -1.0
            goal = self.hip_hold + sign * want * -1.0 * 0.20
            wrong_way = float(vel[1]) * want < 0.0 and abs(float(vel[1])) > 0.05
            if abs(float(pitch)) > 0.18 or wrong_way:
                goal = self.hip_hold
            name = f"{stance}_hip_roll"
            slot = self.index.get(name)
            if slot is not None:
                commanded = ramp_scalar(float(self.q[slot]), goal, 0.15, dt)
                self.q[slot] = float(np.clip(commanded, self.low[slot], self.high[slot]))
            between = min(float(left_ankle[1]), float(right_ankle[1])) <= float(com[1]) <= max(
                float(left_ankle[1]), float(right_ankle[1])
            )
            arrived = (
                between
                and slow
                and upright
                and forces[swing] > 100.0
                and forces[stance] > 100.0
            )
            self.dwell = self.dwell + dt if arrived else 0.0
            if self.dwell >= 0.40:
                self.steps_done += 1
                self.stable_time = self.phase_time
                self._enter("stable")
        elif self.phase == "stable":
            if solved_pose:
                self._ramp_to(self._flat_ankles(solved_pose, ("left", "right")), 0.3, dt)
            self.dwell = self.dwell + dt if (slow and upright and both_contact) else 0.0
            if self.dwell >= 0.30:
                if self.steps_done >= self.n_steps:
                    self._enter("stand")
                else:
                    self.swing_side = "left" if self.swing_side == "right" else "right"
                    hold_name = f"{self.stance_side}_hip_roll"
                    if hold_name in self.index:
                        self.hip_hold = float(self.q[self.index[hold_name]])
                    self._enter("shift")
        self._commit_ankles(com, vel, dt)
        self.log["phase"] = self.phase
        self.log["dwell"] = float(self.dwell)
        self.log["ankle_bias"] = float(self.ankle_bias)
        self.log["on_stance"] = float(on_stance)
        self.log["capture_on_support"] = float(capture_on_either if self.phase in ("touchdown", "transfer", "stable") else capture_on_stance)

    def _enter(self, phase: str) -> None:
        if phase in ("shift", "transfer"):
            self.origin = {name: float(self.q[self.index[name]]) for name in self.names if name in self.index}
            for side in ("left", "right"):
                self.origin[f"{side}_ankle"] = float(self.ankle_nominal.get(side, 0.0))
            self.fraction = 0.0
            self.pose = {}
        self.phase = phase
        self.phase_time = 0.0
        self.dwell = 0.0
        self.support = SupportLatch()
        self._solve_wait = 0.0


def _ankle(env, side: str) -> tuple[np.ndarray, np.ndarray]:
    import mujoco

    body = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_link")
    position = np.asarray(env.data.xpos[body], dtype=float).copy()
    rotation = np.asarray(env.data.xmat[body], dtype=float).reshape(3, 3).copy()
    return position, rotation


def _foot_force(env, side: str) -> tuple[bool, float]:
    import mujoco

    body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_ankle_link")
    total = 0.0
    wrench = np.zeros(6)
    hit = False
    for index in range(int(env.data.ncon)):
        contact = env.data.contact[index]
        if float(contact.dist) > 1e-3:
            continue
        geoms = (int(contact.geom1), int(contact.geom2))
        bodies = [int(env.model.geom_bodyid[geom]) for geom in geoms]
        if body_id not in bodies:
            continue
        other = bodies[0] if bodies[1] == body_id else bodies[1]
        if other == body_id:
            continue
        hit = True
        mujoco.mj_contactForce(env.model, env.data, index, wrench)
        total += max(0.0, float(wrench[0]))
    return hit, float(total)


def _sole_polygon(position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    corners = np.array(
        [
            [SOLE_X[0], -SOLE_Y, 0.0],
            [SOLE_X[1], -SOLE_Y, 0.0],
            [SOLE_X[1], SOLE_Y, 0.0],
            [SOLE_X[0], SOLE_Y, 0.0],
        ]
    )
    world = np.asarray(position, dtype=float) + corners @ np.asarray(rotation, dtype=float).reshape(3, 3).T
    return world[:, :2]


def _capture(com: np.ndarray, vel: np.ndarray) -> np.ndarray:
    omega = float(np.sqrt(9.81 / max(float(com[2]), 0.2)))
    return np.asarray(com[:2], dtype=float) + np.asarray(vel[:2], dtype=float) / omega


def _solve_shift(env, walker: SteadyWalk, target_xy: np.ndarray, left: np.ndarray, right: np.ndarray) -> dict[str, float] | None:
    home = walker._home_dict()
    pose, _rpy_pose, accepted, _info = _shift_posture(env.model, left, right, target_xy, home)
    if not accepted:
        return None
    return pose


def _swing_goal(walker: SteadyWalk, ik, env, target: np.ndarray) -> dict[str, float]:
    """Keep the stance pose that is already holding the body. Only the swing leg tracks."""
    goal: dict[str, float] = {}
    stance = walker.stance_side
    for joint in ("hip_roll", "hip_pitch", "knee"):
        name = f"{stance}_{joint}"
        slot = walker.index.get(name)
        if slot is not None:
            goal[name] = float(walker.q[slot])
    goal[f"{stance}_ankle"] = float(walker.ankle_nominal.get(stance, 0.0))
    solved = ik.solve(target, env, side=walker.swing_side)
    walker._set_leg(goal, walker.swing_side, "hip_pitch", solved.hip_pitch)
    walker._set_leg(goal, walker.swing_side, "knee", solved.knee)
    walker._set_leg(goal, walker.swing_side, "ankle", solved.ankle_pitch)
    # Hip roll stays at the angle that was holding the foot. A lateral
    # correction here rolls the pelvis off the stance sole.
    name = f"{walker.swing_side}_hip_roll"
    slot = walker.index.get(name)
    if slot is not None:
        goal[name] = float(walker.q[slot])
    return walker._flat_ankles(goal, (stance,))


def steady_tick(env, pd, cbf, walker: SteadyWalk, ik) -> dict:
    """One control period: walker target, PD, CBF, then MuJoCo."""
    import mujoco

    state = env.get_state()
    com = np.asarray(state["com"], dtype=float).reshape(3)
    vel = np.asarray(state["com_velocity"], dtype=float).reshape(3)
    capture = _capture(com, vel)
    left, left_rot = _ankle(env, "left")
    right, right_rot = _ankle(env, "right")
    pelvis_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    pelvis_rot = np.asarray(env.data.xmat[pelvis_id], dtype=float).reshape(3, 3)
    roll, pitch, _yaw = _rpy(pelvis_rot)
    ang = np.asarray(env.data.qvel[3:6], dtype=float)
    left_contact, left_force = _foot_force(env, "left")
    right_contact, right_force = _foot_force(env, "right")
    dt = float(env.config.control_dt) if hasattr(env, "config") else 0.02

    solved: dict[str, float] | None = None
    walker._solve_wait -= dt
    if walker.phase == "shift" and walker.steps_done == 0:
        sole = _sole_polygon(
            left if walker.stance_side == "left" else right,
            left_rot if walker.stance_side == "left" else right_rot,
        )
        # Keep the current forward position. The shift only has to move
        # sideways onto the sole; chasing the sole centre in x crouches the legs.
        target_x = float(walker.sagittal_ref if walker.sagittal_ref is not None else com[0])
        target = np.array(
            [
                float(np.clip(target_x, sole[:, 0].min() + 0.02, sole[:, 0].max() - 0.02)),
                float(sole[:, 1].mean()),
            ]
        )
        if not walker.pose:
            solved = _solve_shift(env, walker, target, left, right)
        else:
            solved = dict(walker.pose)
    elif walker.phase == "swing":
        progress = walker.phase_time / 0.50
        target = _swing_xyz(walker.swing_start, walker.landing, 0.05, min(max(progress, 0.0), 1.0))
        solved = _swing_goal(walker, ik, env, target)
    elif walker.phase == "touchdown":
        target = np.asarray(walker.landing, dtype=float).copy()
        target[2] = float(walker.swing_start[2])
        solved = _swing_goal(walker, ik, env, target)
    elif walker.phase == "stable":
        solved = None
    walker.step_inputs(
        dt=dt,
        com=com,
        vel=vel,
        capture=capture,
        roll=float(roll),
        pitch=float(pitch),
        roll_rate=float(ang[0]),
        pitch_rate=float(ang[1]),
        left_ankle=left,
        right_ankle=right,
        left_rotation=left_rot,
        right_rotation=right_rot,
        left_force=left_force,
        right_force=right_force,
        left_contact=left_contact,
        right_contact=right_contact,
        solved_pose=solved,
    )

    stance = walker.stance_side
    swing = walker.swing_side
    on_stance_now = point_on_sole(
        com[:2],
        left if stance == "left" else right,
        left_rot if stance == "left" else right_rot,
    )
    use_both = walker.phase in ("transfer", "stable", "stand", "finished") or (
        walker.phase == "shift" and not on_stance_now
    )
    if walker.phase == "swing":
        use_both = False
    if walker.phase == "touchdown":
        use_both = bool(walker.support.both)
    polygons = []
    if use_both or walker.support.both:
        polygons.append(_sole_polygon(left, left_rot))
        polygons.append(_sole_polygon(right, right_rot))
    elif stance == "left":
        polygons.append(_sole_polygon(left, left_rot))
    else:
        polygons.append(_sole_polygon(right, right_rot))
    cbf.config.margin = 0.0
    cbf.set_support_polygon(np.vstack(polygons))

    q_des = np.clip(walker.q, walker.low, walker.high)
    q_meas = np.asarray(state["joint_qpos"], dtype=float)
    qd_meas = np.asarray(state["joint_qvel"], dtype=float)
    gravity = np.asarray(env.data.qfrc_bias[env._actuator_dof_adr], dtype=float)
    tau_nominal = pd.compute_torque(q_des, q_meas, qd_meas, torque_feedforward=gravity)
    tau_safe = cbf.safe_torque(tau_nominal, com, vel)
    delta = np.asarray(tau_safe, dtype=float) - np.asarray(tau_nominal, dtype=float)

    def torque_at_substep() -> np.ndarray:
        q = env.data.qpos[env._actuator_qpos_adr]
        qd = env.data.qvel[env._actuator_dof_adr]
        hold = np.asarray(env.data.qfrc_bias[env._actuator_dof_adr], dtype=float)
        return pd.compute_torque(q_des, q, qd, torque_feedforward=hold) + delta

    env.step_feedback(torque_at_substep)
    after = env.get_state()
    after_com = np.asarray(after["com"], dtype=float).reshape(3)
    after_vel = np.asarray(after["com_velocity"], dtype=float).reshape(3)
    ref = np.array([walker.waypoint[0], walker.waypoint[1], after_com[2]]) if walker.phase == "transfer" else after_com.copy()
    names = walker.names
    row = {
        "time": float(np.asarray(after["time"]).reshape(-1)[0]),
        "com": after_com.copy(),
        "com_reference": ref,
        "com_error": after_com - ref,
        "com_velocity": after_vel.copy(),
        "com_acceleration": np.zeros(3),
        "intervention": float(np.linalg.norm(tau_safe - tau_nominal)),
        "cbf_status": cbf.status,
        "tau_nominal": np.asarray(tau_nominal, dtype=float).copy(),
        "tau_safe": np.asarray(tau_safe, dtype=float).copy(),
        "q_des": q_des.copy(),
        "qd_des": np.zeros_like(q_des),
        "qpos": after["qpos"].copy(),
        "qvel": after["qvel"].copy(),
        "joint_qpos": after["joint_qpos"].copy(),
        "joint_qvel": after["joint_qvel"].copy(),
        "walk_state": walker.walk_state,
        "support_foot": "both" if use_both or walker.support.both else stance,
        "swing_side": swing if walker.phase == "swing" else "",
        "stance_foot_xy": np.array([float((left if stance == "left" else right)[0]), float((left if stance == "left" else right)[1])]),
        "support_distance": 0.0,
        "rl_active": 0.0,
        "rl_residual": np.zeros(2),
        "capture_xy": _capture(after_com, after_vel),
        "pelvis_height": float(after_com[2]),
        "pelvis_roll": float(roll),
        "pelvis_pitch": float(pitch),
        "left_foot_normal_force": left_force,
        "right_foot_normal_force": right_force,
        "left_contact": float(left_contact),
        "right_contact": float(right_contact),
        "left_foot_xyz": left.copy(),
        "right_foot_xyz": right.copy(),
        "touchdown_time": float(walker.touchdown_time),
        "stable_support_time": float(walker.stable_time),
        "step_number": float(walker.steps_done),
        "shift_fraction": float(walker.fraction),
        "shift_limit": walker.phase,
        "barriers": {},
        "support_x_lower": float(after_com[0] - min(left[0], right[0]) + 0.035),
        "capture_margin_x_lower": 0.05,
        "stance_ankle_correction": 0.0,
    }
    for side in ("left", "right"):
        for joint in LEG_JOINTS:
            name = f"{side}_{joint}"
            slot = names.index(name) if name in names else None
            if slot is None:
                continue
            row[name] = float(after["joint_qpos"][slot])
            row[f"{name}_des"] = float(q_des[slot])
            row[f"{name}_torque"] = float(tau_safe[slot])
            row[f"{name}_nominal"] = float(tau_nominal[slot])
    row["q_des_final"] = float(row.get(f"{stance}_hip_roll_des", np.nan))
    row["q_measured"] = float(row.get(f"{stance}_hip_roll", np.nan))
    row["cbf_intervention_hip_roll"] = float(
        row.get(f"{stance}_hip_roll_nominal", 0.0) - row.get(f"{stance}_hip_roll_torque", 0.0)
    )
    return row
