"""Place the swing foot on the measured capture point.

The centre of mass is not driven onto the next foot. The capture point

    capture = com + com_velocity / omega
    omega = sqrt(g / com_height)

is a target for the airborne foot. The landing starts at the nominal step
and adds a bounded correction. A target the leg IK rejects is pulled back
toward that nominal step until the IK accepts it. The correction ramps in
during the swing and freezes for the last quarter, so the foot is not
chasing the capture point at contact.

Nothing here writes a torque. The walking tick turns the accepted landing
into a joint target, and the existing PD and CBF still sit in front of MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from controllers.adaptive_foot_placement import (
    SOLE_HALF_WIDTH,
    SOLE_HEEL,
    SOLE_TOE,
    placement_ramp,
)
from controllers.capture_point_controller import GRAVITY, angular_velocity, capture_point
from controllers.mpc_controller import lateral_capture_hip_offset

# Bounds on the landing correction, metres. They are a starting limit, not a
# claim that every capture point inside them is reachable. The IK check is
# what accepts a target.
MAX_LATERAL_CORRECTION = 0.16
MAX_FORWARD_CORRECTION = 0.12

# Early swing keeps the nominal landing so the foot can lift. The correction
# then ramps until the freeze, which is the last quarter of the swing.
RAMP_START = 0.15
FREEZE_PROGRESS = 0.75

# How fast the landing may move while it is still adapting, m/s.
TARGET_SLEW = 0.50

# After this fraction of the swing, a landing the foot cannot reach is
# pulled back and an uncatchable step stops driving the body inward.
TRACK_PROGRESS = 0.45

# Shrink the sole by this much before calling a landing capture-valid.
# The capture point does not have to sit on the sole centre.
SOLE_MARGIN = 0.005

# Swing-hip position gain and the CBF's per-joint torque box. Neither is a
# tuning knob: they are the values already in the PD and the filter. A hip
# command further than ``torque_lead`` from the measured angle asks for more
# than that box will pass, which is what left the foot short in
# capture_step_v1.
SWING_HIP_KP = 400.0
TORQUE_BOX = 40.0

# Stance-hip contribution to the body velocity. This is an offset from the
# weight-shift angle, clipped to the existing 0.15 rad capture bound. It
# stops growing once the swing foot can cover the capture point.
HIP_OFFSET_LIMIT = 0.15
# Both feet stay planted during preparation, so the same capture mapping
# needs a wider clip than the walking correction. The joint range is
# unchanged. Only the PREPARE_STEP path reads this value.
PREPARE_CAPTURE_HIP_LIMIT = 0.25
HIP_OFFSET_RATE = 0.30  # rad/s
RELEASE_FORCE_N = 50.0
RELEASE_DWELL_S = 0.15
RELEASE_BLEND_S = 0.30


def horizontal_capture(com: np.ndarray, com_velocity: np.ndarray) -> tuple[np.ndarray, float]:
    """Capture point and ``omega`` from the measured centre of mass."""
    point = capture_point(com, com_velocity)
    height = float(np.asarray(com, dtype=float).reshape(-1)[2])
    return point, angular_velocity(height)


def bounded_foot_correction(
    nominal_xy: np.ndarray,
    capture_xy: np.ndarray,
    *,
    max_forward: float = MAX_FORWARD_CORRECTION,
    max_lateral: float = MAX_LATERAL_CORRECTION,
) -> np.ndarray:
    """Clip ``capture - nominal`` to the forward and lateral bounds.

    The result is added to the nominal landing. It is not a command to put
    the centre of mass on the foot.
    """
    nominal = np.asarray(nominal_xy, dtype=float).reshape(-1)[:2]
    capture = np.asarray(capture_xy, dtype=float).reshape(-1)[:2]
    error = capture - nominal
    return np.array(
        [
            float(np.clip(error[0], -abs(max_forward), abs(max_forward))),
            float(np.clip(error[1], -abs(max_lateral), abs(max_lateral))),
        ],
        dtype=float,
    )


def project_correction(
    nominal_xy: np.ndarray,
    correction_xy: np.ndarray,
    reachable,
) -> tuple[np.ndarray, bool]:
    """Pull ``correction`` toward zero until ``reachable(nominal + correction)``.

    ``reachable`` takes a world xy landing and returns a bool. The full
    correction is kept when it is already reachable. Otherwise a bisection
    finds the largest scale in ``[0, 1]`` that the IK accepts. The nominal
    landing is assumed reachable, so the search always has a solution.
    """
    nominal = np.asarray(nominal_xy, dtype=float).reshape(2).copy()
    correction = np.asarray(correction_xy, dtype=float).reshape(2).copy()
    if bool(reachable(nominal + correction)):
        return nominal + correction, False
    best = 0.0
    low = 0.0
    high = 1.0
    for _ in range(16):
        mid = 0.5 * (low + high)
        if bool(reachable(nominal + mid * correction)):
            best = mid
            low = mid
        else:
            high = mid
    return nominal + best * correction, True


def sole_contains_capture(capture_xy: np.ndarray, foot_xy: np.ndarray) -> bool:
    """True when the capture point lies on the sole of an ankle at ``foot_xy``."""
    capture = np.asarray(capture_xy, dtype=float).reshape(2)
    foot = np.asarray(foot_xy, dtype=float).reshape(2)
    dx = float(capture[0] - foot[0])
    dy = float(capture[1] - foot[1])
    return SOLE_HEEL - 1e-9 <= dx <= SOLE_TOE + 1e-9 and abs(dy) <= SOLE_HALF_WIDTH + 1e-9


def torque_lead(kp: float = SWING_HIP_KP, box: float = TORQUE_BOX) -> float:
    """Hip-angle lead whose PD torque equals the CBF torque box."""
    return abs(float(box)) / max(abs(float(kp)), 1e-6)


# A foot this far above its liftoff height has actually left the ground.
# Entering the swing state is not itself a lift.
LIFT_CLEARANCE = 0.02


def dynamic_q_interval(
    current_q: float,
    remaining_time: float,
    ramp_per_tick: float,
    control_dt: float,
    joint_low: float,
    joint_high: float,
    lead: float | None = None,
) -> tuple[float, float]:
    """Hip angles reachable from ``current_q`` before the swing ends.

    The command changes by ``ramp_per_tick`` each control tick. The joint is
    not assumed to jump to that command: a lead past ``lead`` asks the PD
    for more torque than the CBF box will pass, and the extra angle does not
    move the foot. The window is clipped to the joint stops. It is not the
    full travel of the joint from the home pose.
    """
    rate = abs(float(ramp_per_tick)) / max(float(control_dt), 1e-6)
    if lead is None:
        lead = torque_lead()
    return hip_track_interval(
        float(current_q),
        float(remaining_time),
        rate,
        float(joint_low),
        float(joint_high),
        float(lead),
    )


def dynamic_foot_interval(
    y_at_low: float,
    y_at_high: float,
    current_y: float,
) -> tuple[float, float]:
    """Lateral foot interval spanned by the dynamic hip window."""
    values = (float(y_at_low), float(y_at_high), float(current_y))
    return min(values), max(values)


def capture_required_interval(
    capture_y: float,
    *,
    half_width: float = SOLE_HALF_WIDTH,
    margin: float = 0.0,
) -> tuple[float, float]:
    """Ankle y whose sole contains ``capture_y``, inset by ``margin``."""
    half = abs(float(half_width))
    inset = abs(float(margin))
    return float(capture_y) - half + inset, float(capture_y) + half - inset


def intervals_intersect(a_low: float, a_high: float, b_low: float, b_high: float) -> bool:
    """True when the two closed intervals share a point."""
    lower = max(min(float(a_low), float(a_high)), min(float(b_low), float(b_high)))
    upper = min(max(float(a_low), float(a_high)), max(float(b_low), float(b_high)))
    return lower <= upper + 1e-9


def dynamic_catchable(
    predicted_capture_y: float,
    foot_y_min: float,
    foot_y_max: float,
    *,
    margin: float = 0.01,
) -> bool:
    """True when the predicted touchdown capture lies on a dynamically reachable sole."""
    low, high = feasible_capture_interval(foot_y_min, foot_y_max, margin=margin)
    return capture_is_prepared(predicted_capture_y, low, high)


def foot_is_airborne(foot_z: float, start_z: float, clearance: float = LIFT_CLEARANCE) -> bool:
    """True once the swing foot has risen ``clearance`` above its liftoff height."""
    if not np.isfinite(foot_z) or not np.isfinite(start_z):
        return False
    return float(foot_z) >= float(start_z) + abs(float(clearance))


def support_transfer_sample_ok(
    *,
    contact: bool,
    new_force: float,
    old_force: float,
    capture_inside: bool,
    vy: float,
    vx: float,
    roll: float,
    pitch: float,
    min_force: float = RELEASE_FORCE_N,
    max_abs_vy: float = 0.05,
    max_abs_vx: float = 0.08,
    max_abs_roll: float = 0.35,
    max_abs_pitch: float = 0.35,
) -> bool:
    """One sample of a stable new support.

    Force is required and is not enough. A falling impact can put hundreds
    of newtons on a foot while the body is still rotating and translating.
    """
    return bool(
        contact
        and float(new_force) >= float(min_force)
        and float(new_force) > float(old_force)
        and bool(capture_inside)
        and abs(float(vy)) < float(max_abs_vy)
        and abs(float(vx)) < float(max_abs_vx)
        and abs(float(roll)) < float(max_abs_roll)
        and abs(float(pitch)) < float(max_abs_pitch)
    )


def predict_touchdown_capture(
    com: np.ndarray,
    com_velocity: np.ndarray,
    remaining_time: float,
    accel: np.ndarray | None = None,
) -> tuple[np.ndarray, float]:
    """Capture point at touchdown from the measured horizontal motion.

    Position uses the current velocity over the remaining swing. Velocity at
    touchdown adds the measured acceleration over that same interval. There
    is no second planner: both numbers come from the state the step already
    measures. ``omega`` is ``sqrt(g / height)`` at the current height.
    """
    position = np.asarray(com, dtype=float).reshape(-1)
    velocity = np.asarray(com_velocity, dtype=float).reshape(-1)
    horizon = max(0.0, float(remaining_time))
    if accel is None:
        acceleration = np.zeros(2)
    else:
        acceleration = np.asarray(accel, dtype=float).reshape(-1)[:2]
    omega = angular_velocity(float(position[2]))
    predicted_com = position[:2] + velocity[:2] * horizon
    predicted_vel = velocity[:2] + acceleration * horizon
    return predicted_com + predicted_vel / omega, omega


def hip_track_interval(
    measured: float,
    remaining_time: float,
    ramp_rate: float,
    low: float,
    high: float,
    lead: float,
) -> tuple[float, float]:
    """Hip angles the swing can still reach before touchdown.

    The swing ramp limits how fast the command may move. ``lead`` limits how
    far that command may sit from the measured joint: past it, the PD torque
    exceeds the CBF box and the extra angle does not move the foot. The
    result is clipped to the joint range.
    """
    delta = min(max(0.0, float(ramp_rate)) * max(0.0, float(remaining_time)), abs(float(lead)))
    lower = max(float(low), float(measured) - delta)
    upper = min(float(high), float(measured) + delta)
    if lower > upper:
        held = float(np.clip(measured, min(low, high), max(low, high)))
        return held, held
    return lower, upper


def project_to_interval(value: float, low: float, high: float) -> float:
    """Clip one coordinate into ``[low, high]``."""
    return float(np.clip(value, min(float(low), float(high)), max(float(low), float(high))))


def sole_capture_margin(
    capture_xy: np.ndarray,
    ankle_xy: np.ndarray,
    margin: float = SOLE_MARGIN,
) -> float:
    """Signed distance of the capture point inside the sole.

    Positive means the point is inside the sole after ``margin`` is removed
    from each edge. Negative means it is outside, in metres.
    """
    capture = np.asarray(capture_xy, dtype=float).reshape(2)
    ankle = np.asarray(ankle_xy, dtype=float).reshape(2)
    dx = float(capture[0] - ankle[0])
    dy = float(capture[1] - ankle[1])
    inset = abs(float(margin))
    x_margin = min(dx - (SOLE_HEEL + inset), (SOLE_TOE - inset) - dx)
    y_margin = (SOLE_HALF_WIDTH - inset) - abs(dy)
    return float(min(x_margin, y_margin))


def landing_is_capture_valid(
    capture_xy: np.ndarray,
    ankle_xy: np.ndarray,
    margin: float = SOLE_MARGIN,
) -> bool:
    """True when the sole of an ankle at ``ankle_xy`` contains the capture point."""
    return sole_capture_margin(capture_xy, ankle_xy, margin) >= 0.0


def lateral_is_trackable(desired_y: float, y_min: float, y_max: float) -> bool:
    """True when ``desired_y`` is inside the swing's remaining lateral window."""
    lower = min(float(y_min), float(y_max))
    upper = max(float(y_min), float(y_max))
    return lower - 1e-9 <= float(desired_y) <= upper + 1e-9


def search_reachable_y(current_y: float, accepts, bracket: float = 0.8) -> tuple[float, float]:
    """Lateral ankle interval accepted by ``accepts``.

    ``accepts(y)`` is the existing leg IK. ``bracket`` is only how far the
    search looks; a point inside it is kept only when ``accepts`` says so.
    """
    origin = float(current_y)
    span = max(0.0, float(bracket))

    def extreme(sign: float) -> float:
        if not bool(accepts(origin)):
            return origin
        low = 0.0
        high = span
        best = 0.0
        for _ in range(14):
            mid = 0.5 * (low + high)
            if bool(accepts(origin + sign * mid)):
                best = mid
                low = mid
            else:
                high = mid
        return origin + sign * best

    left = extreme(-1.0)
    right = extreme(1.0)
    return min(left, right), max(left, right)


def feasible_capture_interval(
    foot_y_min: float,
    foot_y_max: float,
    *,
    half_width: float = SOLE_HALF_WIDTH,
    margin: float = 0.01,
) -> tuple[float, float]:
    """Capture-point y interval covered by reachable soles, inset by ``margin``.

    The whole sole counts. The capture point does not have to sit on the
    ankle.
    """
    low = min(float(foot_y_min), float(foot_y_max)) - abs(float(half_width))
    high = max(float(foot_y_min), float(foot_y_max)) + abs(float(half_width))
    inset = abs(float(margin))
    return low + inset, high - inset


# How many control ticks the catchability gap may grow before preparation
# stops pushing the same way and rebuilds the target from the new geometry.
PREPARE_WORSEN_TICKS = 5


def catchable_capture_interval(
    foot_y_min: float,
    foot_y_max: float,
    *,
    sole_half_width: float = SOLE_HALF_WIDTH,
    margin: float = 0.01,
) -> tuple[float, float]:
    """Capture-point y values some reachable sole can contain.

    Each ankle in ``[foot_y_min, foot_y_max]`` covers
    ``[ankle - sole_half_width, ankle + sole_half_width]``. The result is
    that union, inset by ``margin``. The capture point does not have to sit
    on the ankle or at y = 0.
    """
    return feasible_capture_interval(
        foot_y_min, foot_y_max, half_width=sole_half_width, margin=margin
    )


def interval_distance(value: float, low: float, high: float) -> float:
    """How far ``value`` is outside ``[low, high]``. Zero when it is inside."""
    lower = min(float(low), float(high))
    upper = max(float(low), float(high))
    point = float(value)
    if lower <= point <= upper:
        return 0.0
    if point < lower:
        return float(lower - point)
    return float(point - upper)


def select_capture_target(current_capture_y: float, low: float, high: float) -> float:
    """Nearest capture y inside the catchable interval.

    A capture point that is already inside is left where it is. One that is
    outside is brought to the nearest edge. The result is not forced to zero.
    """
    return preparation_target_y(current_capture_y, low, high)


def prepare_lateral_velocity(
    com_y: float,
    target_capture_y: float,
    omega: float,
    *,
    max_lateral: float,
    hold: bool,
) -> float:
    """CoM velocity whose capture point is ``target_capture_y``.

    ``v = omega * (target - com)``, clipped to the existing lateral speed
    cap. ``hold`` drops the command when that motion has been widening the
    catchability gap.
    """
    if hold:
        return 0.0
    raw = float(omega) * (float(target_capture_y) - float(com_y))
    limit = abs(float(max_lateral))
    return float(np.clip(raw, -limit, limit))


def update_worsen_count(previous_gap: float | None, gap: float, count: int) -> int:
    """Count consecutive ticks on which the catchability gap grew."""
    if previous_gap is None or not np.isfinite(float(previous_gap)):
        return 0
    if float(gap) > float(previous_gap) + 1e-4:
        return int(count) + 1
    return 0


def prepare_hip_targets(
    home: float,
    current_capture_y: float,
    target_capture_y: float,
    *,
    gain: float = 12.0,
    limit: float = PREPARE_CAPTURE_HIP_LIMIT,
) -> tuple[float, float, float]:
    """Unsaturated hip angle, clipped angle, and the clip error.

    The offset is ``gain * (current_capture_y - target_capture_y)``, the same
    term the walking capture correction uses. Saturation error is the
    unsaturated angle minus the angle after the ``limit`` clip. It is zero
    when the capture error fits inside the bound, and it changes sign with
    the capture error.
    """
    raw = float(gain) * (float(current_capture_y) - float(target_capture_y))
    bound = abs(float(limit))
    clipped_offset = float(np.clip(raw, -bound, bound))
    unsaturated = float(home) + raw
    clipped = float(home) + clipped_offset
    return unsaturated, clipped, unsaturated - clipped


def prepare_hip_roll_command(
    home: float,
    current_capture_y: float,
    target_capture_y: float,
    previous: float,
    *,
    ramp: float,
    gain: float = 12.0,
    limit: float = HIP_OFFSET_LIMIT,
) -> float:
    """Hip angle that moves the capture point toward ``target_capture_y``.

    The offset is the existing lateral capture term. A target below the
    measured capture point returns a positive angle. On the H1 that positive
    angle, on either planted hip, moves the pelvis toward negative y. The
    command steps from the previous command by at most ``ramp``, then stays
    inside ``home ± limit``. It does not add a further offset when a measured
    angle is already past the command: the previous argument is the last
    command, and PD tracks that command.
    """
    _unsaturated, desired, _saturation = prepare_hip_targets(
        home,
        current_capture_y,
        target_capture_y,
        gain=gain,
        limit=limit,
    )
    held = float(home) if not np.isfinite(previous) else float(previous)
    step = abs(float(ramp))
    stepped = held + float(np.clip(desired - held, -step, step))
    bound = abs(float(limit))
    return float(np.clip(stepped, float(home) - bound, float(home) + bound))


def prepare_hip_trace(
    mpc_q: float,
    prepare_q: float,
    after_override: float,
    before_pd: float,
    tau_nominal: float,
    tau_safe: float,
) -> dict[str, float]:
    """The stance-hip values that show where a prepare command was lost."""
    return {
        "q_des_mpc_hip_roll": float(mpc_q),
        "q_des_prepare_hip_roll": float(prepare_q),
        "q_des_after_walking_override": float(after_override),
        "q_des_before_pd": float(before_pd),
        "tau_nominal_hip_roll": float(tau_nominal),
        "tau_safe_hip_roll": float(tau_safe),
        "cbf_intervention_hip_roll": float(tau_nominal) - float(tau_safe),
    }


def liftoff_permitted(
    *,
    predicted_inside: bool,
    speed_ok: bool,
    dwell_met: bool,
    static_reachable: bool = False,
) -> bool:
    """Swing may start only from a dynamically catchable, slow capture state.

    ``static_reachable`` is accepted and ignored. A static IK success does
    not release the foot.
    """
    del static_reachable
    return bool(predicted_inside and speed_ok and dwell_met)


def preparation_target_y(capture_y: float, low: float, high: float) -> float:
    """Nearest capture y inside ``[low, high]``. The value itself when inside."""
    return float(np.clip(capture_y, min(float(low), float(high)), max(float(low), float(high))))


def capture_is_prepared(capture_y: float, low: float, high: float) -> bool:
    """True when the capture point is inside the feasible interval."""
    lower = min(float(low), float(high))
    upper = max(float(low), float(high))
    return lower - 1e-9 <= float(capture_y) <= upper + 1e-9


def candidate_ankle_y(
    capture_y: float,
    foot_y_min: float,
    foot_y_max: float,
    *,
    half_width: float = SOLE_HALF_WIDTH,
) -> float:
    """Reachable ankle whose sole contains the capture point, else the nearest ankle."""
    lower = min(float(foot_y_min), float(foot_y_max))
    upper = max(float(foot_y_min), float(foot_y_max))
    cover_low = max(lower, float(capture_y) - abs(float(half_width)))
    cover_high = min(upper, float(capture_y) + abs(float(half_width)))
    if cover_low <= cover_high:
        return float(np.clip(capture_y, cover_low, cover_high))
    return float(np.clip(capture_y, lower, upper))


def fall_or_recover(progress: float, catchable: bool) -> tuple[bool, bool]:
    """``(building, recover)`` for one swing tick.

    Early swing may still build the inward velocity. Once the landing phase
    of the check is active, an uncatchable predicted capture point brakes
    that fall instead of increasing it.
    """
    recover = float(progress) >= TRACK_PROGRESS and not bool(catchable)
    building = float(progress) >= RAMP_START and not recover and not bool(catchable)
    return building, recover


def correction_saturated(
    correction_xy: np.ndarray,
    *,
    max_forward: float = MAX_FORWARD_CORRECTION,
    max_lateral: float = MAX_LATERAL_CORRECTION,
) -> bool:
    """True when either axis is on its bound."""
    correction = np.asarray(correction_xy, dtype=float).reshape(2)
    return bool(
        abs(correction[0]) >= abs(max_forward) - 1e-4
        or abs(correction[1]) >= abs(max_lateral) - 1e-4
    )


@dataclass
class CaptureFootPlan:
    """One tick of the swing-foot landing decision."""

    requested_xy: np.ndarray
    accepted_xy: np.ndarray
    commanded_xy: np.ndarray
    correction_xy: np.ndarray
    reachable: bool
    fallback: bool
    frozen: bool
    catchable: bool
    building: bool
    swing_trackable: bool = False
    hip_roll_limit_active: bool = False
    recover: bool = False
    predicted_capture_xy: np.ndarray = field(default_factory=lambda: np.full(2, np.nan))
    feasible_min_y: float = float("nan")
    feasible_max_y: float = float("nan")
    foot_y_error: float = float("nan")
    remaining_time: float = float("nan")
    actual_foot_y: float = float("nan")
    landing_target_y: float = float("nan")
    capture_margin: float = float("nan")
    ik_reachable: bool = False


class CaptureStepTracker:
    """Ramp, slew and freeze one swing's capture landing.

    ``reset`` is called when the swing foot changes. After the freeze, or
    once touchdown has stopped adaptation, the commanded landing stays put
    even if the capture point moves.
    """

    def __init__(self) -> None:
        self.swing = ""
        self.frozen_xy: np.ndarray | None = None
        self._command_xy: np.ndarray | None = None
        self.reachable = True
        self.fallback = False
        self.requested_xy = np.full(2, np.nan)
        self.accepted_xy = np.full(2, np.nan)
        self.correction_xy = np.zeros(2)

    def reset(self, swing: str) -> None:
        self.swing = str(swing)
        self.frozen_xy = None
        self._command_xy = None
        self.reachable = True
        self.fallback = False
        self.requested_xy = np.full(2, np.nan)
        self.accepted_xy = np.full(2, np.nan)
        self.correction_xy = np.zeros(2)

    def plan(
        self,
        nominal_xy: np.ndarray,
        capture_xy: np.ndarray,
        progress: float,
        reachable,
        *,
        adapting: bool,
        dt: float,
    ) -> CaptureFootPlan:
        """Return the landing to track on this tick."""
        nominal = np.asarray(nominal_xy, dtype=float).reshape(2).copy()
        capture = np.asarray(capture_xy, dtype=float).reshape(2)
        requested_correction = bounded_foot_correction(nominal, capture)
        requested = nominal + requested_correction
        self.requested_xy = requested.copy()
        if self.frozen_xy is not None or not adapting or float(progress) >= FREEZE_PROGRESS:
            if self.frozen_xy is None:
                accepted, fallback = project_correction(nominal, requested_correction, reachable)
                self.accepted_xy = accepted.copy()
                self.fallback = bool(fallback)
                self.reachable = bool(reachable(accepted))
                # Freeze the landing the foot is already tracking. Falling
                # through to the unslewed target would step the foot sideways
                # on the tick the landing phase begins.
                self.frozen_xy = (
                    self._command_xy.copy() if self._command_xy is not None else accepted.copy()
                )
            commanded = self.frozen_xy.copy()
            self._command_xy = commanded.copy()
            correction = commanded - nominal
            self.correction_xy = correction.copy()
            catchable = sole_contains_capture(capture, commanded)
            return CaptureFootPlan(
                requested.copy(),
                self.accepted_xy.copy(),
                commanded,
                correction,
                bool(self.reachable),
                bool(self.fallback),
                True,
                catchable,
                False,
            )
        accepted, fallback = project_correction(nominal, requested_correction, reachable)
        self.accepted_xy = accepted.copy()
        self.fallback = bool(fallback)
        self.reachable = bool(reachable(accepted))
        weight = placement_ramp(float(progress), RAMP_START, FREEZE_PROGRESS)
        target = nominal + weight * (accepted - nominal)
        commanded = self._slew(nominal if self._command_xy is None else self._command_xy, target, dt)
        self._command_xy = commanded.copy()
        correction = commanded - nominal
        self.correction_xy = correction.copy()
        catchable = sole_contains_capture(capture, accepted)
        # Keep building body velocity until the swing foot can cover the
        # capture point. A correction that is already on its bound is not a
        # reason to stop: the capture point still has to move into reach.
        # The hip offset itself is clipped, so the fall cannot grow forever.
        building = float(progress) >= RAMP_START and not catchable
        return CaptureFootPlan(
            requested.copy(),
            accepted.copy(),
            commanded,
            correction,
            bool(self.reachable),
            bool(self.fallback),
            False,
            catchable,
            building,
        )

    def _slew(self, current: np.ndarray, target: np.ndarray, dt: float) -> np.ndarray:
        current = np.asarray(current, dtype=float).reshape(2)
        target = np.asarray(target, dtype=float).reshape(2)
        delta = target - current
        step = max(0.0, TARGET_SLEW) * max(0.0, float(dt))
        distance = float(np.linalg.norm(delta))
        if distance > step and distance > 1e-12:
            delta = delta / distance * step
        return current + delta

    def plan_trackable(
        self,
        nominal_xy: np.ndarray,
        predicted_capture_xy: np.ndarray,
        actual_xy: np.ndarray,
        feasible_y: tuple[float, float],
        progress: float,
        remaining_time: float,
        reachable,
        *,
        adapting: bool,
        dt: float,
        hip_limit_active: bool = False,
        ik_reachable: bool = False,
        emergency: bool = False,
    ) -> CaptureFootPlan:
        """Land at the trackable point nearest the predicted capture sole.

        The desired ankle is the predicted capture point, clipped by the
        existing forward and lateral bounds. Its lateral coordinate is then
        pulled into ``feasible_y``, which is the window the swing can still
        reach. The command slews there from the actual foot. Sagittal ``x``
        is not passed through that window.
        """
        nominal = np.asarray(nominal_xy, dtype=float).reshape(2).copy()
        predicted = np.asarray(predicted_capture_xy, dtype=float).reshape(2).copy()
        actual = np.asarray(actual_xy, dtype=float).reshape(2).copy()
        lower = min(float(feasible_y[0]), float(feasible_y[1]), float(actual[1]))
        upper = max(float(feasible_y[0]), float(feasible_y[1]), float(actual[1]))
        desired = nominal + bounded_foot_correction(nominal, predicted)
        self.requested_xy = desired.copy()
        trackable = lateral_is_trackable(float(desired[1]), lower, upper)
        accepted = np.array(
            [float(desired[0]), project_to_interval(float(desired[1]), lower, upper)],
            dtype=float,
        )
        fallback = False
        if reachable is not None and not bool(reachable(accepted)):
            origin = np.array([float(nominal[0]), float(actual[1])], dtype=float)
            if bool(reachable(origin)):
                accepted, fallback = project_correction(origin, accepted - origin, reachable)
                accepted[1] = project_to_interval(float(accepted[1]), lower, upper)
            else:
                fallback = True
        self.accepted_xy = accepted.copy()
        self.fallback = bool(fallback)
        self.reachable = bool(ik_reachable)
        margin = sole_capture_margin(predicted, accepted)
        catchable = bool(landing_is_capture_valid(predicted, accepted))
        # Braking is an emergency only. A step that was prepared does not
        # reverse the fall just because the foot has not arrived yet.
        recover = bool(emergency)
        building = float(progress) >= RAMP_START and not recover and not catchable
        frozen = self.frozen_xy is not None or not adapting or float(progress) >= FREEZE_PROGRESS
        if frozen:
            if self.frozen_xy is None:
                self.frozen_xy = (
                    self._command_xy.copy() if self._command_xy is not None else accepted.copy()
                )
            commanded = self.frozen_xy.copy()
            building = False
        else:
            start = actual if self._command_xy is None else self._command_xy
            commanded = self._slew(start, accepted, dt)
        self._command_xy = commanded.copy()
        correction = commanded - nominal
        self.correction_xy = correction.copy()
        return CaptureFootPlan(
            desired.copy(),
            accepted.copy(),
            commanded.copy(),
            correction.copy(),
            bool(ik_reachable),
            bool(fallback),
            bool(frozen),
            catchable,
            bool(building),
            swing_trackable=bool(trackable),
            hip_roll_limit_active=bool(hip_limit_active),
            recover=bool(recover),
            predicted_capture_xy=predicted.copy(),
            feasible_min_y=float(lower),
            feasible_max_y=float(upper),
            foot_y_error=float(accepted[1] - actual[1]),
            remaining_time=float(remaining_time),
            actual_foot_y=float(actual[1]),
            landing_target_y=float(accepted[1]),
            capture_margin=float(margin),
            ik_reachable=bool(ik_reachable),
        )


class CaptureBodyPrep:
    """Stance-hip angle that builds a fall and then holds it.

    The angle is an absolute joint target. While ``building`` is true it
    moves inward from the weight-shift pose, at ``HIP_OFFSET_RATE``, and
    never more than ``HIP_OFFSET_LIMIT`` from that pose. When the new foot
    has carried load for the dwell, the command blends toward the home
    angle instead of handing the joint back to the weight-shift lean.
    """

    def __init__(self) -> None:
        self.side = ""
        self.base = 0.0
        self.command = 0.0
        self._force_time = 0.0
        self.releasing = False
        self.active = False

    def reset(self) -> None:
        self.side = ""
        self.base = 0.0
        self.command = 0.0
        self._force_time = 0.0
        self.releasing = False
        self.active = False

    def update(
        self,
        side: str,
        base: float,
        home: float,
        building: bool,
        new_force: float,
        dt: float,
        braking: bool = False,
    ) -> float:
        if not self.active or side != self.side:
            self.side = str(side)
            self.base = float(base)
            self.command = float(base)
            self._force_time = 0.0
            self.releasing = False
            self.active = True
        if float(new_force) >= RELEASE_FORCE_N:
            self._force_time += max(0.0, float(dt))
        else:
            self._force_time = 0.0
        if self._force_time >= RELEASE_DWELL_S:
            self.releasing = True
        if bool(braking):
            # An uncatchable step returns the stance hip toward the lean that
            # was holding the body on the support foot. It does not add more
            # inward angle, and it does not hand the joint back to home while
            # the capture point is still outside the swing sole.
            self.releasing = False
            self._force_time = 0.0
            step = HIP_OFFSET_RATE * max(0.0, float(dt))
            delta = float(self.base) - float(self.command)
            self.command = float(self.command) + float(np.clip(delta, -step, step))
        elif self.releasing:
            blend = 0.0 if RELEASE_BLEND_S <= 0.0 else min(1.0, max(0.0, float(dt)) / RELEASE_BLEND_S)
            self.command = float(self.command) + blend * (float(home) - float(self.command))
        elif building:
            sign = 1.0 if side == "left" else -1.0
            moved = float(self.command) + sign * HIP_OFFSET_RATE * max(0.0, float(dt))
            limit = abs(HIP_OFFSET_LIMIT)
            self.command = float(np.clip(moved, self.base - limit, self.base + limit))
        return float(self.command)


def omega_from_height(com_z: float) -> float:
    """``sqrt(g / z)`` with the same floor as the capture-point helper."""
    return angular_velocity(com_z)


# Re-exported so callers do not need a second import for the gravity constant.
__all__ = [
    "GRAVITY",
    "CaptureBodyPrep",
    "CaptureFootPlan",
    "CaptureStepTracker",
    "FREEZE_PROGRESS",
    "MAX_FORWARD_CORRECTION",
    "MAX_LATERAL_CORRECTION",
    "bounded_foot_correction",
    "correction_saturated",
    "candidate_ankle_y",
    "catchable_capture_interval",
    "capture_is_prepared",
    "capture_required_interval",
    "dynamic_catchable",
    "dynamic_foot_interval",
    "dynamic_q_interval",
    "fall_or_recover",
    "foot_is_airborne",
    "interval_distance",
    "intervals_intersect",
    "liftoff_permitted",
    "feasible_capture_interval",
    "hip_track_interval",
    "horizontal_capture",
    "landing_is_capture_valid",
    "lateral_is_trackable",
    "omega_from_height",
    "predict_touchdown_capture",
    "prepare_hip_roll_command",
    "prepare_hip_targets",
    "prepare_hip_trace",
    "PREPARE_CAPTURE_HIP_LIMIT",
    "prepare_lateral_velocity",
    "PREPARE_WORSEN_TICKS",
    "project_correction",
    "preparation_target_y",
    "project_to_interval",
    "search_reachable_y",
    "select_capture_target",
    "update_worsen_count",
    "support_transfer_sample_ok",
    "sole_capture_margin",
    "sole_contains_capture",
    "torque_lead",
]
