"""Double-support load transfer after a confirmed touchdown.

The schedule says how the body should move from the old stance foot toward
the foot that just landed. ``new_fraction`` and ``old_fraction`` are desired
shares of that motion. They are not ground-reaction-force commands, and this
module does not compute a contact force or a joint torque.

What it does produce is a lateral centre-of-mass target and a hip-roll
offset. The existing recovery reference keeps its forward component. The
offset is added on top of the hip-roll target the shift and the MPC already
produced, then the usual path applies it:

    q_des -> PD + gravity -> CBF -> torque

``hold_touchdown_hip_roll`` still chooses the base hip-roll target. When it
is set, that base is the pre-touchdown blend and the load-transfer offset is
added on top. When it is clear, the base is the blend plus the lateral
capture term, and the same offset is added on top. The offset itself stays
inside ``max_hip_roll``.

Limitation: there is no contact-force optimizer. A few centimetres of
lateral reference and a few hundredths of a radian of hip roll can only
encourage the new foot to take weight. The measured normal force may stay
on the old foot.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LoadTransferCommand:
    """One tick of the load-transfer schedule.

    ``active`` is true only in double-support recovery. Outside that state
    the hip-roll offset is zero and the lateral target is left unchanged.
    """

    active: bool
    alpha: float
    new_fraction: float
    old_fraction: float
    target_x: float
    target_y: float
    hip_roll: float
    complete: bool


@dataclass
class LoadTransferTracker:
    """Latch whether the measured load share held through the final window.

    The nominal duration ending does not by itself count as success. During
    the final quarter of the schedule the new foot must carry at least
    ``min_fraction`` of the measured vertical force, with both feet in
    contact and the existing balance checks true, for ``dwell`` seconds.
    """

    success: bool = False
    start_time: float = np.nan
    _hold: float = 0.0

    def observe(
        self,
        *,
        active: bool,
        time: float,
        sample_ok: bool,
        in_final_window: bool,
        dwell: float,
        dt: float,
    ) -> None:
        if not active:
            self._hold = 0.0
            return
        if not np.isfinite(self.start_time):
            self.start_time = float(time)
        if sample_ok and in_final_window:
            self._hold += max(0.0, float(dt))
        else:
            self._hold = 0.0
        if self._hold >= float(dwell) and float(dwell) > 0.0:
            self.success = True


def smoothstep(progress: float) -> float:
    """Cubic smoothstep on ``[0, 1]``. Zero at the start, one at the end."""
    s = float(np.clip(progress, 0.0, 1.0))
    return (3.0 * s * s) - (2.0 * s * s * s)


def load_fractions(time_since: float, duration: float) -> tuple[float, float]:
    """Return ``(new_fraction, old_fraction)``. They sum to one.

    A non-positive duration jumps to the end of the schedule. The caller
    still has to keep the recovery exit behind the stability dwell.
    """
    if float(duration) <= 0.0:
        alpha = 1.0
    else:
        alpha = smoothstep(float(time_since) / float(duration))
    return alpha, 1.0 - alpha


def lateral_transfer_target(
    old_y: float,
    new_y: float,
    alpha: float,
    max_offset: float,
) -> float:
    """Lateral target between the two soles, no farther than ``max_offset``.

    ``alpha`` is the new-foot fraction. The step away from the old sole is
    clipped, so a wide stance does not ask for the whole gap at once. The
    result stays between the two measured sole centres.
    """
    span = float(new_y) - float(old_y)
    step = float(np.clip(float(alpha) * span, -abs(float(max_offset)), abs(float(max_offset))))
    goal = float(old_y) + step
    low = min(float(old_y), float(new_y))
    high = max(float(old_y), float(new_y))
    if high <= low:
        return float(old_y)
    return float(np.clip(goal, low, high))


def hip_roll_correction(old_y: float, new_y: float, alpha: float, max_correction: float) -> float:
    """Small hip-roll offset that leans the body toward the new sole.

    On the H1 both hip-roll axes point the same way. A negative command on
    both legs moves the centre of mass toward +y, so the sign here is the
    opposite of ``new_y - old_y``. The magnitude grows with ``alpha`` and
    never exceeds ``max_correction``.
    """
    limit = abs(float(max_correction))
    direction = float(new_y) - float(old_y)
    if abs(direction) <= 1e-9 or limit == 0.0:
        return 0.0
    sign = -1.0 if direction > 0.0 else 1.0
    return float(np.clip(sign * float(alpha) * limit, -limit, limit))


def plan_load_transfer(
    *,
    state: str,
    time_since: float,
    duration: float,
    old_y: float,
    new_y: float,
    reference_x: float,
    reference_y: float,
    max_com_offset: float,
    max_hip_roll: float,
) -> LoadTransferCommand:
    """Schedule for this tick. Inactive outside double-support recovery."""
    if state != "DOUBLE_SUPPORT_RECOVERY":
        return LoadTransferCommand(
            active=False,
            alpha=0.0,
            new_fraction=0.0,
            old_fraction=1.0,
            target_x=float(reference_x),
            target_y=float(reference_y),
            hip_roll=0.0,
            complete=False,
        )
    new_fraction, old_fraction = load_fractions(time_since, duration)
    return LoadTransferCommand(
        active=True,
        alpha=new_fraction,
        new_fraction=new_fraction,
        old_fraction=old_fraction,
        target_x=float(reference_x),
        target_y=lateral_transfer_target(old_y, new_y, new_fraction, max_com_offset),
        hip_roll=hip_roll_correction(old_y, new_y, new_fraction, max_hip_roll),
        complete=float(duration) <= 0.0 or float(time_since) >= float(duration),
    )


def final_window(time_since: float, duration: float) -> bool:
    """True during the last quarter of the nominal transfer."""
    if float(duration) <= 0.0:
        return True
    return float(time_since) >= 0.75 * float(duration)


def load_sample_ok(
    *,
    new_force: float,
    old_force: float,
    both_contact: bool,
    com_z: float,
    vy: float,
    capture_inside: bool,
    cbf_status: str,
    min_fraction: float,
    max_vy: float,
    fall_height: float = 0.72,
) -> bool:
    """One tick of the measured load-transfer check.

    ``min_fraction`` is a share of the summed measured normal force, not of
    body weight. A missing force reading cannot pass.
    """
    total = float(new_force) + float(old_force)
    if total <= 1e-6:
        share = 0.0
    else:
        share = float(new_force) / total
    return bool(
        both_contact
        and share >= float(min_fraction)
        and float(com_z) > float(fall_height)
        and abs(float(vy)) <= float(max_vy)
        and bool(capture_inside)
        and str(cbf_status) != "fail_safe"
    )
