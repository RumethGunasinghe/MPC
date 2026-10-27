"""Walking state machine with a weight shift before every swing.

Lifting a foot while the centre of mass is still between the feet removes
that foot's support and the robot falls. The machine therefore never opens
a swing until the centre of mass has been moved over the stance foot:

    both feet
        -> shift centre of mass left
        -> lift, advance and land the right foot
        -> both feet down
        -> shift centre of mass right
        -> lift, advance and land the left foot
        -> both feet down
        -> ...

States
------
``STAND``
    Both feet planted. The centre-of-mass target sits halfway between them.
``SHIFT_LEFT`` / ``SHIFT_RIGHT``
    Both feet stay on the ground. The centre-of-mass target moves onto the
    foot named by the state. The other foot is not allowed to lift.
``RIGHT_SWING`` / ``LEFT_SWING``
    The opposite foot is the support. The swinging foot follows the cubic
    lift / move-forward / land trajectory. ``step_duration`` is that
    trajectory's clock. It is not a contact measurement.
``TOUCHDOWN_PREP``
    The last part of the swing. The foot is still on the trajectory. The
    lateral reference eases inward inside the existing 8 mm band. It does
    not jump.
``TOUCHDOWN``
    The trajectory has reached its landing and the foot command is held
    there. The phase ends only after several consecutive settled-contact
    samples. A timer cannot confirm the landing.
``DOUBLE_SUPPORT_RECOVERY``
    Both feet are down. The next shift starts only after the centre of
    mass and the capture point are inside the contact-qualified support,
    the velocity and the pelvis are inside their limits, and those
    conditions have held for the dwell. ``recovery_duration`` is a minimum
    residence. It does not, by itself, start the next shift.
``DOUBLE_SUPPORT``
    Kept for the open-loop :meth:`sample` schedule. The closed-loop
    :meth:`update` path uses the touchdown phases above instead.

A swing is entered only from the shift that loaded the opposite foot.
Leaving a shift still requires the support gate. Leaving recovery does not.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from controllers.adaptive_foot_placement import (
    SOLE_HALF_WIDTH,
    lateral_landing_correction,
    placement_ramp,
)
from controllers.footstep_planner import SwingFootTrajectory

_FEET = ("left", "right")


class WalkState(Enum):
    """Discrete phase of the gait."""

    STAND = "STAND"
    SHIFT_LEFT = "SHIFT_LEFT"
    RIGHT_SWING = "RIGHT_SWING"
    TOUCHDOWN_PREP = "TOUCHDOWN_PREP"
    TOUCHDOWN = "TOUCHDOWN"
    DOUBLE_SUPPORT_RECOVERY = "DOUBLE_SUPPORT_RECOVERY"
    DOUBLE_SUPPORT = "DOUBLE_SUPPORT"
    SHIFT_RIGHT = "SHIFT_RIGHT"
    LEFT_SWING = "LEFT_SWING"


# Support foot required while the machine is *in* that state. "both" means
# neither foot may leave the ground. A swing names the stance foot, which is
# the foot the centre of mass was shifted onto in the previous state.
_SUPPORT = {
    WalkState.STAND: "both",
    WalkState.SHIFT_LEFT: "left",
    WalkState.RIGHT_SWING: "left",
    WalkState.DOUBLE_SUPPORT: "both",
    WalkState.SHIFT_RIGHT: "right",
    WalkState.LEFT_SWING: "right",
}


@dataclass
class WalkConfig:
    """Timing of one weight shift and one swing.

    Attributes:
        step_length: Forward distance from the stance foot to the next
            landing, in metres. Same meaning as in the footstep planner.
        step_height: Swing clearance, in metres.
        step_duration: Seconds spent in ``RIGHT_SWING`` or ``LEFT_SWING``.
            The cubic trajectory splits this into lift, move forward, land.
        shift_duration: Seconds spent moving the centre of mass onto the
            stance foot before the opposite foot is allowed to lift.
        double_support_duration: Seconds of double support in the open-loop
            :meth:`WalkingStateMachine.sample` schedule. The closed-loop
            recovery does not treat this timer as permission to shift again.
        stand_duration: Seconds in ``STAND`` before the first shift, and
            again at the end while the centre of mass returns to the
            middle of the two feet.
        forward: Unit direction of travel. The default is world +x.
        touchdown_confirm_ticks: Consecutive settled-contact samples required
            before ``TOUCHDOWN`` becomes ``DOUBLE_SUPPORT_RECOVERY``.
        touchdown_prep_duration: Seconds at the end of the swing spent in
            ``TOUCHDOWN_PREP``. The foot is still on the cubic.
        touchdown_height: Maximum swing-ankle height, in metres, for one
            settled-contact sample.
        touchdown_vz_min: Minimum swing-foot vertical velocity, in m/s, for
            one settled-contact sample. A foot that is still dropping quickly
            does not count.
        max_abs_vy: Lateral centre-of-mass speed, in m/s, that recovery may
            not exceed. Matches the swing gate.
        max_abs_vx: Forward centre-of-mass speed, in m/s, that recovery may
            not exceed. Matches the swing gate.
        capture_margin: Metres inset from each face of the contact-qualified
            box. The centre of mass, the capture point, and the recovery
            reference use this inset.
        recovery_reference_speed: How fast the recovery reference may move,
            in m/s. The reference slews. It does not jump onto the body.
        recovery_duration: Minimum seconds in ``DOUBLE_SUPPORT_RECOVERY``.
            Reaching this time does not start the next shift.
        stabilization_dwell: Seconds the recovery conditions must hold
            continuously before the next shift or the final stand.
        max_abs_roll: Pelvis roll limit during recovery, in radians.
        max_abs_pitch: Pelvis pitch limit during recovery, in radians.
        landed_ankle_capture: When false, the swing foot that has entered
            ``TOUCHDOWN`` or ``DOUBLE_SUPPORT_RECOVERY`` keeps its landing
            ankle target. The sagittal capture offset is not added to that
            ankle. Hip roll is unaffected. The stance ankle still receives
            the offset.
        hold_touchdown_hip_roll: Experiment. When true, recovery keeps the
            pre-touchdown hip-roll blend instead of adding the lateral
            capture term. The capture path remains in the controller. The
            load-transfer hip-roll offset is still added on top of that blend.
        load_transfer_duration: Nominal seconds of the double-support load
            schedule. Reaching the end does not leave recovery.
        load_transfer_max_com_offset: Largest lateral move of the recovery
            reference away from the old sole, in metres.
        load_transfer_max_hip_roll: Largest load-transfer hip-roll offset, in
            radians. This bound is the offset, not the whole joint command.
        load_transfer_success_fraction: Minimum share of the measured normal
            force the new foot must carry in the final quarter of the schedule.
        load_transfer_success_dwell: Seconds that share must hold, continuously,
            before the load-transfer check latches.
        adaptive_foot_placement: When true, late swing may move the existing
            lateral landing target toward the capture point. The forward step
            length is unchanged. Default false leaves the landing where
            :func:`_landing` put it.
        capture_footstep_gain: Fraction of the sole-geometry correction that
            is applied. One would aim at the ideal ankle; the default keeps
            half of that move.
        max_lateral_step_adjustment: Largest lateral change of the swing
            landing, in metres. The correction is clamped to this, both ways.
            The default is 0.04 m, the first lateral distance the swing IK
            is being asked to execute.
        adaptive_placement_start: Swing progress at which the lateral
            correction begins to ramp in. Earlier progress leaves the
            original target unchanged.
    """

    step_length: float = 0.1
    step_height: float = 0.05
    step_duration: float = 0.8
    shift_duration: float = 0.8
    double_support_duration: float = 0.4
    stand_duration: float = 0.5
    forward: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    touchdown_confirm_ticks: int = 3
    touchdown_prep_duration: float = 0.20
    touchdown_height: float = 0.09
    touchdown_vz_min: float = -0.20
    max_abs_vy: float = 0.03
    max_abs_vx: float = 0.08
    capture_margin: float = 0.01
    recovery_reference_speed: float = 0.15
    recovery_duration: float = 0.0
    stabilization_dwell: float = 0.20
    max_abs_roll: float = 0.35
    max_abs_pitch: float = 0.35
    landed_ankle_capture: bool = False
    hold_touchdown_hip_roll: bool = False
    load_transfer_duration: float = 0.40
    load_transfer_max_com_offset: float = 0.04
    load_transfer_max_hip_roll: float = 0.08
    load_transfer_success_fraction: float = 0.25
    load_transfer_success_dwell: float = 0.10
    adaptive_foot_placement: bool = False
    capture_footstep_gain: float = 0.5
    max_lateral_step_adjustment: float = 0.04
    adaptive_placement_start: float = 0.65

    def __post_init__(self) -> None:
        self.forward = np.asarray(self.forward, dtype=float).reshape(3)
        norm = float(np.linalg.norm(self.forward))
        if norm <= 0.0:
            raise ValueError("forward must be a non-zero 3-vector")
        self.forward = self.forward / norm
        if self.step_length < 0.0:
            raise ValueError("step_length must be non-negative")
        if self.step_height < 0.0:
            raise ValueError("step_height must be non-negative")
        if self.step_duration <= 0.0:
            raise ValueError("step_duration must be positive")
        if self.shift_duration <= 0.0:
            raise ValueError("shift_duration must be positive")
        if self.double_support_duration < 0.0:
            raise ValueError("double_support_duration must be non-negative")
        if self.stand_duration < 0.0:
            raise ValueError("stand_duration must be non-negative")
        if int(self.touchdown_confirm_ticks) < 1:
            raise ValueError("touchdown_confirm_ticks must be at least 1")
        self.touchdown_confirm_ticks = int(self.touchdown_confirm_ticks)
        if self.touchdown_prep_duration < 0.0:
            raise ValueError("touchdown_prep_duration must be non-negative")
        if self.recovery_reference_speed < 0.0:
            raise ValueError("recovery_reference_speed must be non-negative")
        if self.recovery_duration < 0.0:
            raise ValueError("recovery_duration must be non-negative")
        if self.stabilization_dwell < 0.0:
            raise ValueError("stabilization_dwell must be non-negative")
        if self.capture_margin < 0.0:
            raise ValueError("capture_margin must be non-negative")
        if self.max_abs_vx < 0.0 or self.max_abs_vy < 0.0:
            raise ValueError("velocity limits must be non-negative")
        if self.load_transfer_duration < 0.0:
            raise ValueError("load_transfer_duration must be non-negative")
        if self.load_transfer_max_com_offset < 0.0:
            raise ValueError("load_transfer_max_com_offset must be non-negative")
        if self.load_transfer_max_hip_roll < 0.0:
            raise ValueError("load_transfer_max_hip_roll must be non-negative")
        if not 0.0 <= self.load_transfer_success_fraction <= 1.0:
            raise ValueError("load_transfer_success_fraction must be in [0, 1]")
        if self.load_transfer_success_dwell < 0.0:
            raise ValueError("load_transfer_success_dwell must be non-negative")
        if self.capture_footstep_gain < 0.0:
            raise ValueError("capture_footstep_gain must be non-negative")
        if self.max_lateral_step_adjustment < 0.0:
            raise ValueError("max_lateral_step_adjustment must be non-negative")
        if not 0.0 <= float(self.adaptive_placement_start) < 1.0:
            raise ValueError("adaptive_placement_start must be in [0, 1)")


@dataclass(frozen=True)
class WalkOutput:
    """Command at one instant.

    Attributes:
        state: Active gait phase.
        support_foot: ``"left"``, ``"right"`` or ``"both"``.
        com_target: Desired centre of mass in the world frame, shape (3,).
            Horizontal coordinates come from the feet. The vertical
            coordinate is the height passed to :meth:`WalkingStateMachine.start`.
        left_foot: Desired left-foot position, shape (3,). Planted during
            every state except ``LEFT_SWING``.
        right_foot: Desired right-foot position, shape (3,). Planted during
            every state except ``RIGHT_SWING``.
    """

    state: WalkState
    support_foot: str
    com_target: np.ndarray
    left_foot: np.ndarray
    right_foot: np.ndarray


@dataclass
class _Phase:
    """One timed residence in a state."""

    state: WalkState
    t_start: float
    duration: float
    support_foot: str
    com_from: np.ndarray
    com_to: np.ndarray
    left_foot: np.ndarray
    right_foot: np.ndarray
    swing_foot: str | None = None
    swing_target: np.ndarray | None = None


class WalkingStateMachine:
    """Schedule weight shifts and swings, and sample the COM target.

    Call :meth:`start` with the two planted feet and the centre-of-mass
    height. :meth:`sample` is then a function of time: it reports the
    state, which foot is in support, and where the centre of mass should be.
    """

    def __init__(self, config: WalkConfig | None = None) -> None:
        self.config = config if config is not None else WalkConfig()
        self._phases: list[_Phase] = []

    def start(
        self,
        left_foot: np.ndarray,
        right_foot: np.ndarray,
        com_height: float,
        n_steps: int,
        t0: float = 0.0,
        first_swing: str = "right",
    ) -> None:
        """Build the gait from the current planted feet.

        The first swing defaults to the right foot, so the first weight
        shift loads the left foot. That is the order in which a foot can
        be lifted without falling.

        Args:
            left_foot: World position of the left foot at ``t0``, shape (3,).
            right_foot: World position of the right foot at ``t0``, shape (3,).
            com_height: Desired centre-of-mass height, in metres. Held
                constant for the whole gait.
            n_steps: Number of swings. Zero stays in ``STAND``.
            t0: Time of the start of ``STAND``, in seconds.
            first_swing: ``"right"`` or ``"left"``.
        """
        if first_swing not in _FEET:
            raise ValueError(f"first_swing must be 'left' or 'right', got {first_swing!r}")
        if int(n_steps) < 0:
            raise ValueError("n_steps must be non-negative")
        left = np.asarray(left_foot, dtype=float).reshape(3).copy()
        right = np.asarray(right_foot, dtype=float).reshape(3).copy()
        self._phases = _build_phases(
            self.config,
            left,
            right,
            float(com_height),
            int(n_steps),
            float(t0),
            first_swing,
        )
        self._arm_online(left, right, float(com_height), int(n_steps), first_swing)

    def _arm_online(
        self,
        left: np.ndarray,
        right: np.ndarray,
        com_height: float,
        n_steps: int,
        first_swing: str,
    ) -> None:
        """Reset the closed-loop machine used by :meth:`update`.

        Unlike :meth:`sample`, a shift does not end when its timer expires
        unless the caller reports that the centre of mass is over the stance
        foot. The swing, and the forward move of the centre of mass onto the
        new foot, wait for that.
        """
        self._left = left.copy()
        self._right = right.copy()
        self._com_height = float(com_height)
        self._n_steps = int(n_steps)
        self._next_swing = first_swing
        self._steps_done = 0
        self._state = WalkState.STAND
        self._t_state = 0.0
        self._terminal = n_steps == 0
        mid = 0.5 * (left[:2] + right[:2])
        self._com_from = mid.copy()
        self._com_to = mid.copy()
        self._swing_target: np.ndarray | None = None
        self._original_swing_target: np.ndarray | None = None
        self._active_swing: str | None = None
        self._swing_elapsed = 0.0
        self._confirm = 0
        self._dwell = 0.0
        self._recover_time = 0.0
        self._adaptive_correction = 0.0
        self._adaptive_requested = 0.0
        self._adaptive_fallback = False
        self._placement_rejected = False
        self._placement_accepted = False
        self._online = True

    @property
    def state(self) -> WalkState:
        """Phase at the last :meth:`update`, or ``STAND`` before the first one."""
        return self._state

    @property
    def active_swing(self) -> str | None:
        """``"left"`` or ``"right"`` while that foot is swinging or being confirmed."""
        return self._active_swing

    @property
    def swing_elapsed(self) -> float:
        """Seconds of the current swing trajectory, including touchdown prep."""
        return float(self._swing_elapsed)

    @property
    def original_swing_target_y(self) -> float:
        """Lateral landing chosen by the footstep rule, before any correction."""
        target = getattr(self, "_original_swing_target", None)
        if target is None:
            return float("nan")
        return float(target[1])

    @property
    def adaptive_target_y(self) -> float:
        """Lateral landing currently commanded. Equal to the original when idle."""
        if self._swing_target is None:
            return float("nan")
        return float(self._swing_target[1])

    @property
    def adaptive_lateral_correction(self) -> float:
        """Lateral correction added to the original landing, in metres."""
        return float(getattr(self, "_adaptive_correction", 0.0))

    @property
    def adaptive_requested_correction(self) -> float:
        """Bounded correction before the reachability fallback, in metres."""
        return float(getattr(self, "_adaptive_requested", 0.0))

    @property
    def adaptive_placement_fallback(self) -> bool:
        """True once this swing's lateral target was rejected as unreachable."""
        return bool(getattr(self, "_adaptive_fallback", False))

    @property
    def touchdown_confirm_count(self) -> int:
        """Consecutive settled-contact samples accumulated in ``TOUCHDOWN``."""
        return int(self._confirm)

    @property
    def loaded_foot(self) -> str | None:
        """Foot the centre of mass must be over before the other foot may lift."""
        landing = self._state in (WalkState.TOUCHDOWN_PREP, WalkState.TOUCHDOWN)
        if self._state in (WalkState.SHIFT_LEFT, WalkState.RIGHT_SWING) or (
            landing and self._active_swing == "right"
        ):
            return "left"
        if self._state in (WalkState.SHIFT_RIGHT, WalkState.LEFT_SWING) or (
            landing and self._active_swing == "left"
        ):
            return "right"
        return None

    def placement_check_due(self, dt: float) -> bool:
        """True on the tick that must test the full lateral landing for reachability.

        The check happens once, at the start of the late-swing window, so a
        rejected target never enters the trajectory and then jumps back.
        """
        if not bool(self.config.adaptive_foot_placement):
            return False
        if self._placement_accepted or self._placement_rejected:
            return False
        if self._state not in (
            WalkState.RIGHT_SWING,
            WalkState.LEFT_SWING,
            WalkState.TOUCHDOWN_PREP,
        ):
            return False
        if self._original_swing_target is None:
            return False
        duration = float(self.config.step_duration)
        if duration <= 0.0:
            return False
        progress = (float(self._swing_elapsed) + float(dt)) / duration
        return progress >= float(self.config.adaptive_placement_start)

    def full_adaptive_target(
        self,
        capture_xy: np.ndarray | None,
        stance_foot_y: float | None = None,
        stance_inner_y: float | None = None,
    ) -> np.ndarray | None:
        """Landing with the full bounded correction, before the smooth ramp.

        ``None`` when this swing is not adjusting its landing. The forward
        and vertical coordinates are the original footstep target.
        """
        if not bool(self.config.adaptive_foot_placement):
            return None
        if self._original_swing_target is None or self._active_swing not in _FEET:
            return None
        correction = self._requested_lateral_correction(capture_xy, stance_foot_y, stance_inner_y)
        target = self._original_swing_target.copy()
        target[1] = float(self._original_swing_target[1]) + correction
        return target

    def update(
        self,
        dt: float,
        support_ready: bool = False,
        *,
        foot_sample_ok: bool = False,
        recovery_sample_ok: bool = False,
        capture_xy: np.ndarray | None = None,
        placement_reachable: bool = True,
        stance_foot_y: float | None = None,
        stance_inner_y: float | None = None,
    ) -> WalkOutput:
        """Advance one control period.

        ``support_ready`` is true when the centre of mass is inside the sole
        of :attr:`loaded_foot`. A shift will not start its swing until both
        its duration has elapsed and ``support_ready`` is true.

        ``foot_sample_ok`` is one settled-contact reading of the swing foot.
        ``TOUCHDOWN`` counts consecutive true readings and ignores the swing
        timer. ``recovery_sample_ok`` is one tick of the recovery exit
        conditions. Neither flag is inferred from time.

        ``capture_xy`` is the existing capture point, ``c + v/omega``. It is
        not recomputed here. When adaptive placement is on, late swing uses
        its lateral component to shift the landing that :func:`_landing`
        already chose. ``placement_reachable`` false rejects that shift and
        keeps the original landing for the rest of the swing.
        """
        if not getattr(self, "_online", False):
            raise RuntimeError("call start() before update()")
        self._t_state += float(dt)
        if self._state in (
            WalkState.RIGHT_SWING,
            WalkState.LEFT_SWING,
            WalkState.TOUCHDOWN_PREP,
        ):
            self._swing_elapsed += float(dt)
        if self._state is WalkState.DOUBLE_SUPPORT_RECOVERY:
            self._recover_time += float(dt)
        if not self._terminal and self._state in (
            WalkState.STAND,
            WalkState.SHIFT_LEFT,
            WalkState.SHIFT_RIGHT,
        ):
            if self._shift_complete(bool(support_ready)):
                self._transition()
        self._apply_adaptive_placement(
            capture_xy,
            bool(placement_reachable),
            stance_foot_y,
            stance_inner_y,
        )
        self._advance_touchdown(float(dt), bool(foot_sample_ok), bool(recovery_sample_ok))
        return self._emit()

    def _requested_lateral_correction(
        self,
        capture_xy: np.ndarray | None,
        stance_foot_y: float | None,
        stance_inner_y: float | None,
    ) -> float:
        """Bounded correction from the original landing. Zero without a capture point."""
        if capture_xy is None or self._original_swing_target is None:
            return 0.0
        if not bool(self.config.adaptive_foot_placement):
            return 0.0
        capture = np.asarray(capture_xy, dtype=float).reshape(-1)
        if capture.size == 0:
            return 0.0
        capture_y = float(capture[1]) if capture.size >= 2 else float(capture[0])
        if stance_foot_y is None:
            if self._active_swing == "right":
                stance_y = float(self._left[1])
            elif self._active_swing == "left":
                stance_y = float(self._right[1])
            else:
                return 0.0
        else:
            stance_y = float(stance_foot_y)
        return lateral_landing_correction(
            float(self._original_swing_target[1]),
            stance_y,
            capture_y,
            gain=float(self.config.capture_footstep_gain),
            max_adjustment=float(self.config.max_lateral_step_adjustment),
            half_width=SOLE_HALF_WIDTH,
            stance_inner_y=stance_inner_y,
        )

    def _apply_adaptive_placement(
        self,
        capture_xy: np.ndarray | None,
        reachable: bool,
        stance_foot_y: float | None,
        stance_inner_y: float | None,
    ) -> None:
        """Ramp the existing landing's lateral coordinate during late swing.

        The forward coordinate stays on the footstep target. An unreachable
        full correction latches the original landing for the rest of this swing.
        """
        if self._original_swing_target is None or self._swing_target is None:
            self._adaptive_correction = 0.0
            self._adaptive_requested = 0.0
            return
        if self._state not in (
            WalkState.RIGHT_SWING,
            WalkState.LEFT_SWING,
            WalkState.TOUCHDOWN_PREP,
        ):
            return
        if not bool(self.config.adaptive_foot_placement):
            self._swing_target = self._original_swing_target.copy()
            self._adaptive_correction = 0.0
            self._adaptive_requested = 0.0
            self._adaptive_fallback = False
            return
        requested = self._requested_lateral_correction(capture_xy, stance_foot_y, stance_inner_y)
        self._adaptive_requested = float(requested)
        duration = float(self.config.step_duration)
        progress = 0.0 if duration <= 0.0 else float(np.clip(self._swing_elapsed / duration, 0.0, 1.0))
        start = float(self.config.adaptive_placement_start)
        if self._placement_rejected:
            self._swing_target = self._original_swing_target.copy()
            self._adaptive_correction = 0.0
            self._adaptive_fallback = True
            return
        if progress < start:
            self._swing_target = self._original_swing_target.copy()
            self._adaptive_correction = 0.0
            self._adaptive_fallback = False
            return
        if not self._placement_accepted:
            if not reachable and abs(requested) > 1e-6:
                self._placement_rejected = True
                self._adaptive_fallback = True
                self._swing_target = self._original_swing_target.copy()
                self._adaptive_correction = 0.0
                return
            self._placement_accepted = True
        applied = placement_ramp(progress, start) * float(requested)
        target = self._original_swing_target.copy()
        target[1] = float(self._original_swing_target[1]) + applied
        self._swing_target = target
        self._adaptive_correction = float(applied)
        self._adaptive_fallback = False

    def _advance_touchdown(self, dt: float, foot_ok: bool, recovery_ok: bool) -> None:
        """Move through prep, contact confirmation, and recovery.

        The phase at the start of the call is the only one that can advance.
        A tick that first reaches the landing target does not also consume a
        contact sample.
        """
        cfg = self.config
        started = self._state
        if started in (WalkState.RIGHT_SWING, WalkState.LEFT_SWING):
            prep = max(0.0, float(cfg.touchdown_prep_duration))
            if self._swing_elapsed >= float(cfg.step_duration) - prep:
                if prep <= 0.0 or self._swing_elapsed >= float(cfg.step_duration):
                    self._commit_landing()
                    self._enter(WalkState.TOUCHDOWN)
                else:
                    self._enter(WalkState.TOUCHDOWN_PREP)
            return
        if started is WalkState.TOUCHDOWN_PREP:
            if self._swing_elapsed >= float(cfg.step_duration):
                self._commit_landing()
                self._enter(WalkState.TOUCHDOWN)
            return
        if started is WalkState.TOUCHDOWN:
            if foot_ok:
                self._confirm += 1
            else:
                self._confirm = 0
            if self._confirm >= int(cfg.touchdown_confirm_ticks):
                self._count_step()
                self._dwell = 0.0
                self._recover_time = 0.0
                self._enter(WalkState.DOUBLE_SUPPORT_RECOVERY)
            return
        if started is WalkState.DOUBLE_SUPPORT_RECOVERY:
            if recovery_ok:
                self._dwell += dt
            else:
                self._dwell = 0.0
            dwell_ok = self._dwell >= float(cfg.stabilization_dwell)
            held = self._recover_time >= float(cfg.recovery_duration)
            if dwell_ok and held:
                self._finish_recovery()

    def _enter(self, state: WalkState) -> None:
        self._com_from = self._com_to.copy()
        self._t_state = 0.0
        if state is WalkState.TOUCHDOWN:
            self._confirm = 0
        self._state = state

    def _commit_landing(self) -> None:
        """Hold the swing foot on the trajectory's landing target."""
        if self._swing_target is None or self._active_swing not in _FEET:
            return
        if self._active_swing == "right":
            self._right = self._swing_target.copy()
        else:
            self._left = self._swing_target.copy()

    def _count_step(self) -> None:
        if self._active_swing == "right":
            self._steps_done += 1
            self._next_swing = "left"
        elif self._active_swing == "left":
            self._steps_done += 1
            self._next_swing = "right"

    def _finish_recovery(self) -> None:
        """Leave recovery for the final stand, or the next weight shift.

        The caller has already required the dwell. This method does not
        look at ``double_support_duration``.
        """
        nxt = next_state(
            WalkState.DOUBLE_SUPPORT,
            steps_done=self._steps_done,
            n_steps=self._n_steps,
            next_swing=self._next_swing,
        )
        self._swing_target = None
        self._original_swing_target = None
        self._active_swing = None
        self._swing_elapsed = 0.0
        self._t_state = 0.0
        self._dwell = 0.0
        self._adaptive_correction = 0.0
        self._adaptive_requested = 0.0
        self._adaptive_fallback = False
        self._placement_rejected = False
        self._placement_accepted = False
        if nxt is WalkState.STAND:
            self._state = WalkState.STAND
            self._terminal = self._steps_done >= self._n_steps
            self._com_to = 0.5 * (self._left[:2] + self._right[:2])
            self._com_from = self._com_to.copy()
            return
        self._state = nxt
        self._arm_state()

    def _shift_complete(self, support_ready: bool) -> bool:
        duration = self._state_duration(self._state)
        if not np.isfinite(duration) or self._t_state < duration:
            return False
        if self._state in (WalkState.SHIFT_LEFT, WalkState.SHIFT_RIGHT):
            return support_ready
        return True

    def _state_duration(self, state: WalkState) -> float:
        cfg = self.config
        if state is WalkState.STAND:
            return float(cfg.stand_duration) if not self._terminal else np.inf
        if state in (WalkState.SHIFT_LEFT, WalkState.SHIFT_RIGHT):
            return float(cfg.shift_duration)
        if state in (WalkState.RIGHT_SWING, WalkState.LEFT_SWING):
            return float(cfg.step_duration)
        if state is WalkState.DOUBLE_SUPPORT:
            return float(cfg.double_support_duration)
        return np.inf

    def _transition(self) -> None:
        if self._state is WalkState.RIGHT_SWING and self._swing_target is not None:
            self._right = self._swing_target.copy()
            self._steps_done += 1
            self._next_swing = "left"
        elif self._state is WalkState.LEFT_SWING and self._swing_target is not None:
            self._left = self._swing_target.copy()
            self._steps_done += 1
            self._next_swing = "right"
        nxt = next_state(
            self._state,
            steps_done=self._steps_done,
            n_steps=self._n_steps,
            next_swing=self._next_swing,
        )
        # The blend has reached its target by the time a timed state ends.
        self._com_from = self._com_to.copy()
        self._t_state = 0.0
        self._swing_target = None
        self._original_swing_target = None
        self._adaptive_correction = 0.0
        self._adaptive_requested = 0.0
        self._adaptive_fallback = False
        self._placement_rejected = False
        self._placement_accepted = False
        if nxt is None:
            self._state = WalkState.STAND
            self._terminal = True
            self._com_to = 0.5 * (self._left[:2] + self._right[:2])
            return
        self._state = nxt
        self._arm_state()

    def _remember_original_landing(self) -> None:
        """Keep the footstep landing so a later lateral ramp cannot compound."""
        if self._swing_target is None:
            self._original_swing_target = None
        else:
            self._original_swing_target = self._swing_target.copy()
        self._adaptive_correction = 0.0
        self._adaptive_requested = 0.0
        self._adaptive_fallback = False
        self._placement_rejected = False
        self._placement_accepted = False

    def _arm_state(self) -> None:
        if self._state is WalkState.SHIFT_LEFT:
            self._com_to = self._left[:2].copy()
        elif self._state is WalkState.SHIFT_RIGHT:
            self._com_to = self._right[:2].copy()
        elif self._state is WalkState.RIGHT_SWING:
            self._com_to = self._left[:2].copy()
            self._active_swing = "right"
            self._swing_elapsed = 0.0
            self._swing_target = _landing(self._right, self._left, self.config)
            self._remember_original_landing()
        elif self._state is WalkState.LEFT_SWING:
            self._com_to = self._right[:2].copy()
            self._active_swing = "left"
            self._swing_elapsed = 0.0
            self._swing_target = _landing(self._left, self._right, self.config)
            self._remember_original_landing()
        elif self._state is WalkState.STAND:
            self._com_to = 0.5 * (self._left[:2] + self._right[:2])
        # DOUBLE_SUPPORT keeps the centre of mass on the foot that just carried
        # the step, which is already ``_com_to``.

    def _emit(self) -> WalkOutput:
        duration = self._state_duration(self._state)
        if not np.isfinite(duration) or duration <= 0.0:
            fraction = 1.0
        else:
            fraction = float(np.clip(self._t_state / duration, 0.0, 1.0))
        com_xy = _blend(self._com_from, self._com_to, fraction)
        left = self._left.copy()
        right = self._right.copy()
        if (
            self._state in (WalkState.RIGHT_SWING, WalkState.LEFT_SWING, WalkState.TOUCHDOWN_PREP)
            and self._swing_target is not None
            and self._active_swing in _FEET
        ):
            swing_foot = self._active_swing
            liftoff = right if swing_foot == "right" else left
            trajectory = SwingFootTrajectory(
                liftoff, self._swing_target, self.config.step_duration, self.config.step_height
            )
            position, _velocity = trajectory.evaluate(
                min(self._swing_elapsed, self.config.step_duration)
            )
            if swing_foot == "right":
                right = position
            else:
                left = position
        return WalkOutput(
            state=self._state,
            support_foot=self._support_name(),
            com_target=_com(com_xy, self._com_height),
            left_foot=left,
            right_foot=right,
        )

    def _support_name(self) -> str:
        """Stance foot during a swing and while that landing is unconfirmed."""
        if self._state in (
            WalkState.RIGHT_SWING,
            WalkState.LEFT_SWING,
            WalkState.TOUCHDOWN_PREP,
            WalkState.TOUCHDOWN,
        ):
            if self._active_swing == "right":
                return "left"
            if self._active_swing == "left":
                return "right"
        return _SUPPORT.get(self._state, "both")

    def sample(self, time_s: float) -> WalkOutput:
        """State, support foot and centre-of-mass target at ``time_s``.

        Times before the start of the gait hold the initial stand. Times
        after the last step hold the final stand, with the centre of mass
        halfway between the two landed feet.

        Raises:
            RuntimeError: if :meth:`start` has not been called.
        """
        if not self._phases:
            raise RuntimeError("call start() before sample()")
        phase = _phase_at(self._phases, float(time_s))
        com_xy = _blend(phase.com_from, phase.com_to, _phase_fraction(phase, float(time_s)))
        com = np.array([float(com_xy[0]), float(com_xy[1]), float(phase.com_from[2])])
        left = phase.left_foot.copy()
        right = phase.right_foot.copy()
        if phase.swing_foot is not None and phase.swing_target is not None:
            liftoff = left if phase.swing_foot == "left" else right
            swing = SwingFootTrajectory(
                liftoff,
                phase.swing_target,
                phase.duration,
                self.config.step_height,
            )
            position, _velocity = swing.evaluate(float(time_s) - phase.t_start)
            if phase.swing_foot == "left":
                left = position
            else:
                right = position
        return WalkOutput(
            state=phase.state,
            support_foot=phase.support_foot,
            com_target=com,
            left_foot=left,
            right_foot=right,
        )


def touchdown_sample_ok(
    *,
    in_contact: bool,
    foot_z: float,
    foot_vz: float,
    height_limit: float,
    vz_min: float,
) -> bool:
    """One settled-contact sample. Contact, height, and vertical speed are all required."""
    return bool(
        in_contact
        and float(foot_z) <= float(height_limit)
        and float(foot_vz) >= float(vz_min)
    )


def recovery_conditions_met(
    *,
    both_contact: bool,
    com_xy: np.ndarray,
    capture_xy: np.ndarray,
    box_low: np.ndarray,
    box_high: np.ndarray,
    vx: float,
    vy: float,
    roll: float,
    pitch: float,
    margin: float,
    max_vx: float,
    max_vy: float,
    max_roll: float,
    max_pitch: float,
) -> bool:
    """True when this tick is allowed to contribute to the recovery dwell."""
    if not both_contact:
        return False
    low = np.asarray(box_low, dtype=float).reshape(2) + float(margin)
    high = np.asarray(box_high, dtype=float).reshape(2) - float(margin)
    if np.any(high <= low):
        return False
    com = np.asarray(com_xy, dtype=float).reshape(2)
    capture = np.asarray(capture_xy, dtype=float).reshape(2)
    inside_com = bool(np.all(com >= low) and np.all(com <= high))
    inside_capture = bool(np.all(capture >= low) and np.all(capture <= high))
    return bool(
        inside_com
        and inside_capture
        and abs(float(vx)) <= float(max_vx)
        and abs(float(vy)) <= float(max_vy)
        and abs(float(roll)) <= float(max_roll)
        and abs(float(pitch)) <= float(max_pitch)
    )


def contact_qualified_corners(
    soles: dict[str, np.ndarray],
    contacts: dict[str, bool],
) -> np.ndarray:
    """Stack the sole corners whose foot is in contact. Empty when none are."""
    chunks = []
    for side in _FEET:
        if not contacts.get(side):
            continue
        if side not in soles:
            continue
        chunks.append(np.asarray(soles[side], dtype=float).reshape(-1, 2))
    if not chunks:
        return np.zeros((0, 2))
    return np.vstack(chunks)


def support_box_center(vertices: np.ndarray, margin: float) -> np.ndarray:
    """Centre of the box inset by ``margin``. The raw centre if the inset is empty."""
    points = np.asarray(vertices, dtype=float).reshape(-1, 2)
    low = points.min(axis=0)
    high = points.max(axis=0)
    inset_low = low + float(margin)
    inset_high = high - float(margin)
    if np.any(inset_high <= inset_low):
        return 0.5 * (low + high)
    return 0.5 * (inset_low + inset_high)


def next_state(
    state: WalkState,
    *,
    steps_done: int,
    n_steps: int,
    next_swing: str,
) -> WalkState | None:
    """Successor of ``state``, or ``None`` to stay in the terminal stand.

    The support-foot rule is structural: ``RIGHT_SWING`` is reached only
    from ``SHIFT_LEFT``, and ``LEFT_SWING`` only from ``SHIFT_RIGHT``.
    ``DOUBLE_SUPPORT`` returns to a shift while swings remain, and to
    ``STAND`` once they do not.
    """
    if next_swing not in _FEET:
        raise ValueError(f"next_swing must be 'left' or 'right', got {next_swing!r}")
    if state is WalkState.STAND:
        if steps_done >= n_steps:
            return None
        return WalkState.SHIFT_LEFT if next_swing == "right" else WalkState.SHIFT_RIGHT
    if state is WalkState.SHIFT_LEFT:
        return WalkState.RIGHT_SWING
    if state is WalkState.SHIFT_RIGHT:
        return WalkState.LEFT_SWING
    if state is WalkState.RIGHT_SWING or state is WalkState.LEFT_SWING:
        return WalkState.DOUBLE_SUPPORT
    if state is WalkState.DOUBLE_SUPPORT:
        if steps_done >= n_steps:
            return WalkState.STAND
        return WalkState.SHIFT_LEFT if next_swing == "right" else WalkState.SHIFT_RIGHT
    raise ValueError(f"unknown state {state!r}")


def _build_phases(
    config: WalkConfig,
    left: np.ndarray,
    right: np.ndarray,
    com_height: float,
    n_steps: int,
    t0: float,
    first_swing: str,
) -> list[_Phase]:
    """Expand the transition rules into a timed list of phases."""
    phases: list[_Phase] = []
    com_xy = 0.5 * (left[:2] + right[:2])
    clock = float(t0)
    steps_done = 0
    next_swing = first_swing
    state: WalkState | None = WalkState.STAND

    def emit(
        phase_state: WalkState,
        duration: float,
        com_to_xy: np.ndarray,
        swing_foot: str | None = None,
        swing_target: np.ndarray | None = None,
    ) -> None:
        nonlocal clock, com_xy
        if duration <= 0.0:
            com_xy = np.asarray(com_to_xy, dtype=float).reshape(2).copy()
            return
        support = _SUPPORT[phase_state]
        com_from = _com(com_xy, com_height)
        com_to = _com(com_to_xy, com_height)
        phases.append(
            _Phase(
                state=phase_state,
                t_start=clock,
                duration=float(duration),
                support_foot=support,
                com_from=com_from,
                com_to=com_to,
                left_foot=left.copy(),
                right_foot=right.copy(),
                swing_foot=swing_foot,
                swing_target=None if swing_target is None else swing_target.copy(),
            )
        )
        clock += float(duration)
        com_xy = com_to[:2].copy()

    while state is not None:
        # A swing is illegal unless the shift that loaded its stance foot
        # has already been emitted. next_state is the only place that
        # chooses a swing, and it only does so from the matching shift.
        if state is WalkState.STAND and steps_done >= n_steps and phases:
            midpoint = 0.5 * (left[:2] + right[:2])
            emit(WalkState.STAND, config.stand_duration, midpoint)
            emit(WalkState.STAND, np.inf, midpoint)
            break
        if state is WalkState.STAND and steps_done >= n_steps:
            emit(WalkState.STAND, np.inf, com_xy)
            break

        if state is WalkState.SHIFT_LEFT:
            emit(state, config.shift_duration, left[:2])
        elif state is WalkState.SHIFT_RIGHT:
            emit(state, config.shift_duration, right[:2])
        elif state is WalkState.RIGHT_SWING:
            _require_loaded(phases, WalkState.SHIFT_LEFT, "right")
            target = _landing(right, left, config)
            emit(state, config.step_duration, left[:2], swing_foot="right", swing_target=target)
            right = target
            steps_done += 1
            next_swing = "left"
        elif state is WalkState.LEFT_SWING:
            _require_loaded(phases, WalkState.SHIFT_RIGHT, "left")
            target = _landing(left, right, config)
            emit(state, config.step_duration, right[:2], swing_foot="left", swing_target=target)
            left = target
            steps_done += 1
            next_swing = "right"
        elif state is WalkState.DOUBLE_SUPPORT:
            emit(state, config.double_support_duration, com_xy)
        elif state is WalkState.STAND:
            emit(state, config.stand_duration, com_xy)
        else:
            raise RuntimeError(f"unhandled state {state!r}")

        state = next_state(
            state,
            steps_done=steps_done,
            n_steps=n_steps,
            next_swing=next_swing,
        )
    return phases


def _require_loaded(phases: list[_Phase], shift: WalkState, swing_foot: str) -> None:
    """A foot may leave the ground only after the shift onto the other foot."""
    if not phases or phases[-1].state is not shift or phases[-1].support_foot == "both":
        raise RuntimeError(
            f"refusing to lift the {swing_foot} foot before the centre of mass is over the support foot"
        )


def _landing(swing_foot: np.ndarray, stance_foot: np.ndarray, config: WalkConfig) -> np.ndarray:
    """Land ``step_length`` ahead of the stance foot, keeping the swing foot's spacing."""
    forward = config.forward
    stance_ahead = float(np.dot(stance_foot, forward))
    swing_ahead = float(np.dot(swing_foot, forward))
    return swing_foot + (stance_ahead + float(config.step_length) - swing_ahead) * forward


def _com(xy: np.ndarray, height: float) -> np.ndarray:
    horizontal = np.asarray(xy, dtype=float).reshape(2)
    return np.array([horizontal[0], horizontal[1], float(height)])


def _phase_at(phases: list[_Phase], time_s: float) -> _Phase:
    for phase in phases:
        if time_s < phase.t_start + phase.duration:
            return phase
    return phases[-1]


def _phase_fraction(phase: _Phase, time_s: float) -> float:
    if not np.isfinite(phase.duration):
        return 1.0
    if phase.duration <= 0.0:
        return 1.0
    return float(np.clip((time_s - phase.t_start) / phase.duration, 0.0, 1.0))


def _blend(start: np.ndarray, end: np.ndarray, fraction: float) -> np.ndarray:
    """Cubic blend with zero slope at both ends: ``3 s^2 - 2 s^3``."""
    s = float(np.clip(fraction, 0.0, 1.0))
    sigma = s * s * (3.0 - 2.0 * s)
    return (1.0 - sigma) * start[:2] + sigma * end[:2]
