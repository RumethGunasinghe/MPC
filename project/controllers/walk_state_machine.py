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
    lift / move-forward / land trajectory. The centre of mass stays over
    the support foot.
``DOUBLE_SUPPORT``
    The swing has landed. Both feet are planted and the centre of mass
    stays on the foot that just carried the step, until the next shift
    moves it across.

Transitions fire when the current state has used up its duration and the
support foot is the one that state requires. A swing in particular is
entered only from the shift that loaded the opposite foot.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from controllers.footstep_planner import SwingFootTrajectory

_FEET = ("left", "right")


class WalkState(Enum):
    """Discrete phase of the gait."""

    STAND = "STAND"
    SHIFT_LEFT = "SHIFT_LEFT"
    RIGHT_SWING = "RIGHT_SWING"
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
        double_support_duration: Seconds after a landing with both feet
            down before the next weight shift.
        stand_duration: Seconds in ``STAND`` before the first shift, and
            again at the end while the centre of mass returns to the
            middle of the two feet.
        forward: Unit direction of travel. The default is world +x.
    """

    step_length: float = 0.1
    step_height: float = 0.05
    step_duration: float = 0.8
    shift_duration: float = 0.8
    double_support_duration: float = 0.4
    stand_duration: float = 0.5
    forward: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))

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
        self._online = True

    @property
    def loaded_foot(self) -> str | None:
        """Foot the centre of mass must be over before the other foot may lift."""
        if self._state in (WalkState.SHIFT_LEFT, WalkState.RIGHT_SWING):
            return "left"
        if self._state in (WalkState.SHIFT_RIGHT, WalkState.LEFT_SWING):
            return "right"
        return None

    def update(self, dt: float, support_ready: bool) -> WalkOutput:
        """Advance one control period.

        ``support_ready`` is true when the centre of mass is inside the sole
        of :attr:`loaded_foot`. A shift will not start its swing until both
        its duration has elapsed and ``support_ready`` is true.
        """
        if not getattr(self, "_online", False):
            raise RuntimeError("call start() before update()")
        self._t_state += float(dt)
        if not self._terminal and self._shift_complete(bool(support_ready)):
            self._transition()
        return self._emit()

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
        if nxt is None:
            self._state = WalkState.STAND
            self._terminal = True
            self._com_to = 0.5 * (self._left[:2] + self._right[:2])
            return
        self._state = nxt
        self._arm_state()

    def _arm_state(self) -> None:
        if self._state is WalkState.SHIFT_LEFT:
            self._com_to = self._left[:2].copy()
        elif self._state is WalkState.SHIFT_RIGHT:
            self._com_to = self._right[:2].copy()
        elif self._state is WalkState.RIGHT_SWING:
            self._com_to = self._left[:2].copy()
            self._swing_target = _landing(self._right, self._left, self.config)
        elif self._state is WalkState.LEFT_SWING:
            self._com_to = self._right[:2].copy()
            self._swing_target = _landing(self._left, self._right, self.config)
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
            self._state in (WalkState.RIGHT_SWING, WalkState.LEFT_SWING)
            and self._swing_target is not None
        ):
            swing_foot = "right" if self._state is WalkState.RIGHT_SWING else "left"
            liftoff = right if swing_foot == "right" else left
            trajectory = SwingFootTrajectory(
                liftoff, self._swing_target, self.config.step_duration, self.config.step_height
            )
            position, _velocity = trajectory.evaluate(min(self._t_state, self.config.step_duration))
            if swing_foot == "right":
                right = position
            else:
                left = position
        return WalkOutput(
            state=self._state,
            support_foot=_SUPPORT[self._state],
            com_target=_com(com_xy, self._com_height),
            left_foot=left,
            right_foot=right,
        )

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
