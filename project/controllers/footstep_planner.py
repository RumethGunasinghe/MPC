"""Alternating footstep planner for a double-support humanoid.

This is a research prototype. It does not check reachability, friction or
balance. It only produces the foot trajectories a downstream controller can
track.

Schedule
--------
Steps alternate left, right, left, right, ... One step lasts ``step_duration``
seconds and the next step starts as soon as the previous foot lands, so one
foot is in stance for the whole swing of the other.

Step length is the sagittal distance between successive opposite footfalls.
If the stance foot is at forward coordinate ``p`` the swing foot lands at

    p_land = p + step_length

along the forward axis (world +x by default). The same foot's next landing is
therefore ``2 * step_length`` ahead of its previous one: that distance is the
stride, two steps.

Swing path
----------
:class:`SwingFootTrajectory` splits one step into three equal phases — lift,
move forward, land — and fits a cubic on each phase. Adjacent cubics share
position and velocity, so the result is a C1 cubic spline. Velocity is zero
at liftoff, at both phase junctions and at touchdown, which keeps the foot
from sliding as it leaves or strikes the ground and caps the height at
``step_height`` (a single spline through the same knots would overshoot).
Outside a foot's own swing the desired position is held and the velocity is
zero.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

_FEET = ("left", "right")


@dataclass
class FootstepConfig:
    """Timing and size of one swing.

    Attributes:
        step_length: Forward distance from the stance foot to the next
            opposite landing, in metres.
        step_height: Clearance of the swing above the liftoff and landing
            positions, in metres. The foot reaches this height at the end of
            the lift and holds it through the forward phase.
        step_duration: Duration of one swing, in seconds. The other foot
            starts its swing when this one ends.
        forward: Unit direction of travel in the same frame as the foot
            positions. The default is world +x.
    """

    step_length: float = 0.1
    step_height: float = 0.05
    step_duration: float = 0.8
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


@dataclass
class Footstep:
    """One scheduled swing, from liftoff to the landing target."""

    foot: str
    start: np.ndarray
    target: np.ndarray
    t_start: float
    duration: float

    def __post_init__(self) -> None:
        if self.foot not in _FEET:
            raise ValueError(f"foot must be 'left' or 'right', got {self.foot!r}")
        self.start = np.asarray(self.start, dtype=float).reshape(3)
        self.target = np.asarray(self.target, dtype=float).reshape(3)
        self.t_start = float(self.t_start)
        self.duration = float(self.duration)


class FootstepPlanner:
    """Generate alternating left and right foot targets and sample them in time.

    Call :meth:`plan` once with the two foot positions at the start of the
    gait. :meth:`foot_positions` then returns the desired position of each
    foot at any later time: the swing foot follows the arc above, and the
    stance foot stays on its last target.
    """

    def __init__(self, config: FootstepConfig | None = None) -> None:
        self.config = config if config is not None else FootstepConfig()
        self.steps: list[Footstep] = []
        self._planted = {"left": np.zeros(3), "right": np.zeros(3)}
        self._planned = False

    def plan(
        self,
        left_position: np.ndarray,
        right_position: np.ndarray,
        n_steps: int,
        t0: float = 0.0,
        first_foot: str = "left",
    ) -> list[Footstep]:
        """Build ``n_steps`` alternating landings and store the schedule.

        The first swing is ``first_foot``. Each landing is ``step_length``
        ahead of whichever foot is in stance at liftoff. Lateral position and
        ground height are copied from that swing foot's liftoff pose, so the
        feet keep their spacing and walk on level ground.

        Args:
            left_position: World position of the left foot at ``t0``, shape (3,).
            right_position: World position of the right foot at ``t0``, shape (3,).
            n_steps: Number of swings. Zero leaves both feet planted.
            t0: Time of the first liftoff, in seconds.
            first_foot: ``"left"`` or ``"right"``.

        Returns:
            The scheduled steps, also available as ``self.steps``.
        """
        if first_foot not in _FEET:
            raise ValueError(f"first_foot must be 'left' or 'right', got {first_foot!r}")
        if int(n_steps) < 0:
            raise ValueError("n_steps must be non-negative")

        planted = {
            "left": np.asarray(left_position, dtype=float).reshape(3).copy(),
            "right": np.asarray(right_position, dtype=float).reshape(3).copy(),
        }
        forward = self.config.forward
        duration = float(self.config.step_duration)
        length = float(self.config.step_length)
        foot = first_foot
        t_start = float(t0)
        steps: list[Footstep] = []

        for _ in range(int(n_steps)):
            other = "right" if foot == "left" else "left"
            start = planted[foot].copy()
            # Landing forward coordinate = stance forward coordinate + step length.
            # Components orthogonal to ``forward`` stay at the swing foot's
            # liftoff values, which keeps the left/right spacing.
            stance_ahead = float(np.dot(planted[other], forward))
            swing_ahead = float(np.dot(start, forward))
            target = start + (stance_ahead + length - swing_ahead) * forward
            steps.append(
                Footstep(
                    foot=foot,
                    start=start,
                    target=target,
                    t_start=t_start,
                    duration=duration,
                )
            )
            planted[foot] = target
            foot = other
            t_start += duration

        self._planted = planted
        self.steps = steps
        self._planned = True
        return steps

    def foot_positions(self, time_s: float | np.ndarray) -> dict[str, np.ndarray]:
        """Desired foot positions at one time or along a vector of times.

        Args:
            time_s: A scalar second, or an array of seconds of shape (T,).

        Returns:
            ``{"left": positions, "right": positions}``. A scalar time gives
            shape (3,). An array of times gives shape (T, 3) for each foot.
            Before the first liftoff both feet hold the positions passed to
            :meth:`plan`. After the last touchdown they hold the last targets.

        Raises:
            RuntimeError: if :meth:`plan` has not been called.
        """
        if not self._planned:
            raise RuntimeError("call plan() before foot_positions()")

        times = np.asarray(time_s, dtype=float)
        scalar = times.ndim == 0
        times = np.atleast_1d(times)
        left = np.zeros((times.size, 3))
        right = np.zeros((times.size, 3))
        for i, t in enumerate(times):
            sample = self._position_at(float(t))
            left[i] = sample["left"]
            right[i] = sample["right"]
        if scalar:
            return {"left": left[0], "right": right[0]}
        return {"left": left, "right": right}

    def _position_at(self, time_s: float) -> dict[str, np.ndarray]:
        """Both feet at a single instant, using the final planted pose as the base."""
        # Walk the schedule backwards from the final pose: a step that has not
        # finished yet still owns the swing foot, and a step that has not
        # started yet means that foot is still at its liftoff point.
        left = self._planted["left"].copy()
        right = self._planted["right"].copy()
        pose = {"left": left, "right": right}
        for step in reversed(self.steps):
            if time_s >= step.t_start + step.duration:
                continue
            if time_s <= step.t_start:
                pose[step.foot] = step.start.copy()
                continue
            swing = SwingFootTrajectory(
                step.start, step.target, step.duration, self.config.step_height
            )
            pose[step.foot] = swing.evaluate(time_s - step.t_start)[0]
        return pose


class SwingFootTrajectory:
    """Cubic-spline swing from a start position to an end position.

    The step duration is split into three equal phases:

    1. **Lift.** The foot rises ``step_height`` above the start position.
       Horizontal coordinates stay put.
    2. **Move forward.** The foot travels from the lifted start to
       ``step_height`` above the end position.
    3. **Land.** The foot descends onto the end position. Horizontal
       coordinates stay at the landing target.

    Each phase is a cubic Hermite segment with zero velocity at both of its
    endpoints. On a phase running from ``p0`` to ``p1`` over ``dt = T/3``,

        s = (t - t_phase) / dt
        sigma(s) = 3 s^2 - 2 s^3
        p(t) = p0 + sigma(s) (p1 - p0)
        v(t) = (6 s - 6 s^2) (p1 - p0) / dt

    ``sigma`` is a cubic, ``sigma(0) = 0``, ``sigma(1) = 1`` and
    ``sigma'(0) = sigma'(1) = 0``. Because every junction velocity is zero,
    position and velocity are continuous across the whole step, and the
    height never exceeds ``step_height`` above the higher of the two
    endpoints.
    """

    def __init__(
        self,
        start_position: np.ndarray,
        end_position: np.ndarray,
        step_duration: float,
        step_height: float = 0.05,
    ) -> None:
        self.start = np.asarray(start_position, dtype=float).reshape(3).copy()
        self.end = np.asarray(end_position, dtype=float).reshape(3).copy()
        self.duration = float(step_duration)
        self.step_height = float(step_height)
        if self.duration <= 0.0:
            raise ValueError("step_duration must be positive")
        if self.step_height < 0.0:
            raise ValueError("step_height must be non-negative")

        lifted_start = self.start.copy()
        lifted_start[2] += self.step_height
        lifted_end = self.end.copy()
        lifted_end[2] += self.step_height
        # Knots: liftoff, top of lift, start of descent, touchdown.
        self._knots = (self.start, lifted_start, lifted_end, self.end)
        self._phase_duration = self.duration / 3.0

    def evaluate(self, time_s: float | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Position and velocity at one time or along a vector of times.

        ``time_s`` is seconds since liftoff, not a global clock. A scalar
        returns two vectors of shape (3,). An array of shape (T,) returns
        two arrays of shape (T, 3). Times before liftoff hold the start
        position; times after touchdown hold the end position. Velocity is
        zero outside the swing.
        """
        times = np.asarray(time_s, dtype=float)
        scalar = times.ndim == 0
        times = np.atleast_1d(times)
        position = np.zeros((times.size, 3))
        velocity = np.zeros((times.size, 3))
        for i, t in enumerate(times):
            position[i], velocity[i] = self._evaluate_scalar(float(t))
        if scalar:
            return position[0], velocity[0]
        return position, velocity

    def _evaluate_scalar(self, time_s: float) -> tuple[np.ndarray, np.ndarray]:
        if time_s <= 0.0:
            return self.start.copy(), np.zeros(3)
        if time_s >= self.duration:
            return self.end.copy(), np.zeros(3)
        phase = min(int(time_s / self._phase_duration), 2)
        phase_time = time_s - phase * self._phase_duration
        return _cubic_segment(
            self._knots[phase],
            self._knots[phase + 1],
            self._phase_duration,
            phase_time,
        )


def _cubic_segment(
    start: np.ndarray, end: np.ndarray, duration: float, time_s: float
) -> tuple[np.ndarray, np.ndarray]:
    """Cubic from ``start`` to ``end`` with zero velocity at both ends.

    ``time_s`` is seconds from the beginning of this phase, in
    ``[0, duration]``.
    """
    s = float(np.clip(time_s / duration, 0.0, 1.0))
    # sigma(s) = 3 s^2 - 2 s^3,  sigma'(s) = 6 s - 6 s^2.
    sigma = s * s * (3.0 - 2.0 * s)
    dsigma = 6.0 * s * (1.0 - s) / duration
    delta = end - start
    return start + sigma * delta, dsigma * delta
