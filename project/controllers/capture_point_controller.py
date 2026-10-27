"""LIPM capture-point preparation for one step.

The swing foot already has a landing. This module decides where the capture
point should sit on that landing sole, and the centre-of-mass velocity that
would put it there:

    capture = com + com_velocity / omega
    omega = sqrt(g / z)
    desired_com_velocity = omega * (desired_capture - com)

The velocity is clipped and acceleration-limited, then blended in over the
swing. It is a reference for the existing whole-body MPC, not a joint torque
and not a second balance controller. The centre of mass is not moved onto
the foot; only the velocity that points the capture point at the sole is
commanded.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

GRAVITY = 9.81

# Ankle-frame sole. Matches the rectangle used by the walking controller.
SOLE_HEEL = -0.035
SOLE_TOE = 0.140
SOLE_HALF_WIDTH = 0.020


@dataclass
class CaptureCommand:
    """One tick of capture-point preparation."""

    active: bool
    blend: float
    current_capture: np.ndarray
    desired_capture: np.ndarray
    capture_error: np.ndarray
    desired_com_velocity: np.ndarray
    sole_low: np.ndarray
    sole_high: np.ndarray


class CapturePreparation:
    """Desired capture point and the bounded CoM velocity that reaches it.

    ``max_lateral_velocity`` is the lateral speed cap. ``max_forward_velocity``
    is the existing walking speed. ``max_acceleration`` keeps the command
    from stepping when the blend or the target changes. Preparation runs from
    swing progress ``start`` to ``end`` with a smoothstep, then holds.
    """

    def __init__(
        self,
        max_lateral_velocity: float = 0.20,
        max_forward_velocity: float = 0.05,
        max_acceleration: float = 1.0,
        margin: float = 0.015,
        start: float = 0.0,
        end: float = 0.70,
    ) -> None:
        self.max_lateral_velocity = float(max_lateral_velocity)
        self.max_forward_velocity = float(max_forward_velocity)
        self.max_acceleration = float(max_acceleration)
        self.margin = float(margin)
        self.start = float(start)
        self.end = float(end)
        self._velocity = np.zeros(2)

    def reset(self) -> None:
        """Drop the rate limiter. The next swing starts from rest."""
        self._velocity[:] = 0.0

    def step(
        self,
        com: np.ndarray,
        com_velocity: np.ndarray,
        landing: np.ndarray | None,
        rotation: np.ndarray | None,
        progress: float,
        active: bool,
        dt: float,
        capture_target: np.ndarray | None = None,
    ) -> CaptureCommand:
        """Advance one control tick.

        ``landing`` is the planned ankle position. ``rotation`` is the swing
        ankle orientation, so a sole target includes the hip-roll tilt.
        ``capture_target`` replaces that sole centre when the walking phase
        has chosen somewhere else, such as the stance inner edge.
        ``active`` is false outside the swing and the touchdown hold.
        """
        com = np.asarray(com, dtype=float).reshape(3)
        velocity = np.asarray(com_velocity, dtype=float).reshape(3)
        current = capture_point(com, velocity)
        if not active or (landing is None and capture_target is None):
            self.reset()
            empty = np.full(2, np.nan)
            return CaptureCommand(
                False,
                0.0,
                current,
                empty,
                empty,
                np.zeros(2),
                empty,
                empty,
            )
        blend = preparation_blend(progress, self.start, self.end)
        if capture_target is None:
            desired, low, high = sole_capture_target(landing, rotation, self.margin)
        else:
            desired = np.asarray(capture_target, dtype=float).reshape(2).copy()
            if landing is None:
                low = np.full(2, np.nan)
                high = np.full(2, np.nan)
            else:
                _sole, low, high = sole_capture_target(landing, rotation, self.margin)
        raw = angular_velocity(com[2]) * (desired - com[:2])
        raw = _clip_velocity(raw, self.max_forward_velocity, self.max_lateral_velocity)
        limited = self._approach(blend * raw, dt)
        return CaptureCommand(
            True,
            blend,
            current,
            desired,
            desired - current,
            limited,
            low,
            high,
        )

    def _approach(self, target: np.ndarray, dt: float) -> np.ndarray:
        """Move the commanded velocity toward ``target`` at ``max_acceleration``."""
        target = np.asarray(target, dtype=float).reshape(2)
        delta = target - self._velocity
        step = max(0.0, self.max_acceleration) * max(0.0, float(dt))
        distance = float(np.linalg.norm(delta))
        if distance > step and distance > 1e-12:
            delta = delta / distance * step
        self._velocity = self._velocity + delta
        return self._velocity.copy()


def angular_velocity(com_z: float) -> float:
    """LIPM natural frequency ``sqrt(g / z)`` at the measured CoM height."""
    return float(np.sqrt(GRAVITY / max(float(com_z), 0.2)))


def capture_point(com: np.ndarray, com_velocity: np.ndarray) -> np.ndarray:
    """Horizontal capture point from the measured centre of mass."""
    com = np.asarray(com, dtype=float).reshape(-1)
    velocity = np.asarray(com_velocity, dtype=float).reshape(-1)
    height = float(com[2]) if com.size >= 3 else 0.9
    return com[:2] + velocity[:2] / angular_velocity(height)


def preparation_blend(progress: float, start: float = 0.0, end: float = 0.70) -> float:
    """Smoothstep from 0 at ``start`` to 1 at ``end``. Holds after ``end``."""
    begin = float(start)
    finish = float(end)
    if float(progress) <= begin:
        return 0.0
    if finish <= begin or float(progress) >= finish:
        return 1.0
    span = finish - begin
    s = float(np.clip((float(progress) - begin) / span, 0.0, 1.0))
    return (3.0 * s * s) - (2.0 * s * s * s)


def sole_capture_target(
    landing: np.ndarray,
    rotation: np.ndarray | None = None,
    margin: float = 0.015,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """World xy of the inset centre of the planned landing sole.

    The sole is the ankle-frame rectangle, rotated by the swing ankle so a
    hip-roll tilt moves the support region. The target is the centre of that
    sole, clamped inside the rectangle inset by ``margin``. Returns the
    target and the world-xy bounds of the four corners.
    """
    landing = np.asarray(landing, dtype=float).reshape(3)
    if rotation is None:
        rotation = np.eye(3)
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    margin = max(0.0, float(margin))
    corners = _sole_corners()
    world = landing + corners @ rotation.T
    low = world[:, :2].min(axis=0)
    high = world[:, :2].max(axis=0)
    local = np.array([0.5 * (SOLE_HEEL + SOLE_TOE), 0.0, 0.0])
    x_lo = SOLE_HEEL + margin
    x_hi = SOLE_TOE - margin
    y_lo = -SOLE_HALF_WIDTH + margin
    y_hi = SOLE_HALF_WIDTH - margin
    if x_hi <= x_lo:
        local[0] = 0.5 * (SOLE_HEEL + SOLE_TOE)
    else:
        local[0] = float(np.clip(local[0], x_lo, x_hi))
    if y_hi <= y_lo:
        local[1] = 0.0
    else:
        local[1] = float(np.clip(local[1], y_lo, y_hi))
    target = landing + rotation @ local
    return target[:2].copy(), low, high


def _sole_corners() -> np.ndarray:
    return np.array(
        [
            [SOLE_TOE, SOLE_HALF_WIDTH, 0.0],
            [SOLE_TOE, -SOLE_HALF_WIDTH, 0.0],
            [SOLE_HEEL, SOLE_HALF_WIDTH, 0.0],
            [SOLE_HEEL, -SOLE_HALF_WIDTH, 0.0],
        ]
    )


def _clip_velocity(velocity: np.ndarray, max_forward: float, max_lateral: float) -> np.ndarray:
    velocity = np.asarray(velocity, dtype=float).reshape(2).copy()
    velocity[0] = float(np.clip(velocity[0], -abs(max_forward), abs(max_forward)))
    velocity[1] = float(np.clip(velocity[1], -abs(max_lateral), abs(max_lateral)))
    return velocity
