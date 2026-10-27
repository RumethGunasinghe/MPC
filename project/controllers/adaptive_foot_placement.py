"""Late-swing lateral landing adjustment from the capture point.

The footstep planner still chooses the forward landing. This module only
moves that landing's lateral coordinate, and only by a bounded amount, so
the capture point can sit in the combined support of both soles.

The swing ankle is not placed on the capture point. The ideal ankle
position is the one that puts the capture point halfway between the inner
edges of the two soles. A gain below one, and a hard metre clamp, keep the
step inside the existing lateral spacing.

The sole rectangle used here is the same one the walking controller uses:
heel at -0.035 m, toe at 0.140 m, lateral half-width 0.020 m.
"""

from __future__ import annotations

import numpy as np

# Ankle-frame sole, metres. Matches ``_SOLE_CORNERS`` in the demo.
SOLE_HEEL = -0.035
SOLE_TOE = 0.140
SOLE_HALF_WIDTH = 0.020


def placement_ramp(progress: float, start: float = 0.65) -> float:
    """Smoothstep from 0 at ``start`` to 1 at the end of the swing.

    Before ``start`` the value is 0, so the landing target does not move
    during the lift or the early forward travel.
    """
    begin = float(start)
    if float(progress) <= begin or begin >= 1.0:
        return 0.0
    span = 1.0 - begin
    s = float(np.clip((float(progress) - begin) / span, 0.0, 1.0))
    return (3.0 * s * s) - (2.0 * s * s * s)


def lateral_landing_correction(
    swing_target_y: float,
    stance_foot_y: float,
    capture_y: float,
    *,
    gain: float,
    max_adjustment: float,
    half_width: float = SOLE_HALF_WIDTH,
    stance_inner_y: float | None = None,
) -> float:
    """Bounded lateral correction of the swing ankle, in metres.

    Positive moves the swing foot toward +y. The correction is measured
    from ``swing_target_y`` and is clamped to ``±max_adjustment``. It does
    not change the forward landing.

    ``stance_inner_y`` is the stance sole's edge that faces the swing foot.
    When it is omitted, that edge is the stance ankle shifted by
    ``half_width`` toward the swing foot.
    """
    half = abs(float(half_width))
    swing = float(swing_target_y)
    stance = float(stance_foot_y)
    capture = float(capture_y)
    if stance_inner_y is None:
        stance_inner = stance - half if swing <= stance else stance + half
    else:
        stance_inner = float(stance_inner_y)
    # Halfway between the inner edges should be the capture point.
    # swing_inner = swing_y ± half, facing the stance foot.
    if swing <= stance:
        ideal = 2.0 * capture - stance_inner - half
    else:
        ideal = 2.0 * capture - stance_inner + half
    raw = float(gain) * (ideal - swing)
    limit = abs(float(max_adjustment))
    correction = float(np.clip(raw, -limit, limit))
    # Stay on the original side of the stance ankle. An 8 cm clamp on the
    # current spacing does not reach this, but a later gain change might.
    updated = swing + correction
    if swing < stance and updated > stance - half:
        correction = float(np.clip((stance - half) - swing, -limit, limit))
    elif swing > stance and updated < stance + half:
        correction = float(np.clip((stance + half) - swing, -limit, limit))
    return correction


def combined_y_margin(capture_y: float, sole_y: np.ndarray) -> float:
    """Signed distance of ``capture_y`` inside the combined sole y-span.

    Positive means the capture point is between the outer sole edges.
    The value is the distance to the nearer edge. An empty span is zero.
    """
    ys = np.asarray(sole_y, dtype=float).reshape(-1)
    if ys.size == 0:
        return 0.0
    low = float(ys.min())
    high = float(ys.max())
    return float(min(float(capture_y) - low, high - float(capture_y)))
