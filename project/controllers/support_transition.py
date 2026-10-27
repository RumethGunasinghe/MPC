"""Phase-aware support box and the controlled lateral fall.

The CBF stays the last torque filter. Early swing keeps the stance sole.
Late swing moves only the inner face into the gap so the body can fall
toward the swing foot. After both feet are in contact the box is the
measured combined polygon.

The stance hip uses the same lateral capture offset as the whole-body MPC.
Early swing holds the weight-shift lean. Late swing blends that offset in.
Touchdown keeps it. It releases only after the capture point is inside the
combined soles and the new foot is carrying load.
"""

from __future__ import annotations

import numpy as np

from controllers.capture_point_controller import preparation_blend
from controllers.mpc_controller import lateral_capture_hip_offset

# Metres. The standing inset matches the CBF's default face margin.
STANDING_INSET = 0.02
# How far inside the geometric inner edge the late-swing target sits.
INNER_EDGE_INSET = 0.005
# Swing progress over which the inner face opens.
LATE_START = 0.40
LATE_END = 0.75
# Seconds after confirmed contact before the box is fully combined.
RECOVERY_BLEND_S = 0.20
# Swing progress over which the controlled fall blends in.
FALL_START = 0.30
FALL_END = 0.70
# How far the late-swing inner face travels past the measured stance edge.
FALL_REACH = 0.18
# Capture reference the fall steers toward. Not a foot centre.
TRANSITION_Y = 0.0
# Release once the new foot has carried this load for the dwell, and the
# capture point is this far inside the combined soles.
RELEASE_FORCE_N = 50.0
RELEASE_DWELL_S = 0.15
RELEASE_MARGIN_M = 0.01
RELEASE_BLEND_S = 0.30


def support_phase(state: str, swing_progress: float, late_start: float = LATE_START) -> str:
    """Name of the support model for this tick."""
    if state in ("RIGHT_SWING", "LEFT_SWING"):
        if float(swing_progress) >= float(late_start):
            return "LATE_SWING"
        return "EARLY_SWING"
    if state == "TOUCHDOWN_PREP":
        return "TOUCHDOWN_PREP"
    if state == "TOUCHDOWN":
        return "TOUCHDOWN"
    if state == "DOUBLE_SUPPORT_RECOVERY":
        return "DOUBLE_SUPPORT_RECOVERY"
    if state in ("SHIFT_LEFT", "SHIFT_RIGHT"):
        return "SHIFT"
    return "STAND"


def inner_edge_opening(swing_progress: float, state: str) -> float:
    """0 while the stance box is conservative, 1 when the inner edge is open.

    Touchdown prep and touchdown hold the open edge. Recovery is already
    past this blend; the combined-support blend is separate.
    """
    if state in ("TOUCHDOWN_PREP", "TOUCHDOWN", "DOUBLE_SUPPORT_RECOVERY"):
        return 1.0
    if state not in ("RIGHT_SWING", "LEFT_SWING"):
        return 0.0
    return preparation_blend(swing_progress, LATE_START, LATE_END)


def stance_inner_edge_y(corners: np.ndarray, swing_side: str) -> float:
    """World y of the stance sole edge that faces the swinging foot.

    ``swing_side`` is the foot in the air. A right swing stands on the left
    sole, whose inner edge is its smaller y. The corners are the measured
    sole, so hip-roll tilt is already in the y span.
    """
    y = np.asarray(corners, dtype=float).reshape(-1, 2)[:, 1]
    if swing_side == "left":
        return float(y.max())
    return float(y.min())


def _inside_sign(swing_side: str) -> float:
    """+1 when the interior of the stance sole is toward +y of the inner edge."""
    if swing_side == "left":
        return -1.0
    return 1.0


def stance_safe_box(
    corners: np.ndarray,
    swing_side: str,
    opening: float,
    margin: float = STANDING_INSET,
    inner_inset: float = INNER_EDGE_INSET,
) -> tuple[np.ndarray, np.ndarray]:
    """Safe xy box for one stance sole.

    ``opening`` 0 insets every face by ``margin``. ``opening`` 1 leaves the
    outer, heel and toe faces at ``margin`` and moves only the inner face
    to ``inner_inset`` inside the measured edge.
    """
    points = np.asarray(corners, dtype=float).reshape(-1, 2)
    low = points.min(axis=0)
    high = points.max(axis=0)
    inset_low = np.full(2, float(margin))
    inset_high = np.full(2, float(margin))
    # The open face sits on the inner edge. It is never inset further than the
    # closed face, so a demo whose standing margin is already zero does not
    # grow a new wall during late swing.
    closed = float(margin)
    opened = min(closed, float(inner_inset))
    inner = (1.0 - float(opening)) * closed + float(opening) * opened
    if swing_side == "left":
        inset_high[1] = inner
    else:
        inset_low[1] = inner
    safe_low = low + inset_low
    safe_high = high - inset_high
    safe_high = np.maximum(safe_high, safe_low + 1e-3)
    return safe_low, safe_high


def uniform_safe_box(corners: np.ndarray, margin: float = STANDING_INSET) -> tuple[np.ndarray, np.ndarray]:
    """Every face inset by ``margin``. This is the standing box."""
    points = np.asarray(corners, dtype=float).reshape(-1, 2)
    low = points.min(axis=0) + float(margin)
    high = points.max(axis=0) - float(margin)
    high = np.maximum(high, low + 1e-3)
    return low, high


def blend_boxes(
    first_low: np.ndarray,
    first_high: np.ndarray,
    second_low: np.ndarray,
    second_high: np.ndarray,
    blend: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Linear blend of two safe boxes. ``blend`` 0 keeps the first."""
    weight = float(np.clip(blend, 0.0, 1.0))
    low = (1.0 - weight) * np.asarray(first_low, dtype=float) + weight * np.asarray(second_low, dtype=float)
    high = (1.0 - weight) * np.asarray(first_high, dtype=float) + weight * np.asarray(second_high, dtype=float)
    return low, high


def capture_target_y(
    capture_y: float,
    inner_edge_y: float,
    opening: float,
    swing_side: str,
    *,
    combined_center_y: float | None = None,
    recovery_blend: float = 0.0,
) -> float:
    """Lateral capture target for this phase.

    Before the inner edge is open the target is the current capture point,
    so the command does not pull inward during early swing. At full opening
    the target is ``INNER_EDGE_INSET`` inside the measured edge. After
    confirmed contact, ``recovery_blend`` carries it toward the middle of
    the combined support, not toward the far sole.
    """
    edge = float(inner_edge_y) + _inside_sign(swing_side) * INNER_EDGE_INSET
    toward_edge = (1.0 - float(opening)) * float(capture_y) + float(opening) * edge
    if combined_center_y is None or float(recovery_blend) <= 0.0:
        return float(toward_edge)
    weight = float(np.clip(recovery_blend, 0.0, 1.0))
    return float((1.0 - weight) * toward_edge + weight * float(combined_center_y))


def combined_support_active(state: str, both_in_contact: bool) -> bool:
    """The two-foot polygon is used only after contact has been confirmed."""
    return state == "DOUBLE_SUPPORT_RECOVERY" and bool(both_in_contact)


def fall_blend(swing_progress: float, state: str) -> float:
    """0 during early swing, 1 from late swing through the catch.

    Standing and the weight shift stay at 0. Touchdown and recovery hold 1
    so the inward correction is not removed when the foot lands.
    """
    if state in ("TOUCHDOWN_PREP", "TOUCHDOWN", "DOUBLE_SUPPORT_RECOVERY"):
        return 1.0
    if state not in ("RIGHT_SWING", "LEFT_SWING"):
        return 0.0
    return preparation_blend(swing_progress, FALL_START, FALL_END)


def fall_safe_box(
    corners: np.ndarray,
    swing_side: str,
    opening: float,
    reach: float = FALL_REACH,
) -> tuple[np.ndarray, np.ndarray]:
    """Stance sole with only the inner face moved into the gap.

    ``opening`` 0 is the measured sole. ``opening`` 1 moves the inner face
    ``reach`` metres toward the swing foot. The outer, heel and toe faces
    stay on the measured sole.
    """
    points = np.asarray(corners, dtype=float).reshape(-1, 2)
    low = points.min(axis=0).copy()
    high = points.max(axis=0).copy()
    travel = float(np.clip(opening, 0.0, 1.0)) * float(reach)
    if swing_side == "left":
        high[1] = high[1] + travel
    else:
        low[1] = low[1] - travel
    high = np.maximum(high, low + 1e-3)
    return low, high


def stance_hip_roll_capture_target(
    base: float,
    capture_y: float,
    target_y: float = TRANSITION_Y,
    weight: float = 1.0,
    *,
    gain: float = 12.0,
    limit: float = 0.15,
    home: float | None = None,
    release: float = 0.0,
) -> float:
    """Stance hip-roll target for the controlled fall.

    ``weight`` 0 returns the latched lean. ``weight`` 1 adds the MPC's
    lateral capture offset, clipped to ``limit``, toward ``target_y``.
    ``release`` then blends that target toward the double-support home angle.
    """
    offset = lateral_capture_hip_offset(capture_y, target_y, gain, limit)
    held = float(base) + float(np.clip(weight, 0.0, 1.0)) * offset
    if home is None or float(release) <= 0.0:
        return float(held)
    mix = float(np.clip(release, 0.0, 1.0))
    return float((1.0 - mix) * held + mix * float(home))


def apply_stance_hip_roll(
    q_des: np.ndarray,
    joint_names: list[str],
    stance_side: str,
    angle: float,
    low: np.ndarray,
    high: np.ndarray,
) -> np.ndarray:
    """Write the stance hip roll. Every other joint, including the swing hip, stays."""
    q = np.asarray(q_des, dtype=float).copy()
    name = f"{stance_side}_hip_roll"
    index = list(joint_names).index(name)
    q[index] = float(np.clip(angle, float(low[index]), float(high[index])))
    return q


def capture_box_margin(point: np.ndarray, corners: np.ndarray) -> float:
    """Signed distance of ``point`` inside the sole box. Negative when outside."""
    xy = np.asarray(point, dtype=float).reshape(-1)[:2]
    bounds = np.asarray(corners, dtype=float).reshape(-1, 2)
    low = bounds.min(axis=0)
    high = bounds.max(axis=0)
    return float(min(xy[0] - low[0], high[0] - xy[0], xy[1] - low[1], high[1] - xy[1]))


class FallRelease:
    """Hold the inward hip correction until the new foot is actually loaded.

    The dwell is time spent above the force threshold, not a clock started
    at touchdown. The blend stays at 0 until both the force dwell and the
    capture margin are true, then rises smoothly to 1.
    """

    def __init__(self) -> None:
        self.force_time = 0.0
        self.started = False
        self.blend = 0.0

    def update(
        self,
        *,
        in_recovery: bool,
        capture_margin: float,
        new_foot_force: float,
        dt: float,
    ) -> float:
        if not in_recovery:
            self.force_time = 0.0
            self.started = False
            self.blend = 0.0
            return 0.0
        if self.started:
            self.blend = min(1.0, self.blend + max(0.0, float(dt)) / RELEASE_BLEND_S)
            return float(self.blend)
        if float(new_foot_force) >= RELEASE_FORCE_N:
            self.force_time += max(0.0, float(dt))
        else:
            self.force_time = 0.0
        if self.force_time >= RELEASE_DWELL_S and float(capture_margin) > RELEASE_MARGIN_M:
            self.started = True
        return float(self.blend)
