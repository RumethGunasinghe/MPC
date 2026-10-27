"""Phase-aware support box. Geometry needs no MuJoCo; the CBF case solves one QP."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers.capture_point_controller import CaptureCommand, CapturePreparation  # noqa: E402
from controllers.support_transition import (  # noqa: E402
    INNER_EDGE_INSET,
    blend_boxes,
    capture_target_y,
    inner_edge_opening,
    stance_inner_edge_y,
    stance_safe_box,
    uniform_safe_box,
)
from safety.cbf_filter import CBFConfig, CBFFilter  # noqa: E402

# Left sole, world y about [0.22, 0.29]. Right sole is well to the negative side.
LEFT = np.array([[0.00, 0.22], [0.18, 0.22], [0.18, 0.29], [0.00, 0.29]])
RIGHT = np.array([[0.05, -0.16], [0.18, -0.16], [0.18, -0.09], [0.05, -0.09]])
COMBINED = np.vstack([LEFT, RIGHT])


def test_standing_support_region_is_the_conservative_box():
    standing = uniform_safe_box(LEFT, margin=0.02)
    early = stance_safe_box(LEFT, "right", opening=0.0, margin=0.02)
    assert np.allclose(standing[0], early[0])
    assert np.allclose(standing[1], early[1])
    assert inner_edge_opening(0.0, "STAND") == 0.0
    # The standing box does not reach the other foot.
    assert standing[0][1] > 0.20
    assert standing[1][1] < 0.29


def test_early_swing_keeps_the_conservative_stance_support():
    assert inner_edge_opening(0.20, "RIGHT_SWING") == 0.0
    early = stance_safe_box(LEFT, "right", opening=0.0)
    # Inner face is the lower y of the left sole, inset by the standing margin.
    assert abs(float(early[0][1]) - (0.22 + 0.02)) < 1e-9
    assert abs(float(early[1][1]) - (0.29 - 0.02)) < 1e-9


def test_late_swing_opens_only_the_inner_edge():
    assert inner_edge_opening(0.39, "RIGHT_SWING") == 0.0
    assert abs(inner_edge_opening(0.75, "RIGHT_SWING") - 1.0) < 1e-12
    mid = inner_edge_opening(0.55, "RIGHT_SWING")
    assert 0.0 < mid < 1.0
    early = stance_safe_box(LEFT, "right", opening=0.0)
    late = stance_safe_box(LEFT, "right", opening=1.0)
    # Outer face stays put. The inner face moves toward the measured edge.
    assert abs(float(late[1][1]) - float(early[1][1])) < 1e-9
    assert float(late[0][1]) < float(early[0][1])
    edge = stance_inner_edge_y(LEFT, "right")
    assert abs(edge - 0.22) < 1e-12
    assert abs(float(late[0][1]) - (edge + INNER_EDGE_INSET)) < 1e-9
    # The open box is still on the stance sole. It does not cover the right foot.
    assert float(late[0][1]) > 0.20
    # A standing margin of zero is already the geometric sole. Opening it
    # must not pull the inner face back inside that sole.
    shut = stance_safe_box(LEFT, "right", opening=0.0, margin=0.0)
    opened = stance_safe_box(LEFT, "right", opening=1.0, margin=0.0)
    assert abs(float(opened[0][1]) - float(shut[0][1])) < 1e-9


def test_touchdown_does_not_use_combined_support_before_contact():
    open_stance = stance_safe_box(LEFT, "right", opening=1.0)
    combined = uniform_safe_box(COMBINED, margin=0.02)
    held = blend_boxes(open_stance[0], open_stance[1], combined[0], combined[1], 0.0)
    assert np.allclose(held[0], open_stance[0])
    assert np.allclose(held[1], open_stance[1])
    assert float(held[0][1]) > 0.20
    assert float(combined[0][1]) < 0.0
    assert inner_edge_opening(1.0, "TOUCHDOWN") == 1.0


def test_confirmed_touchdown_enables_combined_support():
    open_stance = stance_safe_box(LEFT, "right", opening=1.0)
    combined = uniform_safe_box(COMBINED, margin=0.02)
    full = blend_boxes(open_stance[0], open_stance[1], combined[0], combined[1], 1.0)
    assert np.allclose(full[0], combined[0])
    assert np.allclose(full[1], combined[1])
    assert float(full[0][1]) < 0.0
    assert float(full[1][1]) > 0.20


def test_capture_target_aims_at_the_inner_edge_then_the_combined_middle():
    edge = stance_inner_edge_y(LEFT, "right")
    current = 0.25
    early = capture_target_y(current, edge, opening=0.0, swing_side="right")
    assert abs(early - current) < 1e-12
    late = capture_target_y(current, edge, opening=1.0, swing_side="right")
    assert abs(late - (edge + INNER_EDGE_INSET)) < 1e-12
    # Farther in than the sole centre, and nowhere near the right-foot centre.
    assert late < 0.255
    assert late > 0.20
    middle = capture_target_y(
        current,
        edge,
        opening=1.0,
        swing_side="right",
        combined_center_y=0.06,
        recovery_blend=1.0,
    )
    assert abs(middle - 0.06) < 1e-12


def test_cbf_still_returns_a_safe_torque():
    filt = CBFFilter(control_matrix=np.eye(2), config=CBFConfig(margin=0.02))
    filt.set_support_polygon(np.array([[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2]]))
    safe = filt.safe_torque(np.zeros(2), np.array([0.0, 0.0, 0.9]), np.zeros(3))
    assert safe.shape == (2,)
    assert np.all(np.isfinite(safe))
    assert float(np.linalg.norm(safe)) < 1e-6
    # A command that drives through the lower face is pulled back.
    late = stance_safe_box(LEFT, "right", opening=1.0)
    filt.set_support_bounds(late[0], late[1])
    bounds = filt.support_bounds
    assert abs(float(bounds[0][1]) - float(late[0][1])) < 1e-9
    com = np.array([0.09, float(late[0][1]) + 0.004, 0.9])
    clipped = filt.safe_torque(np.array([0.0, -8.0]), com, np.zeros(3))
    assert np.all(np.isfinite(clipped))
    assert float(clipped[1]) > -8.0


def test_capture_point_controller_writes_no_torque():
    names = set(CaptureCommand.__dataclass_fields__)
    assert "torque" not in names
    assert "tau" not in names
    prep = CapturePreparation()
    command = prep.step(
        np.array([0.05, 0.25, 0.90]),
        np.zeros(3),
        None,
        None,
        1.0,
        True,
        0.02,
        capture_target=np.array([0.05, 0.225]),
    )
    assert isinstance(command, CaptureCommand)
    assert command.active
    assert command.desired_com_velocity.shape == (2,)
    assert float(command.desired_com_velocity[1]) < 0.0
    assert abs(float(command.desired_com_velocity[1])) <= 0.20 + 1e-9


def _main() -> None:
    tests = [
        test_standing_support_region_is_the_conservative_box,
        test_early_swing_keeps_the_conservative_stance_support,
        test_late_swing_opens_only_the_inner_edge,
        test_touchdown_does_not_use_combined_support_before_contact,
        test_confirmed_touchdown_enables_combined_support,
        test_capture_target_aims_at_the_inner_edge_then_the_combined_middle,
        test_cbf_still_returns_a_safe_torque,
        test_capture_point_controller_writes_no_torque,
    ]
    for test in tests:
        test()
        print("passed", test.__name__)
    print(f"{len(tests)} passed")


if __name__ == "__main__":
    _main()
