"""Capture-point preparation. No MuJoCo."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers.capture_point_controller import (  # noqa: E402
    CapturePreparation,
    angular_velocity,
    capture_point,
    preparation_blend,
    sole_capture_target,
)


def test_capture_point_matches_the_lipm_definition():
    com = np.array([0.10, -0.20, 0.90])
    velocity = np.array([0.05, -0.10, 0.0])
    omega = angular_velocity(0.90)
    point = capture_point(com, velocity)
    assert abs(omega - np.sqrt(9.81 / 0.90)) < 1e-12
    assert np.allclose(point, com[:2] + velocity[:2] / omega)


def test_blend_is_zero_at_the_start_and_full_at_070():
    assert preparation_blend(0.0) == 0.0
    assert preparation_blend(-0.1) == 0.0
    assert abs(preparation_blend(0.70) - 1.0) < 1e-12
    assert abs(preparation_blend(1.0) - 1.0) < 1e-12
    earlier = preparation_blend(0.20)
    later = preparation_blend(0.50)
    assert 0.0 < earlier < later < 1.0


def test_sole_target_stays_inside_the_inset_sole_and_follows_tilt():
    landing = np.array([0.15, -0.10, 0.07])
    target, low, high = sole_capture_target(landing, np.eye(3), margin=0.015)
    assert low[1] < target[1] < high[1]
    assert abs(target[1] - landing[1]) < 1e-9
    assert abs(target[0] - (landing[0] + 0.0525)) < 1e-9
    yaw = 0.40
    rotation = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    tilted, _tilted_low, _tilted_high = sole_capture_target(landing, rotation, margin=0.015)
    # Yaw swings the toe sideways, so the sole centre is no longer on the ankle's y.
    assert abs(tilted[1] - target[1]) > 0.01
    local = rotation.T @ np.array([tilted[0] - landing[0], tilted[1] - landing[1], 0.0])
    assert -0.02 + 0.015 <= local[1] <= 0.02 - 0.015
    assert -0.035 + 0.015 <= local[0] <= 0.140 - 0.015


def test_velocity_is_clipped_rate_limited_and_blended():
    controller = CapturePreparation(
        max_lateral_velocity=0.20,
        max_forward_velocity=0.05,
        max_acceleration=1.0,
        start=0.0,
        end=0.70,
    )
    com = np.array([0.0, 0.25, 0.90])
    velocity = np.zeros(3)
    landing = np.array([0.20, -0.12, 0.07])
    idle = controller.step(com, velocity, landing, np.eye(3), 0.5, False, 0.02)
    assert idle.active is False
    assert np.allclose(idle.desired_com_velocity, 0.0)
    # At the start of swing the blend is zero, so the command stays at rest.
    opened = controller.step(com, velocity, landing, np.eye(3), 0.0, True, 0.02)
    assert opened.active is True
    assert opened.blend == 0.0
    assert np.allclose(opened.desired_com_velocity, 0.0)
    # A one-tick jump to the full target cannot exceed the acceleration step.
    full = controller.step(com, velocity, landing, np.eye(3), 1.0, True, 0.02)
    assert full.blend == 1.0
    assert abs(full.desired_com_velocity[1]) <= 0.02 + 1e-9
    assert abs(full.desired_com_velocity[0]) <= 0.02 + 1e-9
    # Held at the full blend, the command reaches the lateral cap and no further.
    reached = full
    for _ in range(40):
        reached = controller.step(com, velocity, landing, np.eye(3), 1.0, True, 0.02)
    assert abs(abs(reached.desired_com_velocity[1]) - 0.20) < 1e-9
    assert abs(reached.desired_com_velocity[0]) <= 0.05 + 1e-9
    # The landing is in -y, so the commanded lateral velocity points that way.
    assert reached.desired_com_velocity[1] < 0.0
    assert reached.desired_capture[1] < 0.0
    assert np.allclose(
        reached.capture_error,
        reached.desired_capture - reached.current_capture,
    )


def main() -> None:
    test_capture_point_matches_the_lipm_definition()
    test_blend_is_zero_at_the_start_and_full_at_070()
    test_sole_target_stays_inside_the_inset_sole_and_follows_tilt()
    test_velocity_is_clipped_rate_limited_and_blended()
    print("capture point preparation tests passed")


if __name__ == "__main__":
    main()
