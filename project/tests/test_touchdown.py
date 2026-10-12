"""Touchdown residual helpers: observation, clipping, phase, reward, checkpoint."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl.safe_rl_training import LagrangeMultiplier, ResidualGaussianPolicy, SafeRLConfig  # noqa: E402
from rl.touchdown_train import _batch  # noqa: E402
from rl.touchdown import (  # noqa: E402
    OBS_NAMES,
    TouchdownAgent,
    TouchdownConfig,
    apply_com_residual,
    build_touchdown_observation,
    cbf_cost,
    detect_touchdown,
    failure_reason,
    normalize_observation,
    reward_components,
    scale_residual,
    stabilization_ok,
    touchdown_rl_active,
)


def test_observation_dimension_and_order():
    assert len(OBS_NAMES) == 23
    raw = build_touchdown_observation(
        com=np.array([0.02, 0.2, 0.95]),
        velocity=np.array([0.01, -0.02, 0.0]),
        reference_xy=np.array([0.02, 0.2]),
        barriers={
            "support_x_lower": 0.04,
            "support_x_upper": 0.08,
            "support_y_lower": 0.01,
            "support_y_upper": 0.03,
        },
        rotation=np.eye(3),
        stance_side="left",
        stance_hip_roll=-0.2,
        stance_ankle=-0.4,
        swing_xyz=np.array([0.1, -0.2, 0.12]),
        swing_vz=-0.1,
        progress=0.8,
        time_since_activation=0.1,
        intervention=3.0,
    )
    assert raw.shape == (23,)
    assert np.all(np.isfinite(raw))
    assert raw[8] == 0.04 and raw[10] == 0.01 and raw[11] == 0.03
    scaled = normalize_observation(raw, TouchdownConfig().obs_scale())
    assert scaled.shape == (23,)
    assert scaled.dtype == np.float32
    dirty = raw.copy()
    dirty[0] = np.nan
    cleaned = normalize_observation(dirty, TouchdownConfig().obs_scale())
    assert cleaned[0] == 0.0


def test_residual_is_clipped():
    bounds = np.array([0.02, 0.02])
    huge = scale_residual(np.array([50.0, -50.0]), bounds)
    assert np.allclose(huge, [0.02, -0.02])
    zero = scale_residual(np.zeros(2), bounds)
    assert np.allclose(zero, 0.0)
    reference = {"com": np.array([[0.05, 0.24, 0.95]])}
    polygon = np.array([[0.0, 0.20], [0.10, 0.20], [0.10, 0.24], [0.0, 0.24]])
    updated, applied = apply_com_residual(reference, np.array([0.02, 0.02]), polygon)
    assert updated["com"][0, 0] <= 0.10 + 1e-9
    assert updated["com"][0, 1] <= 0.24 + 1e-9
    assert applied[1] == 0.0
    assert reference["com"][0, 1] == 0.24


def test_touchdown_and_phase():
    cfg = TouchdownConfig()
    assert not detect_touchdown(progress=0.2, foot_z=0.05, foot_vz=0.0, in_contact=True, config=cfg)
    assert detect_touchdown(progress=0.8, foot_z=0.2, foot_vz=0.0, in_contact=True, config=cfg)
    assert not detect_touchdown(progress=0.8, foot_z=0.2, foot_vz=-1.0, in_contact=False, config=cfg)
    assert detect_touchdown(progress=0.9, foot_z=0.08, foot_vz=-0.1, in_contact=False, config=cfg)
    assert not touchdown_rl_active(
        state_name="RIGHT_SWING", progress=0.2, seconds_since_touchdown=None, stable=False, config=cfg
    )
    assert touchdown_rl_active(
        state_name="RIGHT_SWING", progress=0.7, seconds_since_touchdown=None, stable=False, config=cfg
    )
    assert touchdown_rl_active(
        state_name="DOUBLE_SUPPORT", progress=1.0, seconds_since_touchdown=0.1, stable=False, config=cfg
    )
    assert not touchdown_rl_active(
        state_name="DOUBLE_SUPPORT", progress=1.0, seconds_since_touchdown=0.1, stable=True, config=cfg
    )


def test_reward_success_and_failure():
    cfg = TouchdownConfig()
    raw = np.zeros(len(OBS_NAMES))
    raw[8:12] = 0.05
    parts = reward_components(raw, np.zeros(2), touchdown_event=True, stable_event=True, config=cfg)
    assert parts["touchdown"] == cfg.w_touchdown
    assert parts["stable"] == cfg.w_stable
    assert abs(parts["total"] - sum(value for key, value in parts.items() if key != "total")) < 1e-9
    quiet = reward_components(raw, np.array([0.02, 0.02]), touchdown_event=False, stable_event=False, config=cfg)
    assert quiet["residual"] < 0.0
    assert stabilization_ok(
        com_z=0.93, vx=0.0, vy=0.0, margins=np.array([0.02, 0.02, 0.02, 0.02]),
        pitch=0.1, roll=0.1, config=cfg,
    )
    assert not stabilization_ok(
        com_z=0.93, vx=0.0, vy=0.5, margins=np.array([0.02, 0.02, 0.02, 0.02]),
        pitch=0.1, roll=0.1, config=cfg,
    )
    assert failure_reason(
        com_z=0.5, pitch=0.0, roll=0.0, finite=True, fail_safe_run=0, elapsed=0.0, config=cfg
    ) == "fallen"
    assert failure_reason(
        com_z=0.93, pitch=0.0, roll=0.0, finite=True, fail_safe_run=0, elapsed=0.2, config=cfg
    ) == ""
    assert cbf_cost(40.0, "optimal", cfg) == 1.0
    assert cbf_cost(0.0, "fail_safe", cfg) == 1.0


def test_checkpoint_roundtrip_and_disabled_fallback():
    cfg = TouchdownConfig()
    agent = TouchdownAgent(cfg, deterministic=True)
    agent.policy = ResidualGaussianPolicy(len(OBS_NAMES), 2)
    with tempfile.TemporaryDirectory() as folder:
        path = agent.save(Path(folder) / "policy.pt", seed=3, lagrange=1.0)
        loaded = TouchdownAgent(deterministic=True)
        loaded.load(path)
    obs = np.zeros(len(OBS_NAMES), dtype=np.float32)
    action, _, _, _ = loaded._infer(obs)
    residual = scale_residual(action, loaded.config.bounds())
    assert residual.shape == (2,)
    assert np.all(np.abs(residual) <= loaded.config.bounds() + 1e-8)
    # No policy: the residual helper is not applied and the reference is copied unchanged.
    bare = TouchdownAgent(cfg, deterministic=True)
    assert bare.policy is None
    reference = {"com": np.array([[0.04, 0.22, 0.95]])}
    updated, applied = apply_com_residual(reference, np.zeros(2), None)
    assert np.allclose(updated["com"], reference["com"])
    assert np.allclose(applied, 0.0)


def test_lagrange_uses_mean_step_cost():
    """Four steps of 0.5 must constrain lambda with 0.5, not the sum 2.0."""
    costs = [0.5, 0.5, 0.5, 0.5]
    n = len(costs)
    buffer = {
        "obs": [np.zeros(23, dtype=np.float32) for _ in range(n)],
        "actions": [np.zeros(2, dtype=np.float32) for _ in range(n)],
        "log_probs": [0.0] * n,
        "rewards": [0.0] * n,
        "costs": costs,
        "values": [0.0] * n,
        "cost_values": [0.0] * n,
        "dones": [0.0] * n,
    }
    batch = _batch(buffer, TouchdownConfig())
    assert batch["mean_cost"] == 0.5
    assert abs(batch["mean_cost"] - float(sum(costs))) > 1.0
    # Per-step costs of about 0–1 cannot meet a budget of 5, so that default
    # is not used. 0.5 is half of one full trust-region correction.
    assert TouchdownConfig().cost_budget == 0.5

    below = LagrangeMultiplier(SafeRLConfig(cost_budget=5.0, lagrange_init=1.0, lagrange_lr=0.035, lagrange_max=100.0))
    before = below.value
    below.update(float(batch["mean_cost"]))
    assert below.value < before
    assert 0.0 <= below.value <= 100.0

    above = LagrangeMultiplier(SafeRLConfig(cost_budget=0.25, lagrange_init=1.0, lagrange_lr=0.035, lagrange_max=100.0))
    before = above.value
    above.update(float(batch["mean_cost"]))
    assert above.value > before
    assert 0.0 <= above.value <= 100.0

    capped = LagrangeMultiplier(SafeRLConfig(cost_budget=0.0, lagrange_init=50.0, lagrange_lr=1.0, lagrange_max=100.0))
    capped.update(1.0e6)
    assert capped.value == 100.0
    floored = LagrangeMultiplier(SafeRLConfig(cost_budget=5.0, lagrange_init=0.01, lagrange_lr=1.0, lagrange_max=100.0))
    floored.update(0.0)
    assert floored.value == 0.0


def main() -> None:
    checks = (
        ("observation", test_observation_dimension_and_order),
        ("clip", test_residual_is_clipped),
        ("phase", test_touchdown_and_phase),
        ("reward", test_reward_success_and_failure),
        ("checkpoint", test_checkpoint_roundtrip_and_disabled_fallback),
        ("lagrange_scale", test_lagrange_uses_mean_step_cost),
    )
    for name, fn in checks:
        fn()
        print("PASS", name)
    print(f"{len(checks)} passed")


if __name__ == "__main__":
    main()
