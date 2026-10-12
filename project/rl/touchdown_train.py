"""Train the late-swing touchdown residual.

The deterministic gait runs until late swing. That state is snapshotted and
each episode restarts there, so the policy only learns the landing window.

    python -m rl.touchdown_train --episodes 4 --seed 0
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from controllers.leg_ik import LegIK  # noqa: E402
from controllers.walk_state_machine import WalkConfig, WalkingStateMachine  # noqa: E402
from main_demo import _ankle_pose, build_stack, walk_tick  # noqa: E402
from rl.safe_rl_training import (  # noqa: E402
    LagrangeMultiplier,
    ResidualGaussianPolicy,
    SafeRLConfig,
    _gae,
    constrained_policy_update,
)
from rl.touchdown import OBS_NAMES, TouchdownAgent, TouchdownConfig  # noqa: E402


def _demo_args(seed: int):
    import argparse as _argparse

    return _argparse.Namespace(
        horizon=20,
        control_dt=0.02,
        seed=int(seed),
        model_path=Path(__file__).resolve().parents[1] / "models" / "unitree_h1" / "scene.xml",
        walk=True,
        steps=1,
        step_length=0.1,
        step_duration=0.8,
        swing_ramp=0.03,
        freeze_swing=False,
        late_swing_capture=False,
        landing_handoff=False,
    )


def _build(seed: int):
    args = _demo_args(seed)
    env, mpc, cbf, pd = build_stack(args)
    env.reset(seed=seed)
    state0 = env.get_state()
    gait = WalkingStateMachine(
        WalkConfig(
            step_length=0.1,
            step_height=0.05,
            step_duration=0.8,
            shift_duration=1.5,
            double_support_duration=0.4,
            stand_duration=1.5,
        )
    )
    ik = LegIK(env.model)
    left, _ = _ankle_pose(env, "left")
    right, _ = _ankle_pose(env, "right")
    gait.start(left, right, float(state0["com"][2]), n_steps=1, t0=0.0, first_swing="right")
    cbf.config.margin = 0.0
    return env, mpc, cbf, pd, gait, ik, np.asarray(state0["torso_rotation"], dtype=float).copy()


def _snapshot(env, gait, mpc) -> dict:
    shift = getattr(gait, "_lateral_shift", None)
    return {
        "qpos": env.data.qpos.copy(),
        "qvel": env.data.qvel.copy(),
        "time": float(env.data.time),
        "gait": {key: copy.deepcopy(value) for key, value in gait.__dict__.items() if key != "_lateral_shift"},
        "shift": None
        if shift is None
        else {key: copy.deepcopy(value) for key, value in shift.__dict__.items() if key != "model"},
        "home": np.array(mpc._home_joint_qpos, copy=True),
    }


def _restore(env, gait, mpc, snap: dict, rng: np.random.Generator, config: TouchdownConfig) -> None:
    import mujoco

    env.data.qpos[:] = snap["qpos"]
    env.data.qvel[:] = snap["qvel"]
    if config.randomize and config.disturbance_xy > 0.0:
        noise = rng.uniform(-config.disturbance_xy, config.disturbance_xy, size=2)
        env.data.qvel[:2] += noise
    env.data.time = snap["time"]
    mujoco.mj_forward(env.model, env.data)
    for key, value in snap["gait"].items():
        setattr(gait, key, copy.deepcopy(value))
    shift = getattr(gait, "_lateral_shift", None)
    if shift is not None and snap["shift"] is not None:
        for key, value in snap["shift"].items():
            setattr(shift, key, copy.deepcopy(value))
    mpc._home_joint_qpos = np.array(snap["home"], copy=True)


def _warmup(env, mpc, cbf, pd, gait, ik, torso, config: TouchdownConfig) -> dict:
    rows: list[dict] = []
    duration = float(gait.config.step_duration)
    for _ in range(1500):
        row = walk_tick(env, mpc, cbf, pd, gait, ik, torso, rows)
        rows.append(row)
        name = str(row.get("walk_state", ""))
        progress = float(gait._t_state) / duration if duration > 0.0 else 0.0
        if name in ("RIGHT_SWING", "LEFT_SWING") and progress >= config.late_swing_start:
            return _snapshot(env, gait, mpc)
        if float(np.asarray(row["com"]).reshape(-1)[2]) < config.fall_height:
            raise RuntimeError("deterministic gait fell before late swing")
    raise RuntimeError("late swing was not reached")


def train(config: TouchdownConfig, out: Path) -> Path:
    import torch

    _reset_training_log()
    rng = np.random.default_rng(config.seed)
    torch.manual_seed(config.seed)
    env, mpc, cbf, pd, gait, ik, torso = _build(config.seed)
    snap = _warmup(env, mpc, cbf, pd, gait, ik, torso, config)
    agent = TouchdownAgent(config, deterministic=False)
    agent.policy = ResidualGaussianPolicy(len(OBS_NAMES), 2)
    optimizer = torch.optim.Adam(agent.policy.parameters(), lr=config.learning_rate)
    lagrange = LagrangeMultiplier(
        SafeRLConfig(
            cost_budget=config.cost_budget,
            lagrange_init=config.lagrange_init,
            lagrange_lr=config.lagrange_lr,
            lagrange_max=config.lagrange_max,
        )
    )
    buffer: dict[str, list] = {key: [] for key in ("obs", "actions", "log_probs", "rewards", "costs", "values", "cost_values", "dones")}
    for episode in range(int(config.episodes)):
        _restore(env, gait, mpc, snap, rng, config)
        agent.reset_episode()
        rows: list[dict] = []
        for _ in range(int(config.max_rl_seconds / config.control_dt) + 5):
            row = walk_tick(
                env, mpc, cbf, pd, gait, ik, torso, rows, touchdown_agent=agent
            )
            rows.append(row)
            if row.get("rl_active"):
                parts = row.get("rl_reward_components", {})
                buffer["obs"].append(agent.last["normalized"].copy())
                buffer["actions"].append(agent.last["action"].copy())
                buffer["log_probs"].append(agent.last["log_prob"])
                buffer["rewards"].append(float(parts.get("total", 0.0)))
                buffer["costs"].append(float(parts.get("cbf_cost", 0.0)))
                buffer["values"].append(agent.last["value"])
                buffer["cost_values"].append(agent.last["cost_value"])
                done = bool(agent.stable or agent.reason)
                buffer["dones"].append(float(done))
            if agent.stable or agent.reason or float(row["com"][2]) < config.fall_height:
                break
        if len(buffer["obs"]) >= config.steps_per_update or episode == config.episodes - 1:
            if buffer["obs"]:
                batch = _batch(buffer, config)
                metrics = constrained_policy_update(
                    agent.policy,
                    optimizer,
                    lagrange.value,
                    batch,
                    minibatch_size=config.minibatch_size,
                    epochs=config.epochs_per_update,
                    clip_range=config.clip_range,
                )
                # Same per-step mean the log reports. Do not pass the episode sum.
                lagrange.update(float(metrics["mean_cost"]))
                print(
                    f"episode {episode} steps {len(buffer['obs'])} "
                    f"reward {metrics['mean_reward']:.3f} "
                    f"mean_cbf_cost {metrics['mean_cost']:.3f} "
                    f"max_cbf_cost {batch['max_cost']:.3f} "
                    f"lambda {lagrange.value:.3f} "
                    f"stable {int(agent.stable)} reason {agent.reason or '-'}"
                )
                _append_training_row(
                    episode=int(episode),
                    steps=len(buffer["obs"]),
                    reward=float(metrics["mean_reward"]),
                    mean_cbf_cost=float(metrics["mean_cost"]),
                    max_cbf_cost=float(batch["max_cost"]),
                    lagrange=float(lagrange.value),
                    stable=int(agent.stable),
                    reason=str(agent.reason or "-"),
                    cost_budget=float(config.cost_budget),
                )
                buffer = {key: [] for key in buffer}
    path = agent.save(out, seed=config.seed, lagrange=lagrange.value)
    print(f"saved {path}")
    return path


def _batch(buffer: dict[str, list], config: TouchdownConfig) -> dict[str, np.ndarray]:
    rewards = np.asarray(buffer["rewards"], dtype=float)
    costs = np.asarray(buffer["costs"], dtype=float)
    values = np.asarray(buffer["values"], dtype=float)
    cost_values = np.asarray(buffer["cost_values"], dtype=float)
    dones = np.asarray(buffer["dones"], dtype=float)
    reward_adv, reward_return = _gae(rewards, values, dones, config.gamma, config.gae_lambda)
    cost_adv, cost_return = _gae(costs, cost_values, dones, config.gamma, config.gae_lambda)
    return {
        "obs": np.vstack(buffer["obs"]).astype(np.float32),
        "actions": np.vstack(buffer["actions"]).astype(np.float32),
        "log_probs": np.asarray(buffer["log_probs"], dtype=np.float32),
        "reward_adv": reward_adv.astype(np.float32),
        "cost_adv": cost_adv.astype(np.float32),
        "reward_return": reward_return.astype(np.float32),
        "cost_return": cost_return.astype(np.float32),
        "mean_reward": float(np.mean(rewards)),
        "mean_cost": float(np.mean(costs)),
        "max_cost": float(np.max(costs)) if costs.size else 0.0,
        "mean_intervention": float(np.mean(costs) * config.cbf_cost_scale),
    }


def _training_log_path() -> Path:
    return Path(__file__).resolve().parents[1] / "data" / "logs" / "touchdown_training_latest.csv"


def _reset_training_log() -> None:
    path = _training_log_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("", encoding="utf-8")


def _append_training_row(**fields) -> None:
    """Append one update to the training CSV. Does not affect the policy step."""
    import csv

    path = _training_log_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fields))
        if new_file:
            writer.writeheader()
        writer.writerow(fields)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train the late-swing touchdown residual")
    parser.add_argument("--episodes", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--steps-per-update", type=int, default=64)
    parser.add_argument("--out", type=Path, default=Path("data/runs/touchdown_policy.pt"))
    parser.add_argument("--randomize", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = TouchdownConfig(
        episodes=args.episodes,
        seed=args.seed,
        steps_per_update=args.steps_per_update,
        randomize=bool(args.randomize),
        disturbance_xy=0.02 if args.randomize else 0.0,
    )
    train(config, args.out)


if __name__ == "__main__":
    main()
