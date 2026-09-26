"""Safe reinforcement learning on top of the whole-body MPC.

The policy outputs a residual on the MPC reference rather than raw torques, so
the MPC keeps the robot dynamically consistent while RL adapts the task-level
behaviour. Safety is enforced twice: hard, by the CBF filter at every step, and
soft, by a Lagrangian constraint on the expected cumulative safety cost

    max_pi  E[ sum r_t ]   s.t.  E[ sum c_t ] <= budget.

The update is a clipped PPO surrogate on the Lagrangian advantage

    A = A_reward - lambda * A_cost

with separate value functions for return and cost.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# This file lives in project/rl/. Running it as a script puts that folder on
# sys.path, which hides the sibling packages. Put the project root first.
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from controllers.mpc_controller import MPCConfig, WholeBodyMPC  # noqa: E402
from controllers.pd_controller import PDController  # noqa: E402
from env.industrial_humanoid_env import EnvConfig, IndustrialHumanoidEnv  # noqa: E402
from safety.cbf_filter import CBFConfig, CBFFilter  # noqa: E402
from utils.logger import RunLogger  # noqa: E402

try:
    import torch
    from torch import nn
except ImportError:  # pragma: no cover - requirements.txt includes torch
    torch = None
    nn = None

DEFAULT_RUN_DIR = Path(__file__).resolve().parents[1] / "data" / "runs"


@dataclass
class SafeRLConfig:
    """Hyperparameters for the constrained policy optimisation loop."""

    algorithm: str = "ppo_lagrangian"  # ppo_lagrangian | cpo | sac_lagrangian
    total_steps: int = 2_000_000
    steps_per_update: int = 4096
    minibatch_size: int = 256
    epochs_per_update: int = 10

    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    learning_rate: float = 3e-4

    # Constraint handling
    cost_budget: float = 5.0
    lagrange_init: float = 1.0
    lagrange_lr: float = 0.035
    lagrange_max: float = 100.0

    # Residual action scaling on top of the MPC reference
    residual_scale: float = 0.15
    use_cbf_filter: bool = True
    penalise_cbf_intervention: bool = True

    seed: int = 0
    run_dir: Path = DEFAULT_RUN_DIR
    checkpoint_every: int = 50_000

    # Passed through to the environment. Tests and short runs shrink the episode.
    episode_seconds: float = 20.0
    model_path: Path | None = None


class LagrangeMultiplier:
    """Dual variable updated by projected gradient ascent on the constraint."""

    def __init__(self, config: SafeRLConfig) -> None:
        self.config = config
        self.value = config.lagrange_init

    def update(self, mean_episode_cost: float) -> float:
        violation = mean_episode_cost - self.config.cost_budget
        self.value = float(
            np.clip(self.value + self.config.lagrange_lr * violation, 0.0, self.config.lagrange_max)
        )
        return self.value


class ResidualGaussianPolicy(nn.Module if nn is not None else object):
    """Tanh-Gaussian residual on the CoM reference, plus value heads.

    The mean is scaled by ``residual_scale`` inside the trainer, so the raw
    network output stays O(1). ``log_std`` is a free parameter per action axis.
    """

    def __init__(self, obs_dim: int, act_dim: int = 3) -> None:
        if nn is None:  # pragma: no cover
            raise ImportError("torch is required for the safe RL policy")
        super().__init__()
        self.backbone = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
        )
        self.mean_head = nn.Linear(64, act_dim)
        self.value_head = nn.Linear(64, 1)
        self.cost_head = nn.Linear(64, 1)
        self.log_std = nn.Parameter(torch.full((act_dim,), -0.5))

    def forward(self, obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        features = self.backbone(obs)
        mean = self.mean_head(features)
        value = self.value_head(features).squeeze(-1)
        cost_value = self.cost_head(features).squeeze(-1)
        return mean, value, cost_value

    def distribution(self, mean: torch.Tensor) -> torch.distributions.Normal:
        std = self.log_std.clamp(-5.0, 2.0).exp()
        return torch.distributions.Normal(mean, std)


class SafeRLTrainer:
    """Constrained policy optimisation against ``IndustrialHumanoidEnv``."""

    def __init__(self, config: SafeRLConfig | None = None) -> None:
        if torch is None:  # pragma: no cover
            raise ImportError("torch is required for safe RL training")
        self.config = config or SafeRLConfig()
        self.lagrange = LagrangeMultiplier(self.config)
        self._rng = np.random.default_rng(self.config.seed)
        torch.manual_seed(self.config.seed)

        env_config = EnvConfig(episode_seconds=self.config.episode_seconds, reset_joint_noise=0.0)
        if self.config.model_path is not None:
            env_config.model_path = self.config.model_path
        self.env = IndustrialHumanoidEnv(env_config)
        self.mpc = WholeBodyMPC(self.env.model, MPCConfig(dt=self.env.config.control_dt))
        self.cbf = CBFFilter(model=self.env.model, config=CBFConfig()) if self.config.use_cbf_filter else None
        self.pd = PDController(joint_names=self.env.joint_names)

        obs, _ = self.env.reset(seed=self.config.seed)
        self._obs_dim = int(obs.shape[0])
        self._act_dim = 3  # residual on the CoM position reference
        self.policy = ResidualGaussianPolicy(self._obs_dim, self._act_dim)
        self._optimizer = torch.optim.Adam(self.policy.parameters(), lr=self.config.learning_rate)

        state = self.env.get_state()
        self._com_reference = state["com"].copy()
        self._torso_reference = state["torso_rotation"].copy()
        self._global_step = 0

    def _gravity_torque(self) -> np.ndarray:
        """Actuated slice of ``qfrc_bias``: the torque that cancels gravity."""
        return np.asarray(self.env.data.qfrc_bias[self.env._actuator_dof_adr], dtype=float)

    def _track(self, solution: dict) -> tuple:
        """PD at the physics rate, CBF once per control tick, then step."""
        state = self.env.get_state()
        q_des = solution["q_des"]
        qd_des = solution["qd_des"]
        tau_nominal = self.pd.compute_torque(
            q_des,
            state["joint_qpos"],
            state["joint_qvel"],
            desired_velocities=qd_des,
            torque_feedforward=self._gravity_torque(),
        )
        if self.cbf is not None:
            tau_safe = self.cbf.safe_torque(tau_nominal, state["com"], state["com_velocity"])
        else:
            tau_safe = tau_nominal
        delta = tau_safe - tau_nominal

        def torque_at_substep() -> np.ndarray:
            env = self.env
            q = env.data.qpos[env._actuator_qpos_adr]
            qd = env.data.qvel[env._actuator_dof_adr]
            gravity = np.asarray(env.data.qfrc_bias[env._actuator_dof_adr], dtype=float)
            return self.pd.compute_torque(
                q_des, q, qd, desired_velocities=qd_des, torque_feedforward=gravity
            ) + delta

        obs, reward, terminated, truncated, info = self.env.step_feedback(torque_at_substep)
        return obs, reward, terminated, truncated, info, float(np.linalg.norm(delta))

    def collect_rollouts(self) -> dict[str, np.ndarray]:
        """Run the env for ``steps_per_update`` steps, storing rewards and costs.

        Records CBF interventions so the penalty term can discourage the policy
        from relying on the safety filter.
        """
        cfg = self.config
        obs, _ = self.env.reset(seed=int(self._rng.integers(0, 2**31 - 1)))
        state = self.env.get_state()
        self._com_reference = state["com"].copy()
        self._torso_reference = state["torso_rotation"].copy()

        observations = []
        actions = []
        log_probs = []
        rewards = []
        costs = []
        values = []
        cost_values = []
        dones = []
        interventions = []
        episode_costs = []
        running_cost = 0.0

        for _ in range(cfg.steps_per_update):
            obs_tensor = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                mean, value, cost_value = self.policy(obs_tensor)
                dist = self.policy.distribution(mean)
                action = dist.sample()
                log_prob = dist.log_prob(action).sum(-1)
            residual = cfg.residual_scale * np.tanh(action.squeeze(0).numpy())

            state = self.env.get_state()
            reference = {
                "com": self._com_reference + residual,
                "torso_orientation": self._torso_reference,
            }
            solution = self.mpc.solve(state, reference)
            _next_obs, reward, terminated, truncated, info, intervention = self._track(solution)
            cost = float(info["cost"])
            if cfg.penalise_cbf_intervention:
                reward = float(reward) - 1e-3 * intervention
            done = bool(terminated or truncated)
            running_cost += cost

            observations.append(np.asarray(obs, dtype=np.float32))
            actions.append(action.squeeze(0).numpy().astype(np.float32))
            log_probs.append(float(log_prob))
            rewards.append(float(reward))
            costs.append(cost)
            values.append(float(value))
            cost_values.append(float(cost_value))
            dones.append(float(done))
            interventions.append(intervention)

            self._global_step += 1
            if done:
                episode_costs.append(running_cost)
                running_cost = 0.0
                obs, _ = self.env.reset(seed=int(self._rng.integers(0, 2**31 - 1)))
                state = self.env.get_state()
                self._com_reference = state["com"].copy()
                self._torso_reference = state["torso_rotation"].copy()
                self.mpc.reset()
            else:
                obs = _next_obs

        if running_cost > 0.0 and not episode_costs:
            episode_costs.append(running_cost)

        rewards_arr = np.asarray(rewards, dtype=np.float64)
        costs_arr = np.asarray(costs, dtype=np.float64)
        values_arr = np.asarray(values, dtype=np.float64)
        cost_values_arr = np.asarray(cost_values, dtype=np.float64)
        dones_arr = np.asarray(dones, dtype=np.float64)
        reward_adv, reward_ret = _gae(rewards_arr, values_arr, dones_arr, cfg.gamma, cfg.gae_lambda)
        cost_adv, cost_ret = _gae(costs_arr, cost_values_arr, dones_arr, cfg.gamma, cfg.gae_lambda)
        return {
            "obs": np.asarray(observations, dtype=np.float32),
            "actions": np.asarray(actions, dtype=np.float32),
            "log_probs": np.asarray(log_probs, dtype=np.float64),
            "reward_adv": reward_adv,
            "reward_return": reward_ret,
            "cost_adv": cost_adv,
            "cost_return": cost_ret,
            "episode_cost": float(np.mean(episode_costs) if episode_costs else running_cost),
            "mean_reward": float(rewards_arr.mean()) if rewards_arr.size else 0.0,
            "mean_cost": float(costs_arr.mean()) if costs_arr.size else 0.0,
            "mean_intervention": float(np.mean(interventions)) if interventions else 0.0,
        }

    def update(self, batch: dict[str, np.ndarray]) -> dict[str, float]:
        """One constrained policy update; returns the metrics to log."""
        if batch["obs"].shape[0] == 0:
            return {"policy_loss": 0.0, "lagrange": self.lagrange.value}
        cfg = self.config
        obs = torch.as_tensor(batch["obs"], dtype=torch.float32)
        actions = torch.as_tensor(batch["actions"], dtype=torch.float32)
        old_log_probs = torch.as_tensor(batch["log_probs"], dtype=torch.float32)
        # Lagrangian advantage. Standardise the reward part only; the cost
        # advantage keeps its scale so lambda stays meaningful.
        reward_adv = torch.as_tensor(batch["reward_adv"], dtype=torch.float32)
        reward_adv = (reward_adv - reward_adv.mean()) / (reward_adv.std() + 1e-8)
        cost_adv = torch.as_tensor(batch["cost_adv"], dtype=torch.float32)
        advantage = reward_adv - self.lagrange.value * cost_adv
        reward_return = torch.as_tensor(batch["reward_return"], dtype=torch.float32)
        cost_return = torch.as_tensor(batch["cost_return"], dtype=torch.float32)

        n = obs.shape[0]
        batch_size = max(1, min(cfg.minibatch_size, n))
        last_loss = 0.0
        for _ in range(cfg.epochs_per_update):
            order = torch.randperm(n)
            for start in range(0, n, batch_size):
                index = order[start : start + batch_size]
                mean, value, cost_value = self.policy(obs[index])
                dist = self.policy.distribution(mean)
                log_prob = dist.log_prob(actions[index]).sum(-1)
                ratio = torch.exp(log_prob - old_log_probs[index])
                unclipped = ratio * advantage[index]
                clipped = torch.clamp(ratio, 1.0 - cfg.clip_range, 1.0 + cfg.clip_range) * advantage[index]
                policy_loss = -torch.min(unclipped, clipped).mean()
                value_loss = 0.5 * (value - reward_return[index]).pow(2).mean()
                cost_loss = 0.5 * (cost_value - cost_return[index]).pow(2).mean()
                entropy = dist.entropy().sum(-1).mean()
                loss = policy_loss + value_loss + cost_loss - 0.01 * entropy
                self._optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.policy.parameters(), 1.0)
                self._optimizer.step()
                last_loss = float(loss.detach())
        return {
            "policy_loss": last_loss,
            "lagrange": self.lagrange.value,
            "mean_reward": float(batch["mean_reward"]),
            "mean_cost": float(batch["mean_cost"]),
            "mean_intervention": float(batch["mean_intervention"]),
        }

    def evaluate(self, episodes: int = 10) -> dict[str, float]:
        """Deterministic evaluation: return, cost, and violation rate."""
        returns = []
        costs = []
        violations = 0
        for episode in range(episodes):
            obs, _ = self.env.reset(seed=self.config.seed + episode)
            state = self.env.get_state()
            com_ref = state["com"].copy()
            torso_ref = state["torso_rotation"].copy()
            self.mpc.reset()
            total_reward = 0.0
            total_cost = 0.0
            violated = False
            done = False
            while not done:
                obs_tensor = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
                with torch.no_grad():
                    mean, _, _ = self.policy(obs_tensor)
                residual = self.config.residual_scale * np.tanh(mean.squeeze(0).numpy())
                state = self.env.get_state()
                solution = self.mpc.solve(
                    state, {"com": com_ref + residual, "torso_orientation": torso_ref}
                )
                obs, reward, terminated, truncated, info, _intervention = self._track(solution)
                total_reward += float(reward)
                total_cost += float(info["cost"])
                violated = violated or float(info["cost"]) > 0.0
                done = bool(terminated or truncated)
            returns.append(total_reward)
            costs.append(total_cost)
            violations += int(violated)
        return {
            "return": float(np.mean(returns)) if returns else 0.0,
            "cost": float(np.mean(costs)) if costs else 0.0,
            "violation_rate": float(violations / episodes) if episodes else 0.0,
        }

    def train(self) -> None:
        """Main loop: collect, update the policy, update the multiplier, log."""
        cfg = self.config
        with RunLogger("safe_rl", root=Path(cfg.run_dir), metadata={"algorithm": cfg.algorithm, "seed": cfg.seed}) as logger:
            while self._global_step < cfg.total_steps:
                batch = self.collect_rollouts()
                metrics = self.update(batch)
                lagrange = self.lagrange.update(batch["episode_cost"])
                logger.log_scalars(
                    self._global_step,
                    policy_loss=metrics["policy_loss"],
                    lagrange=lagrange,
                    mean_reward=metrics["mean_reward"],
                    mean_cost=metrics["mean_cost"],
                    mean_intervention=metrics["mean_intervention"],
                    episode_cost=batch["episode_cost"],
                )
                logger.console.info(
                    "step %d  reward %.3f  cost %.3f  lambda %.3f",
                    self._global_step,
                    metrics["mean_reward"],
                    metrics["mean_cost"],
                    lagrange,
                )
                if cfg.checkpoint_every > 0 and self._global_step % cfg.checkpoint_every < cfg.steps_per_update:
                    self.save_checkpoint(self._global_step)

    def save_checkpoint(self, step: int) -> Path:
        """Write the policy weights and the Lagrange multiplier."""
        directory = Path(self.config.run_dir)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"policy_step_{step}.pt"
        torch.save(
            {
                "step": step,
                "policy": self.policy.state_dict(),
                "optimizer": self._optimizer.state_dict(),
                "lagrange": self.lagrange.value,
                "obs_dim": self._obs_dim,
                "act_dim": self._act_dim,
            },
            path,
        )
        return path

    def load_checkpoint(self, path: Path) -> None:
        """Restore a checkpoint written by :meth:`save_checkpoint`."""
        payload = torch.load(path, map_location="cpu", weights_only=False)
        if payload["obs_dim"] != self._obs_dim or payload["act_dim"] != self._act_dim:
            raise ValueError(
                f"checkpoint dims {(payload['obs_dim'], payload['act_dim'])} do not match "
                f"this trainer {(self._obs_dim, self._act_dim)}"
            )
        self.policy.load_state_dict(payload["policy"])
        self._optimizer.load_state_dict(payload["optimizer"])
        self.lagrange.value = float(payload["lagrange"])
        self._global_step = int(payload["step"])


def _gae(
    rewards: np.ndarray,
    values: np.ndarray,
    dones: np.ndarray,
    gamma: float,
    lam: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Generalised advantage estimation.

    ``dones[t] = 1`` means the transition out of ``t`` ended the episode, so
    the bootstrap from ``values[t+1]`` is dropped.
    """
    advantage = np.zeros_like(rewards)
    next_advantage = 0.0
    for t in reversed(range(rewards.size)):
        next_value = 0.0 if t == rewards.size - 1 else values[t + 1]
        mask = 1.0 - dones[t]
        delta = rewards[t] + gamma * next_value * mask - values[t]
        next_advantage = delta + gamma * lam * mask * next_advantage
        advantage[t] = next_advantage
    return advantage, advantage + values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Safe RL training for the industrial H1")
    parser.add_argument("--algorithm", default=SafeRLConfig.algorithm)
    parser.add_argument("--total-steps", type=int, default=SafeRLConfig.total_steps)
    parser.add_argument("--cost-budget", type=float, default=SafeRLConfig.cost_budget)
    parser.add_argument("--seed", type=int, default=SafeRLConfig.seed)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN_DIR)
    parser.add_argument("--no-cbf", action="store_true", help="disable the CBF safety filter")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = SafeRLConfig(
        algorithm=args.algorithm,
        total_steps=args.total_steps,
        cost_budget=args.cost_budget,
        seed=args.seed,
        run_dir=args.run_dir,
        use_cbf_filter=not args.no_cbf,
    )
    SafeRLTrainer(config).train()


if __name__ == "__main__":
    main()
