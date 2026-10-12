"""Late-swing touchdown residual on top of the deterministic walking controller.

The policy does not command torques. It adds a clipped centre-of-mass
residual to the reference the gait already built, and that reference still
goes through the MPC, the PD controller and the CBF.

Observation order is ``OBS_NAMES``. Index ``i`` is divided by ``obs_scale[i]``
before it enters the network, and the same scale is stored in the checkpoint.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

OBS_NAMES: tuple[str, ...] = (
    "com_x",
    "com_y",
    "com_vx",
    "com_vy",
    "capture_x",
    "capture_y",
    "capture_error_x",
    "capture_error_y",
    "margin_rear",
    "margin_front",
    "margin_inner",
    "margin_outer",
    "pelvis_roll",
    "pelvis_pitch",
    "stance_hip_roll",
    "stance_ankle",
    "swing_x",
    "swing_y",
    "swing_z",
    "swing_vz",
    "swing_progress",
    "time_since_activation",
    "cbf_intervention",
)

# Turns the raw observation into roughly unit-scale inputs. Saved with the policy.
DEFAULT_OBS_SCALE: tuple[float, ...] = (
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.2, 0.2,
    0.2, 0.2, 0.2, 0.2,
    0.5, 0.5, 0.5, 0.5,
    1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 100.0,
)


@dataclass
class TouchdownConfig:
    """One place for the touchdown task. Bounds, reward and phase live here."""

    residual_bound_x: float = 0.02
    residual_bound_y: float = 0.02
    late_swing_start: float = 0.65
    touchdown_height: float = 0.09
    touchdown_progress: float = 0.7
    touchdown_vz_min: float = -0.5
    touchdown_confirm_ticks: int = 2
    stabilize_seconds: float = 0.4
    fall_height: float = 0.72
    upright_height: float = 0.85
    max_abs_pitch: float = 0.6
    max_abs_roll: float = 0.6
    stable_abs_pitch: float = 0.35
    stable_abs_roll: float = 0.35
    stable_abs_vy: float = 0.08
    stable_abs_vx: float = 0.12
    fail_safe_limit: int = 15
    max_rl_seconds: float = 4.0
    control_dt: float = 0.02
    seed: int = 0
    episodes: int = 4
    steps_per_update: int = 256
    minibatch_size: int = 64
    epochs_per_update: int = 4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    learning_rate: float = 3e-4
    # Mean CBF cost per RL step, not the sum over the episode. One unit is a
    # full 40 N·m trust-region correction. Observed steps sit near 0–1, so a
    # budget of 5 would never bind. 0.5 allows an average correction of 20 N·m
    # before the multiplier increases.
    cost_budget: float = 0.5
    lagrange_init: float = 1.0
    lagrange_lr: float = 0.035
    lagrange_max: float = 100.0
    randomize: bool = False
    disturbance_xy: float = 0.0
    # Reward weights. The terminal bonuses are added once.
    w_alive: float = 0.05
    w_margin: float = 1.0
    w_capture: float = 0.5
    w_velocity: float = 1.0
    w_residual: float = 0.5
    w_posture: float = 0.2
    w_touchdown: float = 1.0
    w_stable: float = 5.0
    cbf_cost_scale: float = 40.0

    def bounds(self) -> np.ndarray:
        return np.array([self.residual_bound_x, self.residual_bound_y], dtype=float)

    def obs_scale(self) -> np.ndarray:
        scale = np.asarray(DEFAULT_OBS_SCALE, dtype=float)
        if scale.shape != (len(OBS_NAMES),):
            raise ValueError("observation scale does not match OBS_NAMES")
        return scale


def normalize_observation(raw: np.ndarray, scale: np.ndarray) -> np.ndarray:
    """Divide by the saved scale and replace non-finite entries with zero."""
    values = np.asarray(raw, dtype=float).reshape(-1)
    denom = np.asarray(scale, dtype=float).reshape(-1)
    out = np.zeros(len(OBS_NAMES), dtype=np.float32)
    n = min(values.size, out.size, denom.size)
    with np.errstate(divide="ignore", invalid="ignore"):
        scaled = values[:n] / np.maximum(denom[:n], 1e-6)
    finite = np.isfinite(scaled)
    out[:n][finite] = scaled[finite].astype(np.float32)
    return out


def scale_residual(action: np.ndarray, bounds: np.ndarray) -> np.ndarray:
    """Map an unbounded network action through tanh into ``±bounds``."""
    raw = np.asarray(action, dtype=float).reshape(-1)
    limit = np.asarray(bounds, dtype=float).reshape(-1)
    out = np.zeros(2, dtype=float)
    n = min(raw.size, out.size, limit.size)
    out[:n] = np.tanh(raw[:n]) * limit[:n]
    return out


def apply_com_residual(
    reference: dict,
    residual: np.ndarray,
    polygon: np.ndarray | None,
) -> tuple[dict, np.ndarray]:
    """Add ``residual`` to the centre-of-mass reference and keep it inside the sole box.

    The returned residual is what was actually applied after that clamp, which
    can be smaller than the request when the sole face is closer than the bound.
    """
    updated = {key: np.array(value, copy=True) for key, value in reference.items()}
    com = np.asarray(updated["com"], dtype=float)
    delta = np.asarray(residual, dtype=float).reshape(2)
    if com.ndim == 1:
        base = com[:2].copy()
        shifted = base + delta
        if polygon is not None and np.asarray(polygon).size >= 4:
            box = np.asarray(polygon, dtype=float).reshape(-1, 2)
            shifted = np.clip(shifted, box.min(axis=0), box.max(axis=0))
        com[:2] = shifted
        updated["com"] = com
        return updated, shifted - base
    base = com[:, :2].copy()
    shifted = base + delta.reshape(1, 2)
    if polygon is not None and np.asarray(polygon).size >= 4:
        box = np.asarray(polygon, dtype=float).reshape(-1, 2)
        shifted = np.clip(shifted, box.min(axis=0), box.max(axis=0))
    com[:, 0] = shifted[:, 0]
    com[:, 1] = shifted[:, 1]
    updated["com"] = com
    return updated, shifted[0] - base[0]


def touchdown_rl_active(
    *,
    state_name: str,
    progress: float,
    seconds_since_touchdown: float | None,
    stable: bool,
    config: TouchdownConfig,
) -> bool:
    """Late swing, and a short window after touchdown. Off once the step is stable."""
    if stable:
        return False
    swinging = state_name in ("RIGHT_SWING", "LEFT_SWING") and float(progress) >= config.late_swing_start
    recovering = (
        seconds_since_touchdown is not None
        and float(seconds_since_touchdown) < config.stabilize_seconds
        and state_name in ("RIGHT_SWING", "LEFT_SWING", "DOUBLE_SUPPORT", "STAND")
    )
    return bool(swinging or recovering)


def detect_touchdown(
    *,
    progress: float,
    foot_z: float,
    foot_vz: float,
    in_contact: bool,
    config: TouchdownConfig,
) -> bool:
    """True when the swing foot is at the ground and no longer dropping through it.

    Contact with the floor is sufficient together with a late swing phase.
    Without a contact reading, the foot must be below ``touchdown_height`` and
    its vertical speed must have risen above ``touchdown_vz_min``.
    """
    late = float(progress) >= config.touchdown_progress
    if not late:
        return False
    if in_contact and float(foot_vz) >= config.touchdown_vz_min:
        return True
    return bool(float(foot_z) <= config.touchdown_height and float(foot_vz) >= config.touchdown_vz_min)


def stabilization_ok(
    *,
    com_z: float,
    vx: float,
    vy: float,
    margins: np.ndarray,
    pitch: float,
    roll: float,
    config: TouchdownConfig,
) -> bool:
    """Upright, slow, and inside the support box."""
    inside = bool(np.all(np.asarray(margins, dtype=float) > 0.0))
    return bool(
        inside
        and float(com_z) >= config.upright_height
        and abs(float(vy)) <= config.stable_abs_vy
        and abs(float(vx)) <= config.stable_abs_vx
        and abs(float(pitch)) <= config.stable_abs_pitch
        and abs(float(roll)) <= config.stable_abs_roll
    )


def failure_reason(
    *,
    com_z: float,
    pitch: float,
    roll: float,
    finite: bool,
    fail_safe_run: int,
    elapsed: float,
    config: TouchdownConfig,
) -> str:
    """Empty string while the episode may continue."""
    if not finite:
        return "non_finite"
    if float(com_z) < config.fall_height:
        return "fallen"
    if abs(float(pitch)) > config.max_abs_pitch or abs(float(roll)) > config.max_abs_roll:
        return "tipped"
    if int(fail_safe_run) >= config.fail_safe_limit:
        return "cbf_infeasible"
    if float(elapsed) > config.max_rl_seconds:
        return "timeout"
    return ""


def reward_components(
    raw: np.ndarray,
    residual: np.ndarray,
    *,
    touchdown_event: bool,
    stable_event: bool,
    config: TouchdownConfig,
) -> dict[str, float]:
    """Named pieces of the touchdown reward. Their sum is the scalar reward.

    ``raw`` is the un-normalised observation in ``OBS_NAMES`` order.
    """
    obs = np.asarray(raw, dtype=float).reshape(-1)
    margins = obs[8:12]
    capture = obs[6:8]
    velocity = obs[2:4]
    posture = obs[12:14]
    delta = np.asarray(residual, dtype=float).reshape(2)
    bounds = np.maximum(config.bounds(), 1e-6)
    parts = {
        "alive": config.w_alive,
        "margin": config.w_margin * float(np.mean(np.clip(margins, -0.2, 0.2))),
        "capture": config.w_capture * float(-np.mean(np.abs(capture))),
        "velocity": config.w_velocity * float(-(velocity[0] ** 2 + 4.0 * velocity[1] ** 2)),
        "residual": config.w_residual * float(-np.mean((delta / bounds) ** 2)),
        "posture": config.w_posture * float(-(posture[0] ** 2 + posture[1] ** 2)),
        "touchdown": config.w_touchdown if touchdown_event else 0.0,
        "stable": config.w_stable if stable_event else 0.0,
    }
    parts["total"] = float(sum(parts.values()))
    return parts


def cbf_cost(intervention: float, status: str, config: TouchdownConfig) -> float:
    """Soft cost. A full trust-region correction costs 1. Fail-safe costs 1 extra."""
    scale = max(float(config.cbf_cost_scale), 1e-6)
    cost = abs(float(intervention)) / scale
    if str(status) == "fail_safe":
        cost += 1.0
    return float(cost)


def foot_in_contact(env, side: str) -> bool:
    """True when a geom of ``side`` is touching something that is not that same foot."""
    import mujoco

    data = env.data
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        if float(contact.dist) > 1e-3:
            continue
        names = []
        for geom_id in (int(contact.geom1), int(contact.geom2)):
            names.append(mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "")
        own = [name.startswith(f"{side}_") for name in names]
        if any(own) and not all(own):
            return True
    return False


def _pelvis_angles(rotation: np.ndarray) -> tuple[float, float]:
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    roll = float(np.arctan2(matrix[2, 1], matrix[2, 2]))
    pitch = float(np.arctan2(-matrix[2, 0], np.hypot(matrix[2, 1], matrix[2, 2])))
    return roll, pitch


def build_touchdown_observation(
    *,
    com: np.ndarray,
    velocity: np.ndarray,
    reference_xy: np.ndarray,
    barriers: dict[str, float],
    rotation: np.ndarray,
    stance_side: str,
    stance_hip_roll: float,
    stance_ankle: float,
    swing_xyz: np.ndarray,
    swing_vz: float,
    progress: float,
    time_since_activation: float,
    intervention: float,
) -> np.ndarray:
    """Fixed-length raw observation. Non-finite inputs become zero."""
    position = np.asarray(com, dtype=float).reshape(-1)
    vel = np.asarray(velocity, dtype=float).reshape(-1)
    height = max(float(position[2]) if position.size > 2 else 0.9, 0.2)
    omega = float(np.sqrt(9.81 / height))
    capture = position[:2] + vel[:2] / omega
    ref = np.asarray(reference_xy, dtype=float).reshape(2)
    error = capture - ref
    rear = float(barriers.get("support_x_lower", 0.0))
    front = float(barriers.get("support_x_upper", 0.0))
    y_lower = float(barriers.get("support_y_lower", 0.0))
    y_upper = float(barriers.get("support_y_upper", 0.0))
    if stance_side == "right":
        inner, outer = y_upper, y_lower
    else:
        inner, outer = y_lower, y_upper
    roll, pitch = _pelvis_angles(rotation)
    foot = np.asarray(swing_xyz, dtype=float).reshape(3)
    raw = np.array(
        [
            position[0], position[1], vel[0], vel[1],
            capture[0], capture[1], error[0], error[1],
            rear, front, inner, outer,
            roll, pitch,
            float(stance_hip_roll), float(stance_ankle),
            foot[0], foot[1], foot[2], float(swing_vz),
            float(progress), float(time_since_activation), float(intervention),
        ],
        dtype=np.float64,
    )
    raw[~np.isfinite(raw)] = 0.0
    return raw


class TouchdownAgent:
    """Optional late-swing residual. With no checkpoint it leaves the reference alone."""

    def __init__(self, config: TouchdownConfig | None = None, deterministic: bool = True) -> None:
        self.config = config or TouchdownConfig()
        self.deterministic = bool(deterministic)
        self.policy = None
        self._obs_scale = self.config.obs_scale()
        self.reset_episode()

    def reset_episode(self) -> None:
        self._prev_foot_z: float | None = None
        self._active_since: float | None = None
        self._touchdown_since: float | None = None
        self._confirm = 0
        self._pending_touchdown = False
        self._stable_time = 0.0
        self._fail_safe_run = 0
        self._touchdown = False
        self._stable = False
        self._reason = ""
        self._elapsed = 0.0
        self.last: dict = {"active": False, "residual": np.zeros(2), "action": np.zeros(2), "obs": np.zeros(len(OBS_NAMES))}

    @property
    def touchdown(self) -> bool:
        return self._touchdown

    @property
    def stable(self) -> bool:
        return self._stable

    @property
    def reason(self) -> str:
        return self._reason

    def load(self, path: Path) -> None:
        """Load a touchdown checkpoint. Inference uses the Gaussian mean."""
        from rl.safe_rl_training import ResidualGaussianPolicy

        import torch

        payload = torch.load(Path(path), map_location="cpu", weights_only=False)
        if int(payload.get("act_dim", 0)) != 2 or int(payload.get("obs_dim", 0)) != len(OBS_NAMES):
            raise ValueError(
                f"touchdown checkpoint must be obs {len(OBS_NAMES)} act 2, "
                f"got {(payload.get('obs_dim'), payload.get('act_dim'))}"
            )
        if "obs_scale" in payload:
            self._obs_scale = np.asarray(payload["obs_scale"], dtype=float)
        if "residual_bound" in payload:
            bound = np.asarray(payload["residual_bound"], dtype=float).reshape(2)
            self.config.residual_bound_x = float(bound[0])
            self.config.residual_bound_y = float(bound[1])
        self.policy = ResidualGaussianPolicy(len(OBS_NAMES), 2)
        self.policy.load_state_dict(payload["policy"])
        self.policy.eval()

    def save(self, path: Path, *, seed: int, lagrange: float, extra: dict | None = None) -> Path:
        import torch

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "policy": None if self.policy is None else self.policy.state_dict(),
            "obs_dim": len(OBS_NAMES),
            "act_dim": 2,
            "obs_names": OBS_NAMES,
            "obs_scale": self._obs_scale,
            "residual_bound": self.config.bounds(),
            "config": asdict(self.config),
            "seed": int(seed),
            "lagrange": float(lagrange),
        }
        if extra:
            payload.update(extra)
        torch.save(payload, path)
        return path

    def correct(self, env, reference: dict, gait, cbf, command, swing: str | None, state: dict, rows: list[dict], polygon: np.ndarray) -> tuple[dict, dict]:
        """Return the reference to send to the MPC, and the log fields for this tick."""
        started = time.perf_counter()
        info = self._observe(env, reference, gait, cbf, command, swing, state, rows, polygon)
        active = bool(info["active"]) and self.policy is not None
        residual = np.zeros(2)
        action = np.zeros(2)
        log_prob = 0.0
        value = 0.0
        cost_value = 0.0
        if active:
            action, log_prob, value, cost_value = self._infer(info["normalized"])
            residual = scale_residual(action, self.config.bounds())
            reference, residual = apply_com_residual(reference, residual, polygon)
        self.last = {
            "active": active,
            "residual": residual,
            "action": np.asarray(action, dtype=float).reshape(2),
            "obs": info["raw"],
            "normalized": info["normalized"],
            "log_prob": float(log_prob),
            "value": float(value),
            "cost_value": float(cost_value),
            "progress": info["progress"],
            "contact": info["contact"],
            "foot_z": info["foot_z"],
            "foot_vz": info["foot_vz"],
        }
        blank = {
            "rl_active": float(active),
            "rl_action": self.last["action"].copy(),
            "rl_residual": residual.copy(),
            "rl_inference_s": float(time.perf_counter() - started),
            "touchdown_detected": float(self._touchdown),
            "stable_double_support": float(self._stable),
        }
        return reference, blank

    def update_after_step(self, row: dict) -> dict[str, float]:
        """Advance touchdown and stability counters from the tick that just ran."""
        cfg = self.config
        if self._active_since is None:
            return {"total": 0.0, "cbf_cost": 0.0}
        self._elapsed += cfg.control_dt
        progress = float(self.last.get("progress", 0.0))
        detected = detect_touchdown(
            progress=progress,
            foot_z=float(self.last.get("foot_z", 1.0)),
            foot_vz=float(self.last.get("foot_vz", 0.0)),
            in_contact=bool(self.last.get("contact", False)),
            config=cfg,
        )
        self._confirm = self._confirm + 1 if detected else 0
        if self._confirm >= cfg.touchdown_confirm_ticks:
            self._pending_touchdown = True
        in_swing = str(row.get("walk_state", "")) in ("RIGHT_SWING", "LEFT_SWING")
        # The landing clock starts when the gait leaves the swing, so the
        # residual stays on through the hand-off rather than expiring in the air.
        if self._pending_touchdown and self._touchdown_since is None and not in_swing:
            self._touchdown = True
            self._touchdown_since = 0.0
        elif self._touchdown_since is not None:
            self._touchdown_since += cfg.control_dt
        status = str(row.get("cbf_status", "optimal"))
        self._fail_safe_run = self._fail_safe_run + 1 if status == "fail_safe" else 0
        com = np.asarray(row["com"], dtype=float).reshape(-1)
        vel = np.asarray(row["com_velocity"], dtype=float).reshape(-1)
        margins = np.array(
            [
                float(row.get("support_x_lower", 0.0)),
                float(row.get("support_x_upper", 0.0)),
                float(row.get("support_y_lower", 0.0)),
                float(row.get("support_y_upper", 0.0)),
            ]
        )
        pitch = float(row.get("pelvis_pitch", 0.0))
        roll = float(row.get("pelvis_roll", 0.0))
        became_stable = False
        if self._touchdown and not self._stable:
            if stabilization_ok(
                com_z=float(com[2]), vx=float(vel[0]), vy=float(vel[1]),
                margins=margins, pitch=pitch, roll=roll, config=cfg,
            ):
                self._stable_time += cfg.control_dt
            else:
                self._stable_time = 0.0
            if self._stable_time >= cfg.stabilize_seconds:
                self._stable = True
                became_stable = True
        finite = bool(np.all(np.isfinite(com)) and np.all(np.isfinite(vel)))
        if not self._reason:
            self._reason = failure_reason(
                com_z=float(com[2]), pitch=pitch, roll=roll, finite=finite,
                fail_safe_run=self._fail_safe_run, elapsed=self._elapsed, config=cfg,
            )
        parts = reward_components(
            self.last.get("obs", np.zeros(len(OBS_NAMES))),
            self.last.get("residual", np.zeros(2)),
            touchdown_event=self._touchdown_since == 0.0,
            stable_event=became_stable,
            config=cfg,
        )
        parts["cbf_cost"] = cbf_cost(float(row.get("intervention", 0.0)), status, cfg)
        return parts

    def _infer(self, normalized: np.ndarray):
        import torch

        obs = torch.as_tensor(normalized, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            mean, value, cost_value = self.policy(obs)
            dist = self.policy.distribution(mean)
            action = mean if self.deterministic else dist.sample()
            log_prob = dist.log_prob(action).sum(-1)
        return (
            action.squeeze(0).numpy(),
            float(log_prob),
            float(value.squeeze(0)),
            float(cost_value.squeeze(0)),
        )

    def _observe(self, env, reference, gait, cbf, command, swing, state, rows, polygon) -> dict:
        from main_demo import _ankle_pose

        state_name = command.state.value
        duration = float(gait.config.step_duration)
        progress = 0.0
        if state_name in ("RIGHT_SWING", "LEFT_SWING") and duration > 0.0:
            progress = float(np.clip(gait._t_state / duration, 0.0, 1.0))
        elif self._touchdown:
            progress = 1.0
        side = swing or ("right" if state_name == "RIGHT_SWING" else "left" if state_name == "LEFT_SWING" else "")
        if not side:
            side = "right" if getattr(gait, "_next_swing", "left") == "left" else "left"
        foot, rotation = _ankle_pose(env, side if side in ("left", "right") else "right")
        if self._prev_foot_z is None:
            vz = 0.0
        else:
            vz = (float(foot[2]) - self._prev_foot_z) / max(self.config.control_dt, 1e-6)
        self._prev_foot_z = float(foot[2])
        now = float(state["time"]) if "time" in state else float(env.data.time)
        seconds_since = None if self._touchdown_since is None else float(self._touchdown_since)
        active = touchdown_rl_active(
            state_name=state_name,
            progress=progress,
            seconds_since_touchdown=seconds_since,
            stable=self._stable,
            config=self.config,
        )
        if active and self._active_since is None:
            self._active_since = now
        since = 0.0 if self._active_since is None else max(0.0, now - self._active_since)
        com = np.asarray(state["com"], dtype=float)
        if cbf is not None and polygon is not None:
            cbf.set_support_polygon(polygon)
            barriers = cbf.com_barrier_values(com)
        else:
            barriers = {}
        ref = np.asarray(reference["com"], dtype=float)
        ref_xy = ref[0, :2] if ref.ndim == 2 else ref[:2]
        stance_side = "left" if side == "right" else "right"
        names = list(getattr(env, "joint_names", []))
        q = np.asarray(state["joint_qpos"], dtype=float)

        def _joint(joint_name: str) -> float:
            if joint_name not in names:
                return 0.0
            return float(q[names.index(joint_name)])

        previous_intervention = float(rows[-1]["intervention"]) if rows else 0.0
        raw = build_touchdown_observation(
            com=com,
            velocity=np.asarray(state["com_velocity"], dtype=float),
            reference_xy=ref_xy,
            barriers=barriers,
            rotation=np.asarray(state["base_rotation"], dtype=float),
            stance_side=stance_side,
            stance_hip_roll=_joint(f"{stance_side}_hip_roll"),
            stance_ankle=_joint(f"{stance_side}_ankle"),
            swing_xyz=foot,
            swing_vz=vz,
            progress=progress,
            time_since_activation=since,
            intervention=previous_intervention,
        )
        return {
            "raw": raw,
            "normalized": normalize_observation(raw, self._obs_scale),
            "active": active,
            "progress": progress,
            "contact": foot_in_contact(env, side) if side in ("left", "right") else False,
            "foot_z": float(foot[2]),
            "foot_vz": float(vz),
        }


def summarize_trial(rows: list[dict], config: TouchdownConfig | None = None) -> dict[str, float]:
    """One-step outcome used for the baseline versus residual comparison.

    Touchdown is the first double-support row after a swing. Stable means the
    following window stays upright, slow and inside the support box.
    """
    cfg = config or TouchdownConfig()
    empty = {
        "touchdown": 0.0,
        "stable": 0.0,
        "fell": 0.0,
        "time_to_stable": -1.0,
        "com_error_x": 0.0,
        "com_error_y": 0.0,
        "capture_margin": 0.0,
        "min_support_margin": 0.0,
        "vy_at_touchdown": 0.0,
        "peak_abs_vy": 0.0,
        "mean_intervention": 0.0,
        "peak_torque": 0.0,
        "mean_residual": 0.0,
    }
    if not rows:
        return empty
    swing_seen = False
    touch_index = None
    for index, row in enumerate(rows):
        name = str(row.get("walk_state", ""))
        if name in ("RIGHT_SWING", "LEFT_SWING"):
            swing_seen = True
        if swing_seen and name == "DOUBLE_SUPPORT" and touch_index is None:
            touch_index = index
    fell = any(float(np.asarray(row["com"]).reshape(-1)[2]) < cfg.fall_height for row in rows)
    peak_vy = max(abs(float(np.asarray(row["com_velocity"]).reshape(-1)[1])) for row in rows)
    interventions = [float(row.get("intervention", 0.0)) for row in rows]
    residuals = [float(np.linalg.norm(np.asarray(row.get("rl_residual", [0.0, 0.0]), dtype=float))) for row in rows]
    peak_torque = 0.0
    for row in rows:
        if "tau_safe" in row:
            peak_torque = max(peak_torque, float(np.max(np.abs(row["tau_safe"]))))
    result = dict(empty)
    result["fell"] = float(fell)
    result["peak_abs_vy"] = float(peak_vy)
    result["mean_intervention"] = float(np.mean(interventions))
    result["peak_torque"] = float(peak_torque)
    result["mean_residual"] = float(np.mean(residuals))
    if touch_index is None:
        return result
    touch = rows[touch_index]
    result["touchdown"] = 1.0
    error = np.asarray(touch.get("com_error", [0.0, 0.0, 0.0]), dtype=float).reshape(-1)
    result["com_error_x"] = float(error[0])
    result["com_error_y"] = float(error[1])
    result["vy_at_touchdown"] = float(np.asarray(touch["com_velocity"]).reshape(-1)[1])
    margins = []
    for key in ("support_x_lower", "support_x_upper", "support_y_lower", "support_y_upper"):
        if key in touch:
            margins.append(float(touch[key]))
    capture_keys = (
        "capture_margin_x_lower",
        "capture_margin_x_upper",
        "capture_margin_y_lower",
        "capture_margin_y_upper",
    )
    capture = [float(touch[key]) for key in capture_keys if key in touch]
    result["min_support_margin"] = float(min(margins)) if margins else 0.0
    result["capture_margin"] = float(min(capture)) if capture else 0.0
    window = max(1, int(round(cfg.stabilize_seconds / max(cfg.control_dt, 1e-6))))
    held = 0
    for row in rows[touch_index : touch_index + window]:
        com = np.asarray(row["com"], dtype=float).reshape(-1)
        vel = np.asarray(row["com_velocity"], dtype=float).reshape(-1)
        faces = np.array([float(row.get(key, -1.0)) for key in (
            "support_x_lower", "support_x_upper", "support_y_lower", "support_y_upper"
        )])
        if stabilization_ok(
            com_z=float(com[2]), vx=float(vel[0]), vy=float(vel[1]),
            margins=faces, pitch=float(row.get("pelvis_pitch", 0.0)),
            roll=float(row.get("pelvis_roll", 0.0)), config=cfg,
        ):
            held += 1
        else:
            held = 0
        if held >= window:
            result["stable"] = 1.0
            result["time_to_stable"] = float(row["time"]) - float(touch["time"])
            break
    return result
