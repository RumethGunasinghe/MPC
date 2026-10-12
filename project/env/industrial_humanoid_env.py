"""MuJoCo environment for the Unitree H1 in an industrial workcell.

Wraps the H1 MJCF model in a Gymnasium-style API so the MPC, CBF filter and
safe RL layers can all drive the same simulation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from controllers.pd_controller import PDController, PDGains

try:
    import mujoco
except ImportError:  # pragma: no cover - keeps the scaffold importable
    mujoco = None

try:
    import gymnasium as gym
except ImportError:  # pragma: no cover - spaces are optional
    gym = None

# Subclass gymnasium.Env when it is installed so reset/step/spaces match the
# Gymnasium contract. Without the package the class is a plain object and the
# spaces are simply left unset.
_EnvBase = gym.Env if gym is not None else object

DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "unitree_h1" / "scene.xml"

# Joint-name -> limb-group mapping for the PD gains lives in
# controllers.pd_controller, which owns the gain schedule.


@dataclass
class EnvConfig:
    """Static configuration for the industrial humanoid task."""

    model_path: Path = DEFAULT_MODEL_PATH
    control_dt: float = 0.02  # 50 Hz outer control loop
    sim_dt: float = 0.002  # 500 Hz MuJoCo integration step
    episode_seconds: float = 20.0
    render_mode: str | None = None  # None | "human" | "rgb_array"

    # Task definition: pick a payload from the conveyor, place it on the pallet.
    payload_mass_range: tuple[float, float] = (0.0, 15.0)
    pick_position: tuple[float, float, float] = (0.6, 0.0, 0.9)
    place_position: tuple[float, float, float] = (0.4, -0.7, 0.6)

    # Safety limits surfaced to the CBF filter.
    min_com_support_margin: float = 0.05  # metres
    max_joint_velocity: float = 8.0  # rad/s
    keepout_spheres: list[tuple[float, float, float, float]] = field(default_factory=list)

    # Episode termination: the H1 stands ~1.0 m at the pelvis when upright.
    min_base_height: float = 0.6  # metres, below this the robot has fallen
    min_upright: float = 0.5  # cos(angle) between torso z-axis and world z

    # Reset randomisation
    reset_joint_noise: float = 0.02  # rad, uniform perturbation on the home pose
    reset_target_noise: float = 0.05  # metres, jitter on pick / place targets

    # Reward / cost shaping
    w_task: float = 1.0
    w_effort: float = 1e-4
    w_posture: float = 1e-2
    w_upright: float = 0.5
    cost_scale: float = 1.0

    # Names looked up in the MJCF; missing names degrade gracefully.
    payload_body: str = "payload"
    grasp_distance: float = 0.08  # metres, hand-to-payload distance that grasps
    # Robot root body. Left as None the loader picks the free-floating body with
    # the heaviest subtree, which is the humanoid rather than a loose payload.
    root_body: str | None = None


class IndustrialHumanoidEnv(_EnvBase):
    """Gymnasium-compatible MuJoCo environment for the Unitree H1.

    Observation layout (concatenated, all float32):
        joint_qpos      actuated joint positions
        joint_qvel      actuated joint velocities
        base_quat       floating-base orientation (w, x, y, z)
        base_linvel     floating-base linear velocity
        base_angvel     floating-base angular velocity
        com             centre-of-mass position (world frame)
        com_velocity    centre-of-mass linear velocity
        ctrl            actuator commands held from the previous step
        actuator_force  actuator forces realised by the last integration step
        act             internal actuator activations (only if the model has any)
        payload         estimated payload mass and grasp state
        task_phase      one-hot encoding of the pick / carry / place phase

    Actions are joint position targets consumed by the low-level PD loop, which
    runs at ``sim_dt`` while the environment steps at ``control_dt``.
    """

    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, config: EnvConfig | None = None, gains: PDGains | None = None) -> None:
        self.config = config or EnvConfig()
        self.render_mode = self.config.render_mode
        if gym is not None:
            super().__init__()
        if mujoco is None:  # pragma: no cover - import guard
            raise ImportError(
                "mujoco is required to run IndustrialHumanoidEnv; "
                "install it with `pip install -r requirements.txt`"
            )

        self.model = self._load_model(self.config.model_path)
        # The MJCF ships its own timestep; the config is authoritative so that
        # control_dt / sim_dt stays an exact integer ratio.
        self.model.opt.timestep = self.config.sim_dt
        self.data = mujoco.MjData(self.model)

        ratio = self.config.control_dt / self.config.sim_dt
        self._n_substeps = int(round(ratio))
        # MuJoCo advances one opt.timestep per mj_step. A non-integer ratio
        # would make the outer control period drift away from control_dt.
        if self._n_substeps < 1 or abs(ratio - self._n_substeps) > 1e-6:
            raise ValueError(
                f"control_dt ({self.config.control_dt}) must be a positive integer "
                f"multiple of sim_dt ({self.config.sim_dt})"
            )

        self._viewer = None
        self._renderer = None
        self._step_count = 0
        self._max_steps = max(1, round(self.config.episode_seconds / self.config.control_dt))
        self._rng = np.random.default_rng()

        self._gains = gains or PDGains()
        self._index_model()
        # Passing the joint names lets the controller assign each joint the gains
        # of its limb group (stiff legs, compliant arms).
        self._pd = PDController(gains=self._gains, joint_names=self.joint_names)

        # Task state, (re)initialised by reset().
        self._payload_mass = 0.0
        self._grasped = False
        self._phase = 0  # 0 = reach/pick, 1 = carry, 2 = place
        self._pick_target = np.asarray(self.config.pick_position, dtype=float)
        self._place_target = np.asarray(self.config.place_position, dtype=float)
        self._external_wrench_expiry: dict[int, int] = {}
        # MuJoCo zeroes xfrc_applied at the end of every mj_step, so a timed
        # push has to be written again before each physics step.
        self._external_wrenches: dict[int, np.ndarray] = {}

        mujoco.mj_forward(self.model, self.data)
        self._obs_dim = int(self.get_observation().size)
        if gym is not None:
            # Finite bounds: Gymnasium and Stable-Baselines3 reject ±inf boxes.
            # 1e4 covers joint state, CoM and actuator signals for this robot.
            bound = np.float32(1e4)
            self.observation_space = gym.spaces.Box(
                low=-bound, high=bound, shape=(self._obs_dim,), dtype=np.float32
            )
            self.action_space = gym.spaces.Box(
                low=self._action_low.astype(np.float32),
                high=self._action_high.astype(np.float32),
                dtype=np.float32,
            )

    # ------------------------------------------------------------------
    # Model loading and introspection
    # ------------------------------------------------------------------
    @staticmethod
    def _load_model(model_path: Path) -> Any:
        """Compile the MJCF at ``model_path`` into an ``MjModel``."""
        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError(
                f"MJCF scene not found at {path}. Install the Unitree H1 description "
                "into models/unitree_h1/ (see the README in that folder)."
            )
        # from_xml_path resolves <include> and mesh assets relative to the file.
        return mujoco.MjModel.from_xml_path(str(path))

    def _index_model(self) -> None:
        """Cache the joint / actuator / body indices the control stack needs.

        Everything is discovered from the compiled model rather than hard-coded,
        so a modified workcell scene (extra props, a gripper, a fixed base) still
        loads without touching this file.
        """
        model = self.model

        # Robot root and floating base. The workcell scene contains other free
        # joints (the payload), so the base joint is resolved explicitly instead
        # of assuming it is the first joint in the model.
        self._root_body_id, base_joint_id = self._resolve_root_body()
        self._has_free_base = base_joint_id >= 0
        self._base_qpos_adr = int(model.jnt_qposadr[base_joint_id]) if self._has_free_base else -1
        self._base_dof_adr = int(model.jnt_dofadr[base_joint_id]) if self._has_free_base else -1
        # subtree_com of the root body is the robot CoM and therefore excludes
        # static scene geometry (floor, conveyor, pallet) and the free payload.
        self._robot_body_ids = self._subtree_body_ids(self._root_body_id)

        # Actuator -> joint maps. Each H1 actuator drives exactly one hinge.
        self._actuator_joint_ids = np.full(model.nu, -1, dtype=int)
        for i in range(model.nu):
            if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
                self._actuator_joint_ids[i] = int(model.actuator_trnid[i, 0])
        if np.any(self._actuator_joint_ids < 0):
            raise ValueError(
                "IndustrialHumanoidEnv expects joint-transmission actuators; "
                "the scene contains tendon or site actuators."
            )
        self._actuator_qpos_adr = model.jnt_qposadr[self._actuator_joint_ids].astype(int)
        self._actuator_dof_adr = model.jnt_dofadr[self._actuator_joint_ids].astype(int)
        self.joint_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) or f"joint_{j}"
            for j in self._actuator_joint_ids
        ]

        # Position targets are clipped to the joint range where one is defined.
        limited = model.jnt_limited[self._actuator_joint_ids].astype(bool)
        ranges = model.jnt_range[self._actuator_joint_ids]
        self._action_low = np.where(limited, ranges[:, 0], -np.pi)
        self._action_high = np.where(limited, ranges[:, 1], np.pi)
        self.joint_limits = np.column_stack((self._action_low, self._action_high))

        # Control range: torque bounds for motor actuators, position bounds for
        # servo actuators. mjBIAS_AFFINE marks the built-in position/velocity
        # servos, in which case MuJoCo closes the loop and our PD layer is idle.
        ctrl_limited = model.actuator_ctrllimited.astype(bool)
        ctrl_range = model.actuator_ctrlrange
        default_limit = self._gains.torque_limit
        self._ctrl_low = np.where(ctrl_limited, ctrl_range[:, 0], -default_limit)
        self._ctrl_high = np.where(ctrl_limited, ctrl_range[:, 1], default_limit)
        self._position_actuators = bool(
            model.nu > 0
            and np.all(model.actuator_biastype == mujoco.mjtBias.mjBIAS_AFFINE)
        )

        # Bodies used by the task and the safety costs.
        self._payload_body_id = self._body_id(self.config.payload_body)
        self._nominal_payload_inertia = (
            model.body_inertia[self._payload_body_id].copy() if self._payload_body_id >= 0 else None
        )
        self._nominal_payload_mass = (
            float(model.body_mass[self._payload_body_id]) if self._payload_body_id >= 0 else 0.0
        )
        self._foot_body_ids = self._match_body_ids(("ankle", "foot"))
        # The bare menagerie H1 ends each arm at an elbow link, so a scene
        # without hands / a gripper leaves this empty and _hand_position falls
        # back to the CoM.
        self._hand_body_ids = self._match_body_ids(("hand", "wrist"))

        # Body whose orientation the MPC regulates. Falls back to the root body
        # for models that do not name a separate torso link.
        torso_matches = self._match_body_ids(("torso", "chest", "trunk"))
        self._torso_body_id = int(torso_matches[0]) if torso_matches.size else self._root_body_id

        # Home pose: prefer a keyframe from the MJCF, else the compiled qpos0.
        self._home_qpos = model.key_qpos[0].copy() if model.nkey > 0 else model.qpos0.copy()
        self._nominal_joint_qpos = self._home_qpos[self._actuator_qpos_adr].copy()

    def _resolve_root_body(self) -> tuple[int, int]:
        """Return ``(root_body_id, base_free_joint_id)`` for the robot.

        ``base_free_joint_id`` is -1 for a fixed-base model. When
        ``config.root_body`` is unset the heaviest free-floating subtree wins,
        which distinguishes the humanoid from loose props in the same scene.
        """
        model = self.model
        free_joints = [
            j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE
        ]

        if self.config.root_body is not None:
            root_body_id = self._body_id(self.config.root_body)
            if root_body_id < 0:
                raise ValueError(
                    f"root_body {self.config.root_body!r} is not present in {self.config.model_path}"
                )
            joint_id = next((j for j in free_joints if int(model.jnt_bodyid[j]) == root_body_id), -1)
            return root_body_id, joint_id

        if free_joints:
            heaviest = max(free_joints, key=lambda j: float(model.body_subtreemass[model.jnt_bodyid[j]]))
            return int(model.jnt_bodyid[heaviest]), heaviest

        # Fixed base: fall back to the heaviest subtree hanging off the world body.
        children = [b for b in range(1, model.nbody) if int(model.body_parentid[b]) == 0]
        if not children:
            raise ValueError(f"{self.config.model_path} contains no movable bodies")
        return max(children, key=lambda b: float(model.body_subtreemass[b])), -1

    def _body_id(self, name: str) -> int:
        """Body id for ``name``, or -1 when the scene does not define it."""
        return int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name))

    def _subtree_body_ids(self, root_body_id: int) -> np.ndarray:
        """Bodies in the kinematic subtree rooted at ``root_body_id``.

        Used to separate the robot from static workcell props (floor, conveyor,
        pallet) that share the same model.
        """
        members = {root_body_id}
        # body_parentid is topologically sorted, so a single forward pass suffices.
        for body_id in range(root_body_id + 1, self.model.nbody):
            if int(self.model.body_parentid[body_id]) in members:
                members.add(body_id)
        return np.asarray(sorted(members), dtype=int)

    def _match_body_ids(self, tokens: tuple[str, ...]) -> np.ndarray:
        """Bodies whose name contains any of ``tokens`` (case-insensitive)."""
        matches = []
        for body_id in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
            if any(token in name.lower() for token in tokens):
                matches.append(body_id)
        return np.asarray(matches, dtype=int)

    # ------------------------------------------------------------------
    # Core Gymnasium API
    # ------------------------------------------------------------------
    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Reset to a randomised but statically stable standing pose.

        ``options`` is accepted for Gymnasium compatibility and is unused.
        """
        del options
        if gym is not None:
            super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        self._step_count = 0
        mujoco.mj_resetData(self.model, self.data)

        # Start from the home pose and perturb only the actuated joints, so the
        # base stays at its keyframe height and the pose remains stable.
        self.data.qpos[:] = self._home_qpos
        noise = self.config.reset_joint_noise
        if noise > 0.0:
            perturbed = self._nominal_joint_qpos + self._rng.uniform(
                -noise, noise, size=self.model.nu
            )
            self.data.qpos[self._actuator_qpos_adr] = np.clip(
                perturbed, self._action_low, self._action_high
            )
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.clear_external_forces()

        # Randomise the task: payload mass and a jitter on the pick / place poses.
        self._payload_mass = float(self._rng.uniform(*self.config.payload_mass_range))
        self._set_payload_mass(self._payload_mass)
        jitter = self.config.reset_target_noise
        self._pick_target = np.asarray(self.config.pick_position, dtype=float) + self._rng.uniform(
            -jitter, jitter, size=3
        )
        self._place_target = np.asarray(
            self.config.place_position, dtype=float
        ) + self._rng.uniform(-jitter, jitter, size=3)
        self._grasped = False
        self._phase = 0

        # Populate all derived quantities (xpos, subtree_com, ...) before the
        # first observation is read.
        mujoco.mj_forward(self.model, self.data)

        obs = self.get_observation()
        info = {
            "payload_mass": self._payload_mass,
            "pick_target": self._pick_target.copy(),
            "place_target": self._place_target.copy(),
            "phase": self._phase,
        }
        return obs, info

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Advance one control period by substepping the MuJoCo integrator.

        Returns the standard ``(obs, reward, terminated, truncated, info)``
        tuple. ``info["cost"]`` carries the safety cost used by the safe RL
        layer and ``info["barrier"]`` carries the raw CBF values.
        """
        target = np.clip(
            np.asarray(action, dtype=float).reshape(self.model.nu),
            self._action_low,
            self._action_high,
        )

        # Inner loop: recompute the PD torques every sim_dt so tracking stays
        # stable at the 500 Hz integration rate.
        for _ in range(self._n_substeps):
            self._apply_control(target)
            self._reapply_external_wrenches()
            mujoco.mj_step(self.model, self.data)

        # mj_step integrates qpos/qvel but leaves the derived quantities (xipos,
        # subtree_com, cvel, ...) at the pre-integration state, so refresh them
        # before anything reads the CoM or the Jacobians.
        mujoco.mj_forward(self.model, self.data)

        self._step_count += 1
        self._expire_external_forces()
        self._update_task_phase()

        state = self.get_state()
        obs = self.get_observation()
        reward = self.compute_reward(state, target)
        cost = self.compute_safety_cost(state)
        terminated = self._is_fallen(state)
        truncated = self._step_count >= self._max_steps

        info = {
            "cost": cost,
            "barrier": self.barrier_quantities(state),
            "phase": self._phase,
            "grasped": self._grasped,
            "payload_mass": self._payload_mass,
            "com": state["com"].copy(),
            "sim_time": float(self.data.time),
        }
        return obs, reward, terminated, truncated, info

    def step_torque(self, torque: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Advance one control period holding an explicit joint-torque command.

        ``step`` closes its own PD loop around a position target. This path is
        for a caller that has already built the torque (MPC targets, then PD,
        then the CBF filter) and wants that vector written straight to
        ``data.ctrl``. The torque is held constant across the integration
        substeps and clipped to the actuator ctrlrange.
        """
        torque = np.asarray(torque, dtype=float).reshape(self.model.nu).copy()
        return self.step_feedback(lambda: torque)

    def step_feedback(self, torque_fn: Any) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Advance one control period, recomputing the torque every physics step.

        ``torque_fn`` is called once per MuJoCo step and must return a joint
        torque of length ``nu``. A PD law held constant for the whole control
        period (20 ms here) is a zero-order hold, and the leg joints run away
        under it. Re-evaluating inside the substeps is what actually holds a pose.
        """
        last_tau = np.zeros(self.model.nu)
        for _ in range(self._n_substeps):
            tau = np.clip(
                np.asarray(torque_fn(), dtype=float).reshape(self.model.nu),
                self._ctrl_low,
                self._ctrl_high,
            )
            self.data.ctrl[:] = tau
            last_tau = tau
            self._reapply_external_wrenches()
            mujoco.mj_step(self.model, self.data)

        mujoco.mj_forward(self.model, self.data)
        self._step_count += 1
        self._expire_external_forces()
        self._update_task_phase()

        state = self.get_state()
        obs = self.get_observation()
        reward = self.compute_reward(state, last_tau)
        cost = self.compute_safety_cost(state)
        terminated = self._is_fallen(state)
        truncated = self._step_count >= self._max_steps
        info = {
            "cost": cost,
            "barrier": self.barrier_quantities(state),
            "phase": self._phase,
            "grasped": self._grasped,
            "payload_mass": self._payload_mass,
            "com": state["com"].copy(),
            "com_velocity": state["com_velocity"].copy(),
            "torque": last_tau.copy(),
            "sim_time": float(self.data.time),
        }
        return obs, reward, terminated, truncated, info

    def _apply_control(self, target: np.ndarray) -> None:
        """Write one actuator command for the current integration step.

        With MuJoCo position servos the target is the command; with plain motor
        actuators the joint-space PD layer converts targets into torques.
        """
        if self._position_actuators:
            self.data.ctrl[:] = np.clip(target, self._ctrl_low, self._ctrl_high)
            return
        q = self.data.qpos[self._actuator_qpos_adr]
        qd = self.data.qvel[self._actuator_dof_adr]
        tau = self._pd.compute_torque(target, q, qd)
        self.data.ctrl[:] = np.clip(tau, self._ctrl_low, self._ctrl_high)

    def render(self) -> np.ndarray | None:
        """Render according to ``config.render_mode``."""
        mode = self.config.render_mode
        if mode is None:
            return None
        if mode == "human":
            if self._viewer is None:
                # Imported lazily and aliased: `import mujoco.viewer` would
                # shadow the module-level `mujoco` name inside this function.
                import mujoco.viewer as mj_viewer

                self._viewer = mj_viewer.launch_passive(self.model, self.data)
            if self._viewer.is_running():
                self._viewer.sync()
            return None
        if mode == "rgb_array":
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, height=480, width=640)
            self._renderer.update_scene(self.data)
            return self._renderer.render()
        raise ValueError(f"unsupported render_mode {mode!r}; expected {self.metadata['render_modes']}")

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    # ------------------------------------------------------------------
    # Accessors used by the controllers and the safety filter
    # ------------------------------------------------------------------
    def get_state(self) -> dict[str, np.ndarray]:
        """Full dynamic state the MPC linearises around.

        Includes the mass matrix, the bias forces and the stacked foot contact
        Jacobians, i.e. everything needed to assemble

            M(q) qdd + h(q, qd) = S^T tau + J_c^T lambda.
        """
        model, data = self.model, self.data

        mass_matrix = np.zeros((model.nv, model.nv))
        mujoco.mj_fullM(model, data, mass_matrix)

        return {
            "qpos": data.qpos.copy(),
            "qvel": data.qvel.copy(),
            "joint_qpos": data.qpos[self._actuator_qpos_adr].copy(),
            "joint_qvel": data.qvel[self._actuator_dof_adr].copy(),
            "com": self.get_com_position(),
            "com_velocity": self.get_com_velocity(),
            "com_jacobian": self._com_jacobian(),
            "contact_jacobians": self._contact_jacobians(),
            "mass_matrix": mass_matrix,
            "bias_forces": data.qfrc_bias.copy(),
            "actuator_force": data.actuator_force.copy(),
            "base_quat": self._base_quat(),
            "base_position": self._base_position(),
            "base_linvel": self._base_linear_velocity(),
            "base_angvel": self._base_angular_velocity(),
            "base_rotation": data.xmat[self._root_body_id].reshape(3, 3).copy(),
            "torso_rotation": data.xmat[self._torso_body_id].reshape(3, 3).copy(),
            "torso_angular_jacobian": self._torso_angular_jacobian(),
            "support_margin": np.array([self._support_margin()]),
            "payload_mass": np.array([self._payload_mass]),
            "time": np.float64(data.time),
        }

    def get_observation(self) -> np.ndarray:
        """Policy-facing observation vector (see the class docstring layout)."""
        data = self.data
        parts = [
            data.qpos[self._actuator_qpos_adr],  # joint positions
            data.qvel[self._actuator_dof_adr],  # joint velocities
            self._base_quat(),
            self._base_linear_velocity(),
            self._base_angular_velocity(),
            self.get_com_position(),
            self.get_com_velocity(),
            data.ctrl,  # actuator states: last commands ...
            data.actuator_force,  # ... and the forces they produced
        ]
        if self.model.na > 0:
            parts.append(data.act)  # internal activations of stateful actuators
        parts.append([self._payload_mass, float(self._grasped)])
        phase = np.zeros(3)
        phase[self._phase] = 1.0
        parts.append(phase)
        return np.concatenate([np.asarray(p, dtype=np.float64).ravel() for p in parts]).astype(
            np.float32
        )

    def get_com_position(self) -> np.ndarray:
        """Centre of mass of the robot in world coordinates (metres).

        ``subtree_com`` of the floating-base root body is the mass-weighted mean
        over the robot only, so static workcell geometry is excluded.
        """
        return self.data.subtree_com[self._root_body_id].copy()

    def get_com_velocity(self) -> np.ndarray:
        """Linear velocity of the centre of mass in world coordinates (m/s).

        ``subtree_linvel`` is not maintained by ``mj_step``, so the subtree
        velocity pass is run on demand.
        """
        mujoco.mj_subtreeVel(self.model, self.data)
        return self.data.subtree_linvel[self._root_body_id].copy()

    def apply_external_force(
        self,
        force: np.ndarray,
        body_name: str | None = None,
        point: np.ndarray | None = None,
        torque: np.ndarray | None = None,
        duration: float | None = None,
    ) -> None:
        """Apply an external wrench to a body, e.g. a push-recovery disturbance.

        Args:
            force: 3-vector in world coordinates (N).
            body_name: target body; defaults to the floating-base root body.
            point: world-frame application point (m). A force applied away from
                the body CoM is converted into an equivalent force plus torque.
            torque: additional pure torque in world coordinates (N*m).
            duration: seconds to hold the wrench; ``None`` holds it until
                ``clear_external_forces`` or the next ``reset``.

        The wrench is stored and written into ``data.xfrc_applied`` before
        every physics step. MuJoCo clears that array at the end of ``mj_step``,
        so a single write would last only one integration step.
        """
        if body_name is None:
            body_id = self._root_body_id
        else:
            body_id = self._body_id(body_name)
            if body_id < 0:
                raise ValueError(f"body {body_name!r} is not present in the scene")

        force = np.asarray(force, dtype=float).reshape(3)
        wrench_torque = np.zeros(3) if torque is None else np.asarray(torque, dtype=float).reshape(3)
        if point is not None:
            # Shift the force to the body CoM: tau += (p - com) x f
            offset = np.asarray(point, dtype=float).reshape(3) - self.data.xipos[body_id]
            wrench_torque = wrench_torque + np.cross(offset, force)

        wrench = np.concatenate((force, wrench_torque))
        self._external_wrenches[body_id] = wrench
        self.data.xfrc_applied[body_id] = wrench
        if duration is None:
            self._external_wrench_expiry.pop(body_id, None)
        else:
            hold_steps = max(1, round(duration / self.config.control_dt))
            self._external_wrench_expiry[body_id] = self._step_count + hold_steps

    def clear_external_forces(self) -> None:
        """Zero every externally applied wrench."""
        self.data.xfrc_applied[:] = 0.0
        self._external_wrench_expiry.clear()
        self._external_wrenches.clear()

    def _reapply_external_wrenches(self) -> None:
        """Write stored wrenches back into ``xfrc_applied`` before ``mj_step``."""
        for body_id, wrench in self._external_wrenches.items():
            self.data.xfrc_applied[body_id] = wrench

    def _expire_external_forces(self) -> None:
        """Drop timed wrenches whose duration has elapsed."""
        for body_id, expiry in list(self._external_wrench_expiry.items()):
            if self._step_count >= expiry:
                self.data.xfrc_applied[body_id] = 0.0
                self._external_wrenches.pop(body_id, None)
                del self._external_wrench_expiry[body_id]

    # ------------------------------------------------------------------
    # Reward and safety cost
    # ------------------------------------------------------------------
    def compute_reward(self, state: dict[str, np.ndarray], action: np.ndarray) -> float:
        """Task reward: payload tracking, effort penalty, posture regularisation."""
        cfg = self.config

        # Task term: drive the hands towards the active target (pick, then place).
        target = self._place_target if self._phase else self._pick_target
        tracking = -float(np.linalg.norm(self._hand_position() - target))

        # Keep the torso vertical; R[2, 2] is the body z-axis projected on world z.
        upright = float(state["base_rotation"][2, 2])

        effort = float(np.sum(np.square(state["actuator_force"])))
        posture = float(
            np.sum(np.square(state["joint_qpos"] - self._nominal_joint_qpos))
        )

        return (
            cfg.w_task * tracking
            + cfg.w_upright * upright
            - cfg.w_effort * effort
            - cfg.w_posture * posture
        )

    def compute_safety_cost(self, state: dict[str, np.ndarray]) -> float:
        """Constraint violation cost consumed by the safe RL Lagrangian.

        Each term is a hinge on one of the barriers the CBF filter enforces, so
        the cost is zero whenever the state is strictly inside the safe set.
        """
        barriers = self.barrier_quantities(state)
        cost = sum(max(0.0, -value) for value in barriers.values())
        return self.config.cost_scale * float(cost)

    def barrier_quantities(self, state: dict[str, np.ndarray]) -> dict[str, float]:
        """Raw barrier values ``h(x)``; non-negative means safe.

        These mirror the barrier library in ``safety.cbf_filter`` so the filter
        and the RL cost agree on what "safe" means.
        """
        joint_qpos = state["joint_qpos"]
        joint_qvel = state["joint_qvel"]

        barriers = {
            "support_polygon": float(state["support_margin"][0]),
            "joint_limits": float(
                np.min(
                    np.concatenate(
                        (joint_qpos - self._action_low, self._action_high - joint_qpos)
                    )
                )
            ),
            "joint_velocity": float(
                self.config.max_joint_velocity - np.max(np.abs(joint_qvel))
            ),
            "base_height": float(state["base_position"][2] - self.config.min_base_height)
            if self._has_free_base
            else np.inf,
        }
        for i, (x, y, z, radius) in enumerate(self.config.keepout_spheres):
            centre = np.array([x, y, z])
            distance = self._min_body_distance(centre, self._robot_body_ids)
            barriers[f"keepout_{i}"] = float(distance - radius)
        return barriers

    # ------------------------------------------------------------------
    # Internal kinematics / task helpers
    # ------------------------------------------------------------------
    def _base_quat(self) -> np.ndarray:
        """Floating-base orientation as (w, x, y, z); identity for a fixed base."""
        if self._has_free_base:
            adr = self._base_qpos_adr
            return self.data.qpos[adr + 3 : adr + 7].copy()
        return np.array([1.0, 0.0, 0.0, 0.0])

    def _base_position(self) -> np.ndarray:
        """Floating-base origin in world coordinates."""
        if self._has_free_base:
            adr = self._base_qpos_adr
            return self.data.qpos[adr : adr + 3].copy()
        return self.data.xpos[self._root_body_id].copy()

    def _base_linear_velocity(self) -> np.ndarray:
        """Floating-base linear velocity; zeros for a fixed base."""
        if not self._has_free_base:
            return np.zeros(3)
        adr = self._base_dof_adr
        return self.data.qvel[adr : adr + 3].copy()

    def _base_angular_velocity(self) -> np.ndarray:
        """Floating-base angular velocity; zeros for a fixed base."""
        if not self._has_free_base:
            return np.zeros(3)
        adr = self._base_dof_adr
        return self.data.qvel[adr + 3 : adr + 6].copy()

    def _com_jacobian(self) -> np.ndarray:
        """3 x nv Jacobian mapping joint velocities to CoM linear velocity.

        Built as the mass-weighted sum of the per-body CoM Jacobians.
        """
        jacobian = np.zeros((3, self.model.nv))
        body_jac = np.zeros((3, self.model.nv))
        total_mass = 0.0
        for body_id in self._robot_body_ids:
            mass = float(self.model.body_mass[body_id])
            if mass <= 0.0:
                continue
            mujoco.mj_jacBodyCom(self.model, self.data, body_jac, None, int(body_id))
            jacobian += mass * body_jac
            total_mass += mass
        return jacobian / total_mass if total_mass > 0.0 else jacobian

    def _torso_angular_jacobian(self) -> np.ndarray:
        """3 x nv Jacobian mapping joint velocities to torso angular velocity.

        The MPC regulates the torso orientation, so it needs ``omega = J_r qd``
        for the torso body.
        """
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, None, jacr, self._torso_body_id)
        return jacr

    def _contact_jacobians(self) -> dict[str, np.ndarray]:
        """Translational Jacobian per foot body, keyed by body name."""
        jacobians = {}
        for body_id in self._foot_body_ids:
            jacp = np.zeros((3, self.model.nv))
            mujoco.mj_jacBodyCom(self.model, self.data, jacp, None, int(body_id))
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, int(body_id))
            jacobians[name or f"body_{body_id}"] = jacp
        return jacobians

    def _support_margin(self) -> float:
        """Horizontal distance from the CoM to the edge of the foot convex hull.

        Approximated by the distance to the nearest foot centre minus the mean
        half-spacing of the stance, which is adequate for a cost signal; the CBF
        filter uses the exact polygon.
        """
        if self._foot_body_ids.size == 0:
            return float(self.config.min_com_support_margin)
        com_xy = self.get_com_position()[:2]
        foot_xy = self.data.xipos[self._foot_body_ids][:, :2]
        centre = foot_xy.mean(axis=0)
        if foot_xy.shape[0] > 1:
            extent = 0.5 * float(np.max(np.linalg.norm(foot_xy - centre, axis=1)))
        else:
            extent = 0.1
        return float(extent - np.linalg.norm(com_xy - centre))

    def _hand_position(self) -> np.ndarray:
        """Mean hand position, or the CoM when the scene defines no hands."""
        if self._hand_body_ids.size == 0:
            return self.get_com_position()
        return self.data.xipos[self._hand_body_ids].mean(axis=0)

    def _min_body_distance(self, point: np.ndarray, body_ids: np.ndarray) -> float:
        """Smallest distance from ``point`` to any of ``body_ids``."""
        if body_ids.size == 0:
            return float(np.inf)
        offsets = self.data.xipos[body_ids] - point
        return float(np.min(np.linalg.norm(offsets, axis=1)))

    def _set_payload_mass(self, mass: float) -> None:
        """Retune the payload body's mass and inertia for this episode."""
        if self._payload_body_id < 0:
            return
        self.model.body_mass[self._payload_body_id] = mass
        if self._nominal_payload_inertia is not None and self._nominal_payload_mass > 0.0:
            # Same shape, new mass: inertia scales linearly with mass.
            scale = mass / self._nominal_payload_mass
            self.model.body_inertia[self._payload_body_id] = (
                self._nominal_payload_inertia * scale
            )

    def _update_task_phase(self) -> None:
        """Advance the pick / carry / place state machine."""
        hand = self._hand_position()
        if not self._grasped:
            if float(np.linalg.norm(hand - self._pick_target)) <= self.config.grasp_distance:
                self._grasped = True
                self._phase = 1
            return
        if self._phase == 1 and float(np.linalg.norm(hand - self._place_target)) <= (
            2.0 * self.config.grasp_distance
        ):
            self._phase = 2

    def _is_fallen(self, state: dict[str, np.ndarray]) -> bool:
        """True when the robot has lost its posture or the sim has diverged."""
        if not np.all(np.isfinite(state["qpos"])) or not np.all(np.isfinite(state["qvel"])):
            return True
        if not self._has_free_base:
            return False
        base_height = float(state["base_position"][2])
        upright = float(state["base_rotation"][2, 2])
        return base_height < self.config.min_base_height or upright < self.config.min_upright
