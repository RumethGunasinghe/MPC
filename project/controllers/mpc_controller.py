"""Simplified whole-body model predictive controller for a humanoid.

This is a *research prototype*: it trades the full centroidal-dynamics QP for a
set of decoupled linear models that are cheap enough to re-solve every control
tick and simple enough to reason about.

Formulation
-----------
The controller regulates a six-dimensional task vector

    z = [ c ; theta ]        c     in R^3 : centre-of-mass position (world)
                             theta in R^3 : torso orientation error (world)

where ``theta`` is the rotation-vector (matrix-logarithm) coordinate of the
torso attitude error relative to the commanded attitude,

    theta = log( R_torso  R_ref^T ).

For small errors ``theta`` behaves like a Euclidean coordinate whose time
derivative is the torso angular velocity, ``d/dt theta ~= omega``. That is the
one approximation that lets the orientation task reuse the same linear model as
the CoM task. It is accurate to second order in the error angle, which is all a
posture-regulating controller needs.

Each task axis is modelled as a **double integrator** driven by the task
acceleration ``u = [ c_ddot ; omega_dot ]``:

    z_ddot = u

For the CoM this is exact: ``m c_ddot = sum(f_ext)``, so commanding a CoM
acceleration is equivalent to commanding the net external force. What the model
omits is the *feasibility* of that force -- the friction cone and the
centre-of-pressure staying inside the support polygon. Those constraints are
delegated to the CBF safety filter downstream.

Horizon cost
------------
Over a finite horizon of ``N`` knots the controller minimises

    J = sum_{k=1..N}  || z_k - z_ref,k ||^2_Wz  +  || z_dot_k - z_dot_ref,k ||^2_Wv
      + sum_{k=0..N-1} || u_k ||^2_Wu  +  || u_k - u_{k-1} ||^2_Wj

**Every term of this cost is diagonal in the task axes, and the double
integrator does not couple them either.** The six-dimensional problem therefore
splits exactly into six independent scalar problems, one per axis, each with a
2-state model

    x_{k+1} = A x_k + B u_k,     A = [ 1  dt ]   B = [ dt^2/2 ]
                                     [ 0   1 ]       [   dt   ]

This is not an approximation: solving the six small problems reproduces the
monolithic optimum to machine precision, while being roughly a hundred times
faster once the acceleration bounds become active.

Each axis is condensed over the horizon,

    X = Phi x_0 + Gamma U,        Phi in R^{2N x 2},  Gamma in R^{2N x N},

which turns the cost into a linear least-squares problem in the stacked control
``U in R^N``. Because ``Phi`` and ``Gamma`` depend only on ``dt`` and ``N``, the
least-squares matrix is constant and its pseudo-inverse is precomputed in
``__init__``; an unconstrained solve is then a single matrix-vector product.

Box limits on the task accelerations turn that into a *bounded* least-squares
problem, which has no closed form. ``scipy.optimize.lsq_linear`` handles it, but
only *when required*: every axis first evaluates the closed form, and scipy is
invoked only for those axes whose optimum violates a bound. Near the reference
no bound is active and the whole solve stays a handful of matrix-vector
products. The inverse-kinematics stage below follows the same pattern with the
joint-velocity limits.

From task accelerations to joint targets
----------------------------------------
Only the first element ``u_0`` of each optimal sequence is used (receding
horizon). It is converted into joint position targets by a damped least-squares
inverse-kinematics step, asking for the task velocity the horizon solution
implies after one tick, ``v* = v_task + u_0 dt``.

The Jacobian used here needs care. The raw CoM Jacobian ``J_com`` has the
identity in its floating-base translation block, because translating the base
translates the whole robot. A least-norm solve therefore satisfies almost the
entire CoM task by moving the base -- which is *unactuated*, so nothing actually
happens. On the H1 that leaves under 2% of the commanded CoM velocity in the
joints.

The fix is the **contact-relative** CoM Jacobian. With the stance feet planted,
the CoM velocity that joint motion can produce is

    J_rel = J_com - J_feet,       J_feet = mean of the stance-foot Jacobians,

whose base-translation block cancels exactly. And because a planted foot is
stationary in the world, the relative and absolute CoM velocities coincide, so a
world-frame CoM reference can be used unchanged. Restricting the columns to the
actuated joints then makes the task solvable only by joint motion:

    J = [ (J_com - J_feet) ; J_torso,rot ]_actuated  in R^{6 x nu}

    min_qd  || W (J qd - v*) ||^2  +  w_post || qd - qd_post ||^2  +  lambda || qd ||^2

``v*`` is the velocity the joints should produce. It is the reference
velocity plus the MPC's planned acceleration over this tick, plus a
task-space P term. The *measured* task velocity is deliberately not copied:
on a floating base that velocity is the base falling, and feeding it back
into the joints makes the legs join the fall.

    v* = v_ref + 1/2 u_0 dt + Kp (z_ref,0 - z)

Each Jacobian row is scaled by its own norm before stacking, otherwise the
actuated CoM-z row of a floating-base H1 (~0.1) is invisible next to torso yaw
(~1) and the squat is silently dropped.

The posture term ``qd_post = k_post (q_home - q)`` resolves the null space of
the 6-row task (the H1 has 19 actuated joints, so 13 redundant directions) by
drifting the unconstrained joints back towards the home pose. The position
target is that home pose plus one tick of the solved joint velocity, plus a
capture-point shift of the ankles and hips. Integrating from the measured
joint position instead would let a joint that gravity has already bent become
the new target, and the PD would never pull it back.

A squat is only kinematically possible when the *feet* stay planted and the
pelvis is free to drop. Pinning the pelvis and asking the legs to lower the
CoM cannot work: the legs are light and the mass lives in the torso.

Known simplifications
---------------------
* No contact-force variables, friction cones or centre-of-pressure limits, so
  the commanded CoM acceleration may not be dynamically realisable. Without
  those constraints the controller tracks a CoM reference but cannot balance:
  expect it to track well and then topple.
* The stance feet are assumed planted and load-sharing equally. There is no
  contact detection, scheduling or swing phase, so this is a double-support
  posture controller, not a walking controller.
* ``J_dot qd`` is dropped when mapping accelerations to velocities, which is
  valid at the low joint speeds of a posture-regulation task.
* No torque feed-forward is produced: the PD layer closes the loop on position.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.optimize import lsq_linear

try:
    import mujoco
except ImportError:  # pragma: no cover - keeps the scaffold importable
    mujoco = None

# Task layout: the first three axes are the CoM, the last three the torso
# orientation error. Axes 0-2 share one set of weights and bounds, axes 3-5 the
# other, so only two distinct scalar problems are ever built.
_N_TASK = 6
_COM = slice(0, 3)
_ORI = slice(3, 6)
_AXIS_GROUP = (0, 0, 0, 1, 1, 1)  # index into the two weight groups


@dataclass
class MPCConfig:
    """Horizon and cost weights for the simplified whole-body MPC."""

    horizon: int = 20  # N knots, i.e. a 0.4 s preview at dt = 0.02
    dt: float = 0.02

    # Task-space tracking weights.
    w_com: float = 100.0
    w_com_velocity: float = 10.0
    w_orientation: float = 30.0
    w_angular_velocity: float = 3.0

    # Regularisation on the commanded task accelerations. ``w_jerk`` penalises
    # the change between consecutive accelerations, which is what keeps the
    # receding-horizon solution smooth from tick to tick.
    w_acceleration: float = 1e-3  # on CoM acceleration (m/s^2)
    w_angular_acceleration: float = 1e-3  # on angular acceleration (rad/s^2)
    w_jerk: float = 1e-2

    # Box limits on the task accelerations. Finite values make a solve fall back
    # to scipy.optimize.lsq_linear whenever the closed form violates them.
    max_com_acceleration: float = 5.0  # m/s^2, ~0.5 g
    max_angular_acceleration: float = 20.0  # rad/s^2
    bounded: bool = True  # set False to always use the unconstrained closed form

    # Inverse-kinematics stage.
    w_ik_com: float = 5.0  # relative priority of the CoM rows
    w_ik_orientation: float = 1.0  # relative priority of the orientation rows
    w_posture: float = 1e-3  # pull towards the home pose in the null space
    posture_gain: float = 0.5  # 1/s, proportional gain of that pull
    damping: float = 1e-6  # Tikhonov damping, keeps J rank-deficiency benign
    max_joint_velocity: float = 8.0  # rad/s, bounds on the IK solution
    # Extra task-space P gain. The measured task velocity is not fed back into
    # this term; see ``_joint_targets``.
    ik_position_gain: float = 4.0  # 1/s
    # Scale each controllable Jacobian row by its own norm before stacking.
    # Without this the actuated CoM-z row (~0.1) is invisible next to the
    # torso-yaw row (~1). Rows the actuated joints cannot move are dropped.
    scale_jacobian_rows: bool = True

    # Make the CoM rows contact-relative by subtracting the stance-foot
    # Jacobian. Required for a floating base in double support (see the module
    # docstring). Turn this off only for a *model* whose root is welded to the
    # world -- not for a test that teleports the pelvis, which cannot squat.
    contact_relative: bool = True

    # Capture-point ankle offset, in rad per metre of capture-point error.
    # Shifts both ankles (and the hip rolls, laterally) so the centre of
    # pressure follows the capture point. Zero disables it.
    capture_gain: float = 12.0


class WholeBodyMPC:
    """Receding-horizon CoM + torso-orientation controller.

    The decision variables are the task accelerations
    ``u_k = [c_ddot_k ; omega_dot_k]`` over a finite horizon; the output is a
    vector of joint position targets for the PD layer. See the module docstring
    for the full derivation.
    """

    def __init__(self, model: Any, config: MPCConfig | None = None) -> None:
        self.model = model
        self.config = config or MPCConfig()
        if self.config.horizon < 1:
            raise ValueError(f"horizon must be >= 1, got {self.config.horizon}")
        if self.config.dt <= 0.0:
            raise ValueError(f"dt must be > 0, got {self.config.dt}")

        # Previous solution (N x 6), used both as the warm start for the jerk
        # term and as the fall-back when a solve fails.
        self._last_solution: np.ndarray | None = None

        self._index_model()
        self._build_problem()

    # ------------------------------------------------------------------
    # Set-up (done once)
    # ------------------------------------------------------------------
    def _index_model(self) -> None:
        """Cache the actuator -> joint maps, joint limits and the home pose.

        The MPC needs these to turn a joint-velocity solution into position
        targets for the actuated joints only; the floating base is not
        commanded. Everything is read from the compiled model, so a fixed-base
        or re-ordered model works without changes here.
        """
        model = self.model
        if model is None or mujoco is None:
            # Keeps the module importable and unit-testable without MuJoCo; the
            # horizon solver below does not depend on the model at all.
            self.num_joints = 0
            self.nv = 0
            self._actuator_qpos_adr = np.zeros(0, dtype=int)
            self._actuator_dof_adr = np.zeros(0, dtype=int)
            self._joint_low = np.zeros(0)
            self._joint_high = np.zeros(0)
            self._home_joint_qpos = np.zeros(0)
            self._joint_names: list[str] = []
            return

        self.num_joints = int(model.nu)
        self.nv = int(model.nv)

        joint_ids = np.array(
            [int(model.actuator_trnid[i, 0]) for i in range(model.nu)], dtype=int
        )
        self._actuator_qpos_adr = model.jnt_qposadr[joint_ids].astype(int)
        self._actuator_dof_adr = model.jnt_dofadr[joint_ids].astype(int)

        limited = model.jnt_limited[joint_ids].astype(bool)
        ranges = model.jnt_range[joint_ids]
        self._joint_low = np.where(limited, ranges[:, 0], -np.inf)
        self._joint_high = np.where(limited, ranges[:, 1], np.inf)

        home_qpos = model.key_qpos[0] if model.nkey > 0 else model.qpos0
        self._home_joint_qpos = np.asarray(home_qpos)[self._actuator_qpos_adr].copy()
        self._joint_names = []
        for joint_id in joint_ids:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(joint_id))
            self._joint_names.append(name or "")

    def _build_problem(self) -> None:
        """Assemble the per-axis condensed model and factor the least squares.

        Builds ``Phi`` and ``Gamma`` for the scalar double integrator so that
        ``X = Phi x_0 + Gamma U`` for one task axis, then stacks the weighted
        residual rows for each of the two weight groups (CoM and orientation)
        and precomputes their pseudo-inverses.
        """
        cfg = self.config
        N, dt = cfg.horizon, cfg.dt

        # Scalar double integrator: x = [position, velocity].
        A = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([[0.5 * dt * dt], [dt]])
        self._A, self._B = A, B

        # Condensed prediction: x_{k+1} = A^{k+1} x_0 + sum_{j<=k} A^{k-j} B u_j.
        Phi = np.zeros((2 * N, 2))
        Gamma = np.zeros((2 * N, N))
        A_power = np.eye(2)
        for k in range(N):
            A_power = A_power @ A  # A^(k+1)
            Phi[2 * k : 2 * k + 2, :] = A_power
            for j in range(k + 1):
                Gamma[2 * k : 2 * k + 2, j] = (np.linalg.matrix_power(A, k - j) @ B).ravel()
        self._Phi, self._Gamma = Phi, Gamma

        # First-difference operator for the jerk penalty: row k is u_k - u_{k-1}.
        D = np.eye(N) - np.eye(N, k=-1)
        self._D = D
        self._sqrt_w_jerk = float(np.sqrt(cfg.w_jerk))

        # Two weight groups: CoM axes and orientation axes.
        self._groups: list[dict[str, Any]] = []
        for w_position, w_velocity, w_control, max_control in (
            (cfg.w_com, cfg.w_com_velocity, cfg.w_acceleration, cfg.max_com_acceleration),
            (
                cfg.w_orientation,
                cfg.w_angular_velocity,
                cfg.w_angular_acceleration,
                cfg.max_angular_acceleration,
            ),
        ):
            # State weights alternate position / velocity down the stacked X.
            sqrt_state = np.sqrt(np.tile([w_position, w_velocity], N))
            matrix = np.vstack(
                (
                    sqrt_state[:, None] * Gamma,
                    np.sqrt(w_control) * np.eye(N),
                    self._sqrt_w_jerk * D,
                )
            )
            bound = abs(float(max_control))
            self._groups.append(
                {
                    "sqrt_state": sqrt_state,
                    "matrix": matrix,
                    # The unconstrained optimum is a fixed linear map, evaluated
                    # first on every solve.
                    "pinv": np.linalg.pinv(matrix),
                    "low": np.full(N, -bound),
                    "high": np.full(N, bound),
                    "bounded": bool(cfg.bounded and np.isfinite(bound)),
                }
            )

    # ------------------------------------------------------------------
    # Solve
    # ------------------------------------------------------------------
    def solve(
        self,
        state: dict[str, np.ndarray],
        reference: dict[str, np.ndarray] | None = None,
    ) -> dict[str, np.ndarray]:
        """Solve one MPC iteration.

        Args:
            state: current whole-body state from
                ``IndustrialHumanoidEnv.get_state``. Requires ``com``,
                ``com_velocity``, ``com_jacobian``, ``torso_rotation``,
                ``torso_angular_jacobian``, ``qvel`` and ``joint_qpos``.
            reference: desired trajectories. Recognised keys, each either a
                single target or one row per horizon knot:

                * ``com`` (3,) or (N, 3) -- desired CoM position
                * ``com_velocity`` (3,) or (N, 3) -- desired CoM velocity
                * ``torso_orientation`` -- desired torso attitude as a 3x3
                  rotation matrix, a (w, x, y, z) quaternion, or a stack of
                  either
                * ``angular_velocity`` (3,) or (N, 3) -- desired torso angular
                  velocity

                ``None``, or any omitted key, means "hold the current value",
                which makes the controller a posture regulator.

        Returns:
            Dict with ``q_des`` / ``qd_des`` (targets for the PD layer),
            ``u_task`` (the applied task acceleration), ``com_prediction`` (the
            predicted CoM over the horizon, for logging and plots) and
            ``solve_info`` diagnostics.
        """
        task = self._task_state(state, reference)
        control, prediction, info = self._solve_horizon(task)

        # Receding horizon: apply only the first control of each sequence.
        u0 = control[0]
        q_des, qd_des, ik_info = self._joint_targets(state, task, u0)

        self._last_solution = control
        return {
            "q_des": q_des,
            "qd_des": qd_des,
            "u_task": u0,
            "com_acceleration": u0[_COM],
            "angular_acceleration": u0[_ORI],
            "com_prediction": prediction[:, _COM],
            "orientation_error_prediction": prediction[:, _ORI],
            "com_reference": task["position_reference"][:, _COM],
            "orientation_error": task["position"][_ORI],
            "solve_info": {**info, **ik_info},
        }

    def _task_state(
        self,
        state: dict[str, np.ndarray],
        reference: dict[str, np.ndarray] | None,
    ) -> dict[str, np.ndarray]:
        """Build the current task state and the stacked horizon reference.

        The CoM enters directly. The orientation enters through the rotation
        vector ``theta = log(R_torso R_ref^T)``, measured against the *first*
        reference attitude so that every knot shares one linearisation frame:
        the reference for knot ``k`` is then
        ``theta_ref,k = log(R_ref,k R_ref,0^T)``, which is zero for a constant
        attitude command.
        """
        N = self.config.horizon
        reference = reference or {}

        com = np.asarray(state["com"], dtype=float).reshape(3)
        com_velocity = np.asarray(state["com_velocity"], dtype=float).reshape(3)
        rotation = np.asarray(state["torso_rotation"], dtype=float).reshape(3, 3)
        angular_jacobian = np.asarray(state["torso_angular_jacobian"], dtype=float)
        qvel = np.asarray(state["qvel"], dtype=float)
        # Angular velocity consistent with the Jacobian used by the IK stage.
        omega = angular_jacobian @ qvel

        com_reference = _expand(reference.get("com"), com, N)
        com_velocity_reference = _expand(reference.get("com_velocity"), np.zeros(3), N)
        angular_velocity_reference = _expand(reference.get("angular_velocity"), np.zeros(3), N)

        rotation_reference = _expand_rotations(reference.get("torso_orientation"), rotation, N)
        # Linearisation frame: the first commanded attitude.
        frame = rotation_reference[0]
        theta = _rotation_log(rotation @ frame.T)
        theta_reference = np.array([_rotation_log(R @ frame.T) for R in rotation_reference])

        return {
            "position": np.concatenate((com, theta)),  # (6,)
            "velocity": np.concatenate((com_velocity, omega)),  # (6,)
            "position_reference": np.hstack((com_reference, theta_reference)),  # (N, 6)
            "velocity_reference": np.hstack(
                (com_velocity_reference, angular_velocity_reference)
            ),  # (N, 6)
        }

    def _solve_horizon(
        self, task: dict[str, np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
        """Minimise the horizon cost, one independent problem per task axis.

        For each axis the weighted residual is

            [ sqrt(Wx) Gamma ]       [ sqrt(Wx) (X_ref - Phi x0) ]
            [ sqrt(Wu) I     ] U  -  [ 0                         ]
            [ sqrt(Wj) D     ]       [ sqrt(Wj) u_prev e_0       ]

        The precomputed pseudo-inverse gives the unconstrained optimum. Only if
        that violates an acceleration bound is the axis handed to
        ``scipy.optimize.lsq_linear``, which costs orders of magnitude more.

        Returns:
            ``(control, prediction, info)`` with ``control`` of shape
            ``(N, 6)``, ``prediction`` the predicted task positions
            ``(N, 6)``, and solver diagnostics.
        """
        N = self.config.horizon
        control = np.zeros((N, _N_TASK))
        prediction = np.zeros((N, _N_TASK))
        axes_using_scipy: list[int] = []
        statuses: list[int] = []

        for axis in range(_N_TASK):
            group = self._groups[_AXIS_GROUP[axis]]

            # Initial state and reference trajectory for this axis, interleaved
            # as [position_1, velocity_1, position_2, ...] to match Phi/Gamma.
            x0 = np.array([task["position"][axis], task["velocity"][axis]])
            x_ref = np.empty(2 * N)
            x_ref[0::2] = task["position_reference"][:, axis]
            x_ref[1::2] = task["velocity_reference"][:, axis]

            # Continuity: the jerk penalty on the first knot references the
            # control applied last tick, so the plan does not jump between
            # solves.
            jerk_rhs = np.zeros(N)
            if self._last_solution is not None:
                jerk_rhs[0] = self._last_solution[0, axis]

            rhs = np.concatenate(
                (
                    group["sqrt_state"] * (x_ref - self._Phi @ x0),
                    np.zeros(N),
                    self._sqrt_w_jerk * jerk_rhs,
                )
            )

            # Closed form first; exact unless a bound is hit.
            sequence = group["pinv"] @ rhs
            if group["bounded"] and not _within(sequence, group["low"], group["high"]):
                result = lsq_linear(
                    group["matrix"], rhs, bounds=(group["low"], group["high"]), method="bvls"
                )
                if np.all(np.isfinite(result.x)):
                    sequence = result.x
                    axes_using_scipy.append(axis)
                    statuses.append(int(result.status))
                elif self._last_solution is not None:
                    # Degenerate solve: keep the previous plan for this axis
                    # rather than commanding NaN.
                    sequence = self._last_solution[:, axis]

            control[:, axis] = sequence
            prediction[:, axis] = (self._Phi @ x0 + self._Gamma @ sequence)[0::2]

        return (
            control,
            prediction,
            {
                "solver": "lsq_linear" if axes_using_scipy else "pinv",
                "bounded_axes": axes_using_scipy,
                "status": min(statuses) if statuses else 0,
            },
        )

    def _joint_targets(
        self,
        state: dict[str, np.ndarray],
        task: dict[str, np.ndarray],
        u0: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
        """Map the optimal task acceleration onto joint position targets.

        Converts the first predicted knot into a task velocity, row-scales the
        contact-relative Jacobian, and inverts it over the actuated joints in a
        damped least-squares sense. A posture term resolves the null space.
        See the module docstring for why the raw CoM Jacobian cannot be used.
        """
        cfg = self.config
        dt = cfg.dt
        if not self.num_joints:
            return np.zeros(0), np.zeros(0), {"ik_solver": "none"}

        jacobian = self._task_jacobian(state)  # 6 x nu, actuated columns only

        # Task velocity the actuated joints should produce. The measured task
        # velocity is left out on purpose: it is dominated by the unactuated
        # base, and tracking it commands the legs to follow the fall.
        #
        #     v* = v_ref + 1/2 u_0 dt + Kp (z_ref,0 - z)
        position_error = task["position_reference"][0] - task["position"]
        velocity_target = (
            task["velocity_reference"][0]
            + 0.5 * u0 * dt
            + cfg.ik_position_gain * position_error
        )

        # Equalise the task rows the actuated joints can actually move. The raw
        # CoM-z row of a floating-base H1 is an order of magnitude smaller than
        # the torso-yaw row, so without this the least-squares solve quietly
        # drops the squat and holds the heading.
        #
        # Torso pitch and roll are floating-base coordinates: the hip joints do
        # not appear in that Jacobian, so those rows are numerical noise
        # (~1e-5). Dividing by that norm turns a milliradian of error into a
        # joint-velocity command of thousands of rad/s and the robot folds.
        # Those rows are dropped. Pitch and roll are regulated through the CoM
        # task instead, which the legs can move.
        if cfg.scale_jacobian_rows:
            row_scale = np.linalg.norm(jacobian, axis=1)
            controllable = row_scale >= 1e-2
            row_scale = np.where(controllable, row_scale, 1.0)
            jacobian = jacobian / row_scale[:, None]
            velocity_target = velocity_target / row_scale
            jacobian[~controllable] = 0.0
            velocity_target[~controllable] = 0.0

        # Row weights: relative priority of the CoM versus the orientation task.
        sqrt_task_weights = np.sqrt(
            np.concatenate((np.full(3, cfg.w_ik_com), np.full(3, cfg.w_ik_orientation)))
        )

        # Posture bias: a joint velocity that decays the offset from the home
        # pose, used to pick one solution out of the task's null space.
        joint_qpos = np.asarray(state["joint_qpos"], dtype=float)
        posture_velocity = cfg.posture_gain * (self._home_joint_qpos - joint_qpos)

        # Stack the three residual blocks into one least-squares problem.
        sqrt_posture = np.sqrt(cfg.w_posture)
        sqrt_damping = np.sqrt(cfg.damping)
        identity = np.eye(self.num_joints)
        matrix = np.vstack(
            (
                sqrt_task_weights[:, None] * jacobian,
                sqrt_posture * identity,
                sqrt_damping * identity,
            )
        )
        rhs = np.concatenate(
            (
                sqrt_task_weights * velocity_target,
                sqrt_posture * posture_velocity,
                np.zeros(self.num_joints),
            )
        )

        bound = abs(float(cfg.max_joint_velocity))
        low = np.full(self.num_joints, -bound)
        high = np.full(self.num_joints, bound)

        # Closed form first, exactly as in the horizon solve: scipy is only
        # needed when the unconstrained solution exceeds a joint-velocity limit.
        qd, *_ = np.linalg.lstsq(matrix, rhs, rcond=None)
        ik_info: dict[str, Any] = {"ik_solver": "lstsq"}
        if np.isfinite(bound) and not _within(qd, low, high):
            result = lsq_linear(matrix, rhs, bounds=(low, high), method="trf")
            qd = result.x
            ik_info = {"ik_solver": "lsq_linear", "ik_status": int(result.status)}

        if not np.all(np.isfinite(qd)):
            qd = np.zeros(self.num_joints)
            ik_info["ik_failed"] = True
        ik_info["ik_residual"] = float(np.linalg.norm(jacobian @ qd - velocity_target))

        # The target is the home pose, not the measured pose. A measured joint
        # that has already bent under gravity must stay an error, otherwise the
        # PD has nothing to correct. ``qd * dt`` is the one-tick task motion;
        # the capture offset is what keeps the floating base from toppling.
        q_des = self._home_joint_qpos + qd * dt + self._capture_offset(state, task)
        q_des = np.clip(q_des, self._joint_low, self._joint_high)
        return q_des, np.zeros(self.num_joints), ik_info

    def _capture_offset(self, state: dict[str, np.ndarray], task: dict[str, np.ndarray]) -> np.ndarray:
        """Ankle and hip shift that moves the centre of pressure toward the capture point.

        Shifting both ankles with the sagittal capture-point error places the
        centre of pressure under the falling body. Hip pitch is left to the
        posture target: coupling it to the ankle folds the knee and drops the
        pelvis. Both hip rolls take the lateral error. The H1's left and right
        axes point the same way, so both legs use the same sign.
        """
        offset = np.zeros(self.num_joints)
        gain = float(self.config.capture_gain)
        if gain == 0.0 or not self._joint_names:
            return offset
        com = np.asarray(state["com"], dtype=float).reshape(3)
        vel = np.asarray(state["com_velocity"], dtype=float).reshape(3)
        reference = np.asarray(task["position_reference"][0, :3], dtype=float)
        omega = np.sqrt(9.81 / max(float(com[2]), 0.2))
        error_x = (com[0] - reference[0]) + vel[0] / omega
        error_y = (com[1] - reference[1]) + vel[1] / omega
        for index, name in enumerate(self._joint_names):
            if "ankle" in name:
                offset[index] = gain * error_x
            elif "hip_roll" in name:
                offset[index] = gain * error_y
        # A larger shift rolls the foot onto the heel or the toe and the body
        # pivots over that edge. A few degrees is enough to move the centre
        # of pressure.
        return np.clip(offset, -0.15, 0.15)

    def _task_jacobian(self, state: dict[str, np.ndarray]) -> np.ndarray:
        """Stacked 6 x nu task Jacobian over the actuated joints.

        For a floating base the CoM rows are made contact-relative by
        subtracting the mean stance-foot Jacobian, which cancels the base
        translation block; see the module docstring. With
        ``config.contact_relative = False`` (a fixed or pinned base) the raw CoM
        Jacobian is used instead, since joint motion already drives it.
        """
        com_jacobian = np.asarray(state["com_jacobian"], dtype=float)
        angular_jacobian = np.asarray(state["torso_angular_jacobian"], dtype=float)

        contacts = state.get("contact_jacobians") or {}
        if self.config.contact_relative and contacts:
            foot_jacobian = np.mean(
                [np.asarray(j, dtype=float) for j in contacts.values()], axis=0
            )
            com_jacobian = com_jacobian - foot_jacobian

        return np.vstack(
            (com_jacobian[:, self._actuator_dof_adr], angular_jacobian[:, self._actuator_dof_adr])
        )

    def reset(self) -> None:
        """Drop the warm-start cache between episodes."""
        self._last_solution = None


# ----------------------------------------------------------------------
# Small maths helpers
# ----------------------------------------------------------------------
def _within(values: np.ndarray, low: np.ndarray, high: np.ndarray, tol: float = 1e-9) -> bool:
    """True when ``values`` satisfies the box ``[low, high]`` to within ``tol``."""
    return bool(np.all(values >= low - tol) and np.all(values <= high + tol))


def _expand(value: Any, default: np.ndarray, horizon: int) -> np.ndarray:
    """Broadcast a reference into a ``(horizon, 3)`` trajectory.

    Accepts ``None`` (use ``default``), a single 3-vector (held constant over
    the horizon) or a ``(horizon, 3)`` trajectory. Shorter trajectories are
    padded by repeating the last knot, which is the usual convention for a
    reference that runs out before the horizon does.
    """
    if value is None:
        return np.tile(np.asarray(default, dtype=float).reshape(3), (horizon, 1))
    array = np.asarray(value, dtype=float)
    if array.ndim == 1:
        return np.tile(array.reshape(3), (horizon, 1))
    if array.shape[0] >= horizon:
        return array[:horizon, :3].astype(float)
    padding = np.tile(array[-1, :3], (horizon - array.shape[0], 1))
    return np.vstack((array[:, :3], padding)).astype(float)


def _expand_rotations(value: Any, default: np.ndarray, horizon: int) -> np.ndarray:
    """Broadcast an attitude reference into ``(horizon, 3, 3)`` rotation matrices.

    Accepts ``None``, a single rotation (3x3 matrix or (w, x, y, z) quaternion),
    or a stack of either, padding short trajectories like :func:`_expand`.
    """
    if value is None:
        return np.tile(np.asarray(default, dtype=float).reshape(1, 3, 3), (horizon, 1, 1))

    array = np.asarray(value, dtype=float)
    if array.shape == (4,):  # single quaternion
        rotations = [_quaternion_to_matrix(array)]
    elif array.shape == (3, 3):  # single rotation matrix
        rotations = [array]
    elif array.ndim == 2 and array.shape[1] == 4:  # quaternion trajectory
        rotations = [_quaternion_to_matrix(q) for q in array]
    elif array.ndim == 3 and array.shape[1:] == (3, 3):  # matrix trajectory
        rotations = list(array)
    else:
        raise ValueError(
            "torso_orientation must be a 3x3 matrix, a (w, x, y, z) quaternion, "
            f"or a stack of either; got shape {array.shape}"
        )

    while len(rotations) < horizon:
        rotations.append(rotations[-1])
    return np.asarray(rotations[:horizon])


def _quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Convert a MuJoCo-ordered ``(w, x, y, z)`` quaternion to a rotation matrix."""
    w, x, y, z = np.asarray(quaternion, dtype=float).reshape(4)
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        raise ValueError("cannot convert a zero-norm quaternion to a rotation")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _rotation_log(rotation: np.ndarray) -> np.ndarray:
    """Matrix logarithm of a rotation, as a rotation vector ``axis * angle``.

    This is the inverse of the exponential map on SO(3). For a rotation of
    angle ``phi`` about a unit ``axis``,

        phi = arccos( (tr(R) - 1) / 2 ),
        axis = 1 / (2 sin phi) * [ R32 - R23, R13 - R31, R21 - R12 ].

    The two singular branches are handled separately: near ``phi = 0`` the skew
    part is already the rotation vector to first order, and near ``phi = pi``
    ``sin phi -> 0`` so the axis is recovered from the symmetric part instead.
    """
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    skew = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cosine))

    if angle < 1e-8:
        # log(R) ~= skew/2 for R ~= I.
        return 0.5 * skew

    if angle > np.pi - 1e-6:
        # Near a half turn: (R + I)/2 = axis axis^T, so the axis is the
        # normalised column of that matrix with the largest diagonal entry.
        symmetric = 0.5 * (rotation + np.eye(3))
        axis = symmetric[:, int(np.argmax(np.diag(symmetric)))]
        norm = np.linalg.norm(axis)
        if norm < 1e-12:
            return np.zeros(3)
        axis = axis / norm
        # The log map is two-valued at pi; pick the sign consistent with skew.
        if float(axis @ skew) < 0.0:
            axis = -axis
        return angle * axis

    return angle / (2.0 * np.sin(angle)) * skew
