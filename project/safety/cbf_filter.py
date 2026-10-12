"""Control barrier function safety filter for humanoid joint torques.

Projects a nominal torque onto the set of torques that keep the centre of mass
inside the support polygon, by solving a small quadratic program with CVXPY:

    min_u   || u - u_nominal ||^2
    s.t.    higher-order CBF inequalities on the support box
            u_min <= u <= u_max

This is a research prototype. The map from torque to horizontal CoM
acceleration is a constant linearisation (stance feet planted, home pose),
not a full centroidal QP recomputed every tick.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Sequence

import numpy as np

try:
    import cvxpy as cp
except ImportError:  # pragma: no cover - keeps the scaffold importable
    cp = None

try:
    import mujoco
except ImportError:  # pragma: no cover
    mujoco = None


# ---------------------------------------------------------------------------
# Mathematics
# ---------------------------------------------------------------------------
#
# Barrier. Approximate the support polygon by an axis-aligned box in the
# horizontal plane, shrunk by a margin. The centre of mass c = (c_x, c_y)
# is safe when it lies inside that box. Each face is one barrier; for the
# +x face
#
#     h(c) = x_hi - c_x ,          h >= 0  inside the box.
#
# h depends on position only, and torque does not appear until the second
# derivative, so this is a relative-degree-2 barrier. A linear class-K pair
# (alpha_1, alpha_2) gives the standard higher-order CBF condition: the
# first auxiliary function and its own barrier inequality
#
#     psi_1     = h_dot + alpha_1 h
#     psi_1_dot + alpha_2 psi_1 >= 0.
#
# Expanding with h_dot = -v_x and h_ddot = -c_ddot_x yields
#
#     -c_ddot_x - (alpha_1 + alpha_2) v_x + alpha_1 alpha_2 (x_hi - c_x) >= 0.
#
# The lower face h = c_x - x_lo flips both signs. The y faces are identical
# with (c_y, v_y, c_ddot_y).
#
# Torque enters through a linear stance model, frozen at the home pose,
#
#     c_ddot_xy = B u + a0 ,     B in R^{2 x nu},
#
# where u is the joint-torque vector. B is the CoM Jacobian times the
# torque-to-acceleration map of the floating base with both feet held by
# their translational contact Jacobian (see ``stance_com_acceleration_map``).
# Joint torques are internal forces on a free body; without that stance
# constraint they cannot accelerate the CoM at all, and the filter would
# have nothing to enforce.
#
# Substituting c_ddot_x = B_x u + a0_x into the +x inequality and rearranging,
#
#     -(B_x u + a0_x)  >=  (alpha_1 + alpha_2) v_x - alpha_1 alpha_2 (x_hi - c_x).
#
# The right-hand side is state-dependent and is the only thing that changes
# between ticks. The four faces plus the actuator box
#
#     u_min <= u <= u_max
#
# are linear, and the objective is strictly convex, so the program is a QP.
# When the CoM is already outside and no torque inside the actuator limits
# can satisfy the inequalities, a non-negative slack relaxes them. The hard
# QP is always attempted first, so a feasible problem is solved exactly as
# min ||u - u_nominal||^2 with the slack fixed at zero.


@dataclass
class BarrierSpec:
    """One control barrier function and its class-K extension.

    ``grad_h`` is the gradient of ``h`` with respect to whatever coordinates
    the barrier was written in (joint positions, a link position, ...). The
    torque QP below does not use these callables: its CoM constraint is the
    higher-order form derived above, whose input Jacobian is the constant
    matrix ``B``.
    """

    name: str
    h: Callable[[dict[str, np.ndarray]], float]
    grad_h: Callable[[dict[str, np.ndarray]], np.ndarray]
    alpha: float = 10.0  # linear class-K gain
    relative_degree: int = 2
    slack_penalty: float | None = None  # None => hard constraint


@dataclass
class CBFConfig:
    """Weights, support box and solver options for the torque filter."""

    solver: str = "OSQP"
    # Class-K gains of the relative-degree-2 CoM barrier. In the interior the
    # acceleration the filter will still allow is about alpha1*alpha2*h, so
    # very small gains treat a normal balancing torque as a violation and
    # rewrite it. Larger gains leave that torque alone until the CoM is
    # actually close to a face.
    alpha1: float = 12.0
    alpha2: float = 12.0
    # Support box in the world horizontal plane, metres. The home CoM of the
    # H1 sits near (0.03, 0.00), so these defaults leave it inside.
    support_half_extents: tuple[float, float] = (0.08, 0.12)
    support_center: tuple[float, float] = (0.0, 0.0)
    margin: float = 0.02  # metres shaved off each face
    # Largest horizontal acceleration (m/s^2) the filter may demand. The
    # stance map is a small-acceleration linearisation: a 300 N shove makes
    # the raw barrier ask for tens of m/s^2, the QP then saturates the hips,
    # and that torque pitches the robot into a flip. Clamping the request
    # keeps the correction inside the region where the linearisation holds.
    max_recovery_acceleration: float = 4.0
    # How far, in N*m, the safe torque may move from the nominal command.
    # The stance linearisation will otherwise spend the whole actuator range
    # trying to meet the barrier, and that torque flexes the knees and throws
    # the robot over. The residual barrier violation is left to the slack.
    max_torque_deviation: float = 40.0
    # Penalty on the slack used only when the hard QP is infeasible.
    default_slack_penalty: float = 1e4
    fail_safe: str = "hold"  # "hold" | "zero" | "raise"
    # Optional actuator box. ``None`` means "read it from the MJCF" when a
    # model is supplied, otherwise +/- torque_limit.
    torque_limit: float = 200.0  # N*m
    u_min: np.ndarray | None = None
    u_max: np.ndarray | None = None


class CBFFilter:
    """Minimally invasive torque filter keeping the CoM inside the support polygon.

    The barrier is the axis-aligned box of the polygon last passed to
    :meth:`set_support_polygon`. While both feet are down that box covers both
    soles. During a swing it covers only the stance sole, so the support
    region changes with the step.

    Inputs of :meth:`safe_torque` are the nominal torque command and the
    centre-of-mass position and velocity. The output is a torque of the same
    shape, inside the actuator limits, and as close as possible to the nominal
    command in the Euclidean norm.
    """

    def __init__(
        self,
        barriers: Sequence[BarrierSpec] = (),
        action_dim: int = 0,
        config: CBFConfig | None = None,
        model: Any = None,
        control_matrix: np.ndarray | None = None,
    ) -> None:
        """Build the parametric QP.

        Args:
            barriers: extra barriers evaluated for logging. They are not rows
                of the torque QP; that QP is the CoM support constraint.
            action_dim: length of the torque vector. Inferred from ``model``
                or ``control_matrix`` when those are given.
            config: box, gains and solver. Defaults to :class:`CBFConfig`.
            model: compiled MuJoCo model. When given, the torque-to-CoM map
                ``B`` and the actuator limits are linearised from it.
            control_matrix: explicit ``B`` of shape ``(2, nu)`` such that
                ``c_ddot_xy = B @ u``. Overrides the linearisation.
        """
        self.barriers = list(barriers)
        self.config = config or CBFConfig()
        self.model = model
        self._last_safe_action: np.ndarray | None = None
        self._last_status = "unsolved"

        if control_matrix is not None:
            control_matrix = np.asarray(control_matrix, dtype=float)
            if control_matrix.ndim != 2 or control_matrix.shape[0] != 2:
                raise ValueError(
                    f"control_matrix must have shape (2, nu), got {control_matrix.shape}"
                )
            action_dim = int(control_matrix.shape[1])
            self._B = control_matrix.copy()
            self._a0 = np.zeros(2)
            self._stance_com_xy = np.zeros(2)
            limits = _limits_from_config(self.config, action_dim)
        elif model is not None:
            self._B, self._a0, self._stance_com_xy, limits = stance_com_acceleration_map(model)
            # Centre the box on the standing CoM. The default (0, 0) sits on
            # the edge of the H1's feet, so the barrier is already tight at reset.
            if self.config.support_center == (0.0, 0.0):
                self.config.support_center = (
                    float(self._stance_com_xy[0]),
                    float(self._stance_com_xy[1]),
                )
            action_dim = int(self._B.shape[1])
            if self.config.u_min is not None or self.config.u_max is not None:
                limits = _limits_from_config(self.config, action_dim, fallback=limits)
        else:
            self._B = np.zeros((2, action_dim))
            self._a0 = np.zeros(2)
            self._stance_com_xy = np.zeros(2)
            limits = _limits_from_config(self.config, action_dim)

        self.action_dim = int(action_dim)
        self._u_min = limits[0]
        self._u_max = limits[1]
        self._hard = None
        self._soft = None
        self._dev_param = None
        self._u_min_param = None
        self._u_max_param = None
        if self.action_dim > 0 and cp is not None:
            self._build_qp()

    # ------------------------------------------------------------------
    # QP
    # ------------------------------------------------------------------
    def _build_qp(self) -> None:
        """Assemble the hard QP and the slacked fallback, once.

        Decision variable ``u`` is the torque. Parameters are the nominal
        torque, the actuator box and the four CBF right-hand sides, so a tick
        only writes numbers and re-solves. OSQP warm-starts from the previous
        primal solution.
        """
        nu = self.action_dim
        u = cp.Variable(nu, name="torque")
        slack = cp.Variable(4, nonneg=True, name="cbf_slack")
        u_nom = cp.Parameter(nu, name="u_nominal")
        u_min = cp.Parameter(nu, name="u_min")
        u_max = cp.Parameter(nu, name="u_max")
        rhs = cp.Parameter(4, name="cbf_rhs")

        # B is constant. The bias acceleration a0 is folded into rhs, so these
        # rows constrain c_ddot = B u + a0.
        bx = self._B[0]
        by = self._B[1]
        # Face order: +x, -x, +y, -y. See the module comment for the sign of
        # each inequality; the slack is added on the feasible side so s = 0
        # recovers the hard constraint.
        cbf_rows = [
            -bx @ u + slack[0] >= rhs[0],
            bx @ u + slack[1] >= rhs[1],
            -by @ u + slack[2] >= rhs[2],
            by @ u + slack[3] >= rhs[3],
        ]
        hard_rows = [
            -bx @ u >= rhs[0],
            bx @ u >= rhs[1],
            -by @ u >= rhs[2],
            by @ u >= rhs[3],
        ]
        box = [u >= u_min, u <= u_max]
        # Keep the correction local. A feasible barrier that only exists at
        # the actuator stops is not a safe torque on the real robot.
        deviation = cp.Parameter(nonneg=True, name="torque_deviation")
        near_nominal = [u >= u_nom - deviation, u <= u_nom + deviation]
        tracking = cp.sum_squares(u - u_nom)

        self._u = u
        self._slack = slack
        self._u_nom_param = u_nom
        self._u_min_param = u_min
        self._u_max_param = u_max
        self._rhs_param = rhs
        self._hard = cp.Problem(cp.Minimize(tracking), box + near_nominal + hard_rows)
        penalty = float(self.config.default_slack_penalty)
        self._soft = cp.Problem(
            cp.Minimize(tracking + penalty * cp.sum_squares(slack)),
            box + near_nominal + cbf_rows,
        )
        self._dev_param = deviation
        self._dev_param.value = float(self.config.max_torque_deviation)
        # Bounds never change unless the caller overrides them, but they are
        # parameters so a later set_actuator_limits() does not rebuild.
        self._u_min_param.value = self._u_min
        self._u_max_param.value = self._u_max

    def safe_torque(
        self,
        u_nominal: np.ndarray,
        com_position: np.ndarray,
        com_velocity: np.ndarray,
    ) -> np.ndarray:
        """Return the safe torque closest to ``u_nominal``.

        Args:
            u_nominal: nominal joint torques, shape ``(nu,)``, in N*m.
            com_position: centre-of-mass position, world frame, shape ``(3,)``
                or ``(2,)``. Only the horizontal part enters the barrier.
            com_velocity: centre-of-mass velocity, same frame and shapes.

        Returns:
            Torque vector of shape ``(nu,)``, satisfying the actuator limits.
            Equals ``u_nominal`` (clipped to those limits) when that command
            already satisfies the CBF inequalities.

        Raises:
            ImportError: if cvxpy is not installed.
            ValueError: if ``u_nominal`` does not have length ``action_dim``,
                or if the QP fails and ``config.fail_safe`` is ``"raise"``.
        """
        if self.action_dim == 0:
            return np.zeros(0)
        if cp is None or self._hard is None:
            raise ImportError(
                "cvxpy is required to solve the CBF quadratic program; "
                "install it with `pip install -r requirements.txt`."
            )

        u_nominal = np.asarray(u_nominal, dtype=float).reshape(-1)
        if u_nominal.size != self.action_dim:
            raise ValueError(
                f"u_nominal has length {u_nominal.size}, expected {self.action_dim}"
            )
        com = np.asarray(com_position, dtype=float).reshape(-1)
        vel = np.asarray(com_velocity, dtype=float).reshape(-1)
        if com.size < 2 or vel.size < 2:
            raise ValueError("com_position and com_velocity need at least x and y")

        rhs = self._cbf_rhs(com[:2], vel[:2])
        self._u_nom_param.value = u_nominal
        self._rhs_param.value = rhs
        if self._dev_param is not None:
            self._dev_param.value = float(self.config.max_torque_deviation)
        # Warm start at the nominal command. OSQP uses the previous factorisation
        # as well once the sparsity pattern has been seen.
        self._u.value = np.clip(u_nominal, self._u_min, self._u_max)

        self._hard.solve(solver=self._solver(), warm_start=True, verbose=False)
        if self._hard.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) and self._u.value is not None:
            safe = np.asarray(self._u.value, dtype=float).reshape(-1).copy()
            self._last_status = "optimal"
        else:
            safe = self._solve_relaxed(u_nominal)

        # The QP's box constraint is the actuator limit; clip once more so a
        # slightly inaccurate solve cannot command past the MJCF ctrlrange.
        safe = np.clip(safe, self._u_min, self._u_max)
        self._last_safe_action = safe
        return safe

    def _solve_relaxed(self, u_nominal: np.ndarray) -> np.ndarray:
        """Re-solve with a slack when the hard CBF is infeasible.

        The objective is no longer exactly ``||u - u_nominal||^2``: a penalty
        on the slack is added so the filter still returns a torque inside the
        actuator limits instead of failing closed.
        """
        self._slack.value = np.zeros(4)
        self._soft.solve(solver=self._solver(), warm_start=True, verbose=False)
        if self._soft.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) and self._u.value is not None:
            self._last_status = "relaxed"
            return np.asarray(self._u.value, dtype=float).reshape(-1).copy()
        return self._fail_safe(u_nominal)

    def _fail_safe(self, u_nominal: np.ndarray) -> np.ndarray:
        """Last resort when both QPs fail: stay inside the actuator box."""
        self._last_status = "fail_safe"
        mode = self.config.fail_safe
        if mode == "raise":
            raise RuntimeError("CBF quadratic program failed and fail_safe='raise'")
        if mode == "zero":
            return np.clip(np.zeros(self.action_dim), self._u_min, self._u_max)
        # "hold": repeat the previous safe torque, or clip the nominal command
        # if this is the first tick.
        if self._last_safe_action is not None:
            return self._last_safe_action.copy()
        return np.clip(u_nominal, self._u_min, self._u_max)

    def _solver(self) -> Any:
        """Resolve ``config.solver`` to a cvxpy solver constant."""
        name = self.config.solver.upper()
        if name == "OSQP" and "OSQP" in cp.installed_solvers():
            return cp.OSQP
        if name == "CLARABEL" and "CLARABEL" in cp.installed_solvers():
            return cp.CLARABEL
        if name == "SCS" and "SCS" in cp.installed_solvers():
            return cp.SCS
        installed = cp.installed_solvers()
        for candidate in ("OSQP", "CLARABEL", "SCS"):
            if candidate in installed:
                return getattr(cp, candidate)
        raise RuntimeError(f"no QP solver installed; cvxpy reports {installed}")

    def _cbf_rhs(self, com_xy: np.ndarray, vel_xy: np.ndarray) -> np.ndarray:
        """Right-hand side of the four HOCBF inequalities at this state.

        Order is ``(+x, -x, +y, -y)``. For the upper face
        ``h = limit - c`` the inequality ``-B u >= rhs`` has

            rhs = a0 - L,   L = clip( -(alpha_1+alpha_2) v + alpha_1 alpha_2 h )

        and for the lower face ``h = c - limit`` the inequality ``B u >= rhs``
        has

            rhs = L - a0,   L = clip( -(alpha_1+alpha_2) v - alpha_1 alpha_2 h ).

        ``clip`` limits ``L`` to ``± max_recovery_acceleration``.

        A negative ``rhs`` is a constraint that is automatically satisfied by
        ``u = 0``; it becomes positive when the CoM is close to a face and
        still moving towards it, which is when the filter must intervene.
        """
        cfg = self.config
        center = np.asarray(cfg.support_center, dtype=float)
        half = np.asarray(cfg.support_half_extents, dtype=float) - cfg.margin
        if np.any(half <= 0.0):
            raise ValueError(
                "support_half_extents must be larger than margin, "
                f"got extents {cfg.support_half_extents} and margin {cfg.margin}"
            )
        alpha_sum = cfg.alpha1 + cfg.alpha2
        alpha_prod = cfg.alpha1 * cfg.alpha2
        rhs = np.zeros(4)
        for axis in (0, 1):
            upper = center[axis] + half[axis]
            lower = center[axis] - half[axis]
            h_hi = upper - com_xy[axis]
            h_lo = com_xy[axis] - lower
            v = vel_xy[axis]
            # a0 shifts the constraint so a torque that only cancels gravity
            # (c_ddot ≈ 0) is not treated as a push toward the boundary.
            # Upper face requires c_ddot <= L_hi, lower face c_ddot >= L_lo.
            # Clamp both so a fast shove cannot demand an acceleration the
            # hips can only produce by pitching the whole body over.
            a_max = float(cfg.max_recovery_acceleration)
            limit_hi = min(max(-alpha_sum * v + alpha_prod * h_hi, -a_max), a_max)
            limit_lo = min(max(-alpha_sum * v - alpha_prod * h_lo, -a_max), a_max)
            rhs[2 * axis] = self._a0[axis] - limit_hi
            rhs[2 * axis + 1] = limit_lo - self._a0[axis]
        return rhs

    def set_actuator_limits(self, u_min: np.ndarray, u_max: np.ndarray) -> None:
        """Replace the torque box ``u_min <= u <= u_max``."""
        u_min = np.asarray(u_min, dtype=float).reshape(-1)
        u_max = np.asarray(u_max, dtype=float).reshape(-1)
        if u_min.size != self.action_dim or u_max.size != self.action_dim:
            raise ValueError(
                f"actuator limits must have length {self.action_dim}, "
                f"got {u_min.size} and {u_max.size}"
            )
        if np.any(u_min > u_max):
            raise ValueError("u_min must be <= u_max elementwise")
        self._u_min = u_min.copy()
        self._u_max = u_max.copy()
        if self._u_min_param is not None:
            self._u_min_param.value = self._u_min
            self._u_max_param.value = self._u_max

    # ------------------------------------------------------------------
    # Gymnasium-style wrapper used by main_demo
    # ------------------------------------------------------------------
    def filter(
        self,
        state: dict[str, np.ndarray],
        u_nom: np.ndarray,
    ) -> tuple[np.ndarray, dict[str, float]]:
        """Filter a nominal torque using the CoM stored in ``state``.

        ``state`` must contain ``com`` and ``com_velocity`` (as returned by
        ``IndustrialHumanoidEnv.get_state``). ``u_nom`` is a torque, not a
        joint-position target.

        Returns:
            ``(u_safe, barriers)`` where ``barriers`` maps each support-box
            face to its value of ``h``. Non-negative means the CoM is inside
            that face.
        """
        if "com" not in state or "com_velocity" not in state:
            raise KeyError("state must contain 'com' and 'com_velocity'")
        u_safe = self.safe_torque(u_nom, state["com"], state["com_velocity"])
        return u_safe, self.barrier_values(state)

    def barrier_values(self, state: dict[str, np.ndarray]) -> dict[str, float]:
        """Evaluate every barrier without solving the QP (for logging)."""
        values: dict[str, float] = {}
        if "com" in state:
            values.update(self.com_barrier_values(state["com"]))
        for spec in self.barriers:
            values[spec.name] = float(spec.h(state))
        return values

    def set_support_polygon(self, vertices: np.ndarray) -> None:
        """Aim the CoM barrier at the feet that are currently on the ground.

        ``vertices`` is an ``(n, 2)`` array of world-frame sole corners. The
        filter's constraint is a box, so the polygon is reduced to its
        axis-aligned bounds. For the H1 soles, which are aligned with the
        world axes, those bounds are the polygon. The QP reads the center and
        the half extents on the next solve; it does not need to be rebuilt.
        """
        points = np.asarray(vertices, dtype=float).reshape(-1, 2)
        if points.shape[0] == 0:
            raise ValueError("support polygon needs at least one vertex")
        low = points.min(axis=0)
        high = points.max(axis=0)
        center = 0.5 * (low + high)
        half = 0.5 * (high - low)
        # A face inside the margin would make the safe set empty and the
        # filter would push the CoM even when it is standing on the sole.
        half = np.maximum(half, float(self.config.margin) + 1e-3)
        self.config.support_center = (float(center[0]), float(center[1]))
        self.config.support_half_extents = (float(half[0]), float(half[1]))

    def com_barrier_values(self, com_position: np.ndarray) -> dict[str, float]:
        """Signed distance of the CoM to each face of the support box.

        Positive means the CoM is inside that face. The four names match the
        inequality order in the QP.
        """
        com = np.asarray(com_position, dtype=float).reshape(-1)
        center = np.asarray(self.config.support_center, dtype=float)
        half = np.asarray(self.config.support_half_extents, dtype=float) - self.config.margin
        names = ("support_x_upper", "support_x_lower", "support_y_upper", "support_y_lower")
        values = {}
        for axis, (hi_name, lo_name) in enumerate(
            ((names[0], names[1]), (names[2], names[3]))
        ):
            upper = center[axis] + half[axis]
            lower = center[axis] - half[axis]
            values[hi_name] = float(upper - com[axis])
            values[lo_name] = float(com[axis] - lower)
        return values

    def is_safe(self, state: dict[str, np.ndarray]) -> bool:
        """True when every barrier is non-negative at the current state."""
        values = self.barrier_values(state)
        if not values:
            return True
        return all(value >= 0.0 for value in values.values())

    @property
    def control_matrix(self) -> np.ndarray:
        """The ``(2, nu)`` map ``c_ddot_xy = B @ u`` used by the CBF."""
        return self._B.copy()

    @property
    def status(self) -> str:
        """``optimal``, ``relaxed`` or ``fail_safe`` from the last solve."""
        return self._last_status


def stance_com_acceleration_map(
    model: Any,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[np.ndarray, np.ndarray]]:
    """Linearise ``c_ddot_xy = B u + a0`` at the home pose with the feet planted.

    The KKT system of the constrained dynamics at zero velocity is

        [ M    J_c^T ] [ qdd ] = [ S^T u - bias ]
        [ J_c   0    ] [ -λ  ]   [ 0            ]

    with ``J_c`` the stacked translational Jacobians of the ankle bodies, so
    the feet cannot slide. The columns of ``B`` are the horizontal CoM
    acceleration from each unit torque. ``a0`` is the same acceleration when
    the torque is zero, i.e. gravity acting through the stance constraint.
    ``com_xy`` is the home centre of mass, used to centre the support box.

    Returns:
        ``B`` of shape ``(2, nu)``, ``a0`` of shape ``(2,)``, ``com_xy`` of
        shape ``(2,)``, and ``(u_min, u_max)`` from the MJCF actuator
        ctrlrange (unlimited actuators get +/- 1e6).
    """
    if mujoco is None:  # pragma: no cover
        raise ImportError("mujoco is required to linearise the CoM acceleration map")

    data = mujoco.MjData(model)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    nu, nv = int(model.nu), int(model.nv)
    joint_ids = np.array([int(model.actuator_trnid[i, 0]) for i in range(nu)], dtype=int)
    dof = model.jnt_dofadr[joint_ids].astype(int)

    jac_com = _com_jacobian(model, data)
    jac_c = _foot_contact_jacobian(model, data)
    mass_matrix = np.zeros((nv, nv))
    mujoco.mj_fullM(model, data, mass_matrix)

    nc = jac_c.shape[0]
    kkt = np.zeros((nv + nc, nv + nc))
    kkt[:nv, :nv] = mass_matrix
    kkt[:nv, nv:] = jac_c.T
    kkt[nv:, :nv] = jac_c
    # A tiny negative mass on the multiplier block keeps the KKT matrix from
    # being exactly singular when the two feet are nearly coplanar.
    kkt[nv:, nv:] = -1e-8 * np.eye(nc)

    rhs = np.zeros((nv + nc, nu))
    for column, dof_index in enumerate(dof):
        rhs[int(dof_index), column] = 1.0  # S^T: this actuator's unit torque
    solution, *_ = np.linalg.lstsq(kkt, rhs, rcond=1e-8)
    # Horizontal CoM acceleration produced by each unit torque.
    control_matrix = jac_com[:2] @ solution[:nv]

    # Gravity (and any other bias) with the feet planted and u = 0.
    bias_rhs = np.zeros(nv + nc)
    bias_rhs[:nv] = -data.qfrc_bias
    bias_solution, *_ = np.linalg.lstsq(kkt, bias_rhs, rcond=1e-8)
    a0 = jac_com[:2] @ bias_solution[:nv]
    root = int(model.jnt_bodyid[0]) if model.njnt else 1
    com_xy = np.asarray(data.subtree_com[root][:2], dtype=float).copy()

    limited = model.actuator_ctrllimited.astype(bool)
    ctrl_range = np.asarray(model.actuator_ctrlrange, dtype=float)
    u_min = np.where(limited, ctrl_range[:, 0], -1e6)
    u_max = np.where(limited, ctrl_range[:, 1], 1e6)
    return control_matrix, a0, com_xy, (u_min, u_max)


def _com_jacobian(model: Any, data: Any) -> np.ndarray:
    """Mass-weighted CoM Jacobian, shape ``(3, nv)``, over the robot subtree."""
    nv = int(model.nv)
    jacobian = np.zeros((3, nv))
    body_jac = np.zeros((3, nv))
    root = int(model.jnt_bodyid[0]) if model.njnt else 1
    if model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
        root = 1 if model.nbody > 1 else 0
    members = {root}
    for body_id in range(root + 1, model.nbody):
        if int(model.body_parentid[body_id]) in members:
            members.add(body_id)
    total_mass = 0.0
    for body_id in members:
        mass = float(model.body_mass[body_id])
        if mass <= 0.0:
            continue
        mujoco.mj_jacBodyCom(model, data, body_jac, None, int(body_id))
        jacobian += mass * body_jac
        total_mass += mass
    if total_mass > 0.0:
        jacobian /= total_mass
    return jacobian


def _foot_contact_jacobian(model: Any, data: Any) -> np.ndarray:
    """Stacked translational Jacobians of every ankle/foot body, shape ``(3 n, nv)``."""
    rows = []
    for body_id in range(model.nbody):
        name = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or "").lower()
        if "ankle" not in name and "foot" not in name:
            continue
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jacBodyCom(model, data, jacp, None, int(body_id))
        rows.append(jacp)
    if not rows:
        # No feet: a zero-row Jacobian would make the KKT system unconstrained.
        # Return a dummy row of zeros so the caller still gets a finite B; the
        # resulting map is the free-floating one, which is nearly zero.
        return np.zeros((1, model.nv))
    return np.vstack(rows)


def _limits_from_config(
    config: CBFConfig,
    action_dim: int,
    fallback: tuple[np.ndarray, np.ndarray] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Actuator box from the config, or a symmetric torque limit."""
    if config.u_min is not None and config.u_max is not None:
        u_min = np.asarray(config.u_min, dtype=float).reshape(-1)
        u_max = np.asarray(config.u_max, dtype=float).reshape(-1)
        if u_min.size != action_dim or u_max.size != action_dim:
            raise ValueError(
                f"config actuator limits have lengths {u_min.size}, {u_max.size}; "
                f"expected {action_dim}"
            )
        return u_min, u_max
    if fallback is not None:
        return fallback
    limit = abs(float(config.torque_limit))
    return np.full(action_dim, -limit), np.full(action_dim, limit)


# ----------------------------------------------------------------------
# Barrier library (evaluation / logging; the torque QP uses the box above)
# ----------------------------------------------------------------------
def support_polygon_barrier(
    margin: float = 0.05,
    half_extents: tuple[float, float] = (0.08, 0.12),
    center: tuple[float, float] = (0.0, 0.0),
) -> BarrierSpec:
    """``h(x) =`` smallest signed distance from the CoM to the box boundary.

    Positive inside the box. ``grad_h`` is the gradient with respect to the
    horizontal CoM of the active (closest) face.
    """

    def h(state: dict[str, np.ndarray]) -> float:
        return float(min(_box_margins(state["com"], half_extents, center, margin).values()))

    def grad_h(state: dict[str, np.ndarray]) -> np.ndarray:
        margins = _box_margins(state["com"], half_extents, center, margin)
        active = min(margins, key=margins.get)
        # dh/dc for each face. Only the horizontal CoM coordinates appear.
        table = {
            "support_x_upper": np.array([-1.0, 0.0]),
            "support_x_lower": np.array([1.0, 0.0]),
            "support_y_upper": np.array([0.0, -1.0]),
            "support_y_lower": np.array([0.0, 1.0]),
        }
        return table[active]

    return BarrierSpec(name="support_polygon", h=h, grad_h=grad_h, relative_degree=2)


def joint_limit_barrier(joint_index: int, lower: float, upper: float) -> BarrierSpec:
    """``h(x) = min(q - lower, upper - q)`` for a single joint.

    This is a position-level barrier used for logging. It is not a row of the
    torque QP, which only constrains the CoM and the actuator torque box.
    """

    def h(state: dict[str, np.ndarray]) -> float:
        q = float(np.asarray(state["joint_qpos"], dtype=float).reshape(-1)[joint_index])
        return min(q - lower, upper - q)

    def grad_h(state: dict[str, np.ndarray]) -> np.ndarray:
        q = np.asarray(state["joint_qpos"], dtype=float).reshape(-1)
        gradient = np.zeros(q.size)
        # The active side of the min() is the nearer limit.
        gradient[joint_index] = 1.0 if (q[joint_index] - lower) <= (upper - q[joint_index]) else -1.0
        return gradient

    return BarrierSpec(
        name=f"joint_limit_{joint_index}",
        h=h,
        grad_h=grad_h,
        relative_degree=2,
    )


def keepout_sphere_barrier(centre: np.ndarray, radius: float, link: str) -> BarrierSpec:
    """``h = ||p_link - centre|| - radius`` for an exclusion sphere.

    Reads ``state["body_positions"][link]``. If that entry is absent the
    barrier reports ``+inf`` (no evidence of a violation) rather than inventing
    a position. Not a row of the torque QP.
    """
    centre = np.asarray(centre, dtype=float).reshape(3)

    def _position(state: dict[str, np.ndarray]) -> np.ndarray | None:
        positions = state.get("body_positions")
        if positions is None or link not in positions:
            return None
        return np.asarray(positions[link], dtype=float).reshape(3)

    def h(state: dict[str, np.ndarray]) -> float:
        position = _position(state)
        if position is None:
            return float(np.inf)
        return float(np.linalg.norm(position - centre) - radius)

    def grad_h(state: dict[str, np.ndarray]) -> np.ndarray:
        position = _position(state)
        if position is None:
            return np.zeros(3)
        offset = position - centre
        norm = np.linalg.norm(offset)
        if norm < 1e-9:
            return np.zeros(3)
        return offset / norm

    return BarrierSpec(name=f"keepout_{link}", h=h, grad_h=grad_h, relative_degree=2)


def _box_margins(
    com: np.ndarray,
    half_extents: tuple[float, float],
    center: tuple[float, float],
    margin: float,
) -> dict[str, float]:
    """Signed distance to each face. Positive when ``com`` is inside."""
    c = np.asarray(com, dtype=float).reshape(-1)
    half = np.asarray(half_extents, dtype=float) - margin
    origin = np.asarray(center, dtype=float)
    return {
        "support_x_upper": float(origin[0] + half[0] - c[0]),
        "support_x_lower": float(c[0] - (origin[0] - half[0])),
        "support_y_upper": float(origin[1] + half[1] - c[1]),
        "support_y_lower": float(c[1] - (origin[1] - half[1])),
    }

