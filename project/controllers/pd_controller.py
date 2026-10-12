"""Low-level joint-space PD controller.

Runs at the MuJoCo integration rate and tracks the joint targets produced by
the MPC, optionally adding the MPC feed-forward torque.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Joint-name substrings used to sort joints into limb groups. A humanoid's leg
# joints carry the body mass and need far stiffer gains than the arms, so the
# gains are assigned per group rather than per joint. The first matching token
# wins, so the order here resolves ambiguous names.
_LIMB_TOKENS: tuple[tuple[str, str], ...] = (
    ("shoulder", "arms"),
    ("elbow", "arms"),
    ("wrist", "arms"),
    ("hand", "arms"),
    ("arm", "arms"),
    ("torso", "torso"),
    ("waist", "torso"),
    ("spine", "torso"),
    ("pelvis", "torso"),
    ("hip", "legs"),
    ("knee", "legs"),
    ("ankle", "legs"),
    ("foot", "legs"),
    ("leg", "legs"),
)

# Group used for joints whose name matches none of the tokens above. Legs are
# the safe default: too much stiffness merely tracks hard, too little lets the
# robot collapse under its own weight.
_DEFAULT_LIMB = "legs"


@dataclass
class PDGains:
    """Per-joint proportional and derivative gains for the Unitree H1.

    Gains are grouped by limb because the leg joints carry the body mass and
    need far stiffer tracking than the arms.
    """

    kp_legs: float = 400.0
    kd_legs: float = 40.0
    kp_torso: float = 300.0
    kd_torso: float = 30.0
    kp_arms: float = 60.0
    kd_arms: float = 3.0
    torque_limit: float = 200.0  # N*m

    # Optional clamp on the position error before it is multiplied by kp. A
    # step change in the target (a new MPC solution, a reset) would otherwise
    # produce a torque spike that the actuators cannot deliver and that upsets
    # the integrator. ``None`` disables the clamp.
    max_position_error: float | None = None  # rad

    def for_group(self, group: str) -> tuple[float, float]:
        """Return the ``(kp, kd)`` pair for the limb group ``group``.

        Raises:
            KeyError: if ``group`` is not one of legs / torso / arms.
        """
        table = {
            "legs": (self.kp_legs, self.kd_legs),
            "torso": (self.kp_torso, self.kd_torso),
            "arms": (self.kp_arms, self.kd_arms),
        }
        if group not in table:
            raise KeyError(f"unknown limb group {group!r}; expected one of {sorted(table)}")
        return table[group]


def classify_joint(joint_name: str) -> str:
    """Map a joint name onto a limb group ("legs", "torso" or "arms").

    Matching is case-insensitive and substring based, so it works with the
    menagerie H1 names (``left_knee``, ``torso``, ``right_elbow``) as well as
    the URDF-style ``*_joint`` variants.
    """
    lowered = joint_name.lower()
    for token, group in _LIMB_TOKENS:
        if token in lowered:
            return group
    return _DEFAULT_LIMB


class PDController:
    """Joint PD tracking with optional feed-forward torque.

    The control law is a decoupled, per-joint PD law plus a feed-forward term:

        tau = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff

    It has no internal state, so it can be called at any rate; the environment
    runs it once per MuJoCo integration step while the MPC above it replans at
    a slower control rate. Gains are vectors, so every joint can be tuned
    independently even though they default to per-limb values.
    """

    def __init__(
        self,
        num_joints: int | None = None,
        gains: PDGains | None = None,
        joint_names: list[str] | None = None,
    ) -> None:
        """Build the per-joint gain vectors.

        Args:
            num_joints: number of actuated joints. Optional when
                ``joint_names`` is given, in which case it must agree with it.
            gains: limb-grouped gains; defaults to :class:`PDGains`.
            joint_names: actuated joint names in actuator order. When supplied,
                each joint is assigned the gains of the limb group its name
                implies. Without it every joint gets the leg gains, and the
                caller is expected to refine them with :meth:`set_gains`.

        Raises:
            ValueError: if neither argument is given, or if they disagree.
        """
        if num_joints is None and joint_names is None:
            raise ValueError("PDController needs either num_joints or joint_names")
        if joint_names is not None:
            if num_joints is not None and num_joints != len(joint_names):
                raise ValueError(
                    f"num_joints ({num_joints}) does not match "
                    f"len(joint_names) ({len(joint_names)})"
                )
            num_joints = len(joint_names)
        if num_joints < 0:
            raise ValueError(f"num_joints must be non-negative, got {num_joints}")

        self.num_joints = int(num_joints)
        self.gains = gains or PDGains()
        self.joint_names = list(joint_names) if joint_names is not None else []

        # Expand the limb-grouped gains into one entry per joint. Without joint
        # names every joint falls back to the default group.
        kp_default, kd_default = self.gains.for_group(_DEFAULT_LIMB)
        self.kp = np.full(self.num_joints, kp_default, dtype=float)
        self.kd = np.full(self.num_joints, kd_default, dtype=float)
        for i, name in enumerate(self.joint_names):
            self.kp[i], self.kd[i] = self.gains.for_group(classify_joint(name))

    def compute_torque(
        self,
        desired_positions: np.ndarray,
        current_positions: np.ndarray,
        current_velocities: np.ndarray,
        desired_velocities: np.ndarray | None = None,
        torque_feedforward: np.ndarray | None = None,
    ) -> np.ndarray:
        """Return the joint torques that track ``desired_positions``.

            tau = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff

        Args:
            desired_positions: target joint positions ``q_des`` (rad).
            current_positions: measured joint positions ``q`` (rad).
            current_velocities: measured joint velocities ``qd`` (rad/s).
            desired_velocities: target joint velocities ``qd_des`` (rad/s).
                Defaults to zero, which makes the derivative term pure damping
                — the right choice when holding a posture.
            torque_feedforward: additional torque ``tau_ff`` (N*m), typically
                the MPC's inverse-dynamics solution or a gravity-compensation
                term. The PD part then only has to reject the residual error.

        Returns:
            Torque vector of shape ``(num_joints,)``, clipped element-wise to
            ``+/- gains.torque_limit``.

        Raises:
            ValueError: if any input does not have shape ``(num_joints,)``.
        """
        q_des = self._as_vector(desired_positions, "desired_positions")
        q = self._as_vector(current_positions, "current_positions")
        qd = self._as_vector(current_velocities, "current_velocities")

        # Proportional term: pull each joint towards its target. The error is
        # optionally clamped so a discontinuous target cannot demand a torque
        # spike the actuator could never deliver anyway.
        position_error = q_des - q
        max_error = self.gains.max_position_error
        if max_error is not None:
            position_error = np.clip(position_error, -max_error, max_error)

        # Derivative term: damp the joint towards the desired velocity. With the
        # default zero target this is pure viscous damping.
        if desired_velocities is None:
            velocity_error = -qd
        else:
            velocity_error = self._as_vector(desired_velocities, "desired_velocities") - qd

        torque = self.kp * position_error + self.kd * velocity_error

        # Feed-forward term: added after the feedback so the saturation below
        # bounds the total commanded torque, not just the feedback part.
        if torque_feedforward is not None:
            torque = torque + self._as_vector(torque_feedforward, "torque_feedforward")

        # Saturate to the actuator's torque envelope. The caller may clip again
        # against per-actuator limits from the model; this bound is the generic
        # one carried by the gains.
        return np.clip(torque, -self.gains.torque_limit, self.gains.torque_limit)

    def set_gains(self, kp: np.ndarray, kd: np.ndarray) -> None:
        """Override the per-joint gain vectors (used for gain scheduling).

        Raises:
            ValueError: if either vector does not have shape ``(num_joints,)``.
        """
        self.kp = self._as_vector(kp, "kp")
        self.kd = self._as_vector(kd, "kd")

    def _as_vector(self, values: np.ndarray, name: str) -> np.ndarray:
        """Coerce ``values`` to a float array of shape ``(num_joints,)``."""
        array = np.asarray(values, dtype=float).ravel()
        if array.size != self.num_joints:
            raise ValueError(
                f"{name} has {array.size} element(s), expected {self.num_joints}"
            )
        return array
