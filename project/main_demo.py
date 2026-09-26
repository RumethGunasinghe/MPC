"""Demonstration: MPC, PD and a CBF torque filter on the Unitree H1.

The control stack on every tick is

    state -> WholeBodyMPC -> PDController -> CBFFilter -> joint torques -> MuJoCo

The MPC holds the torso attitude recorded at reset and holds the centre of
mass over the middle of the feet. The PD layer turns the resulting joint targets
into a nominal torque, and the CBF filter projects that torque onto the set
that keeps the centre of mass inside the support box and inside the actuator
limits. The safe torque is what the simulator applies.

At t = 2 s a horizontal force is applied to the torso, standing in for a
heavy manipulation load. The viewer stays open until the window is closed.
COM position, COM velocity and the size of each safety intervention are
written to ``data/demo_log.npz`` when the window closes.

Usage:
    python main_demo.py
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from controllers.mpc_controller import MPCConfig, WholeBodyMPC
from controllers.pd_controller import PDController
from env.industrial_humanoid_env import EnvConfig, IndustrialHumanoidEnv
from safety.cbf_filter import CBFConfig, CBFFilter
from utils.logger import get_console_logger

LOG = get_console_logger("main_demo")

# A shove in +x at the torso. 300 N held for half a second is several times
# the moment the feet can resist, so the body rotates over the toe. 80 N for
# 0.2 s is enough to make the robot lean and the safety filter intervene,
# and the stance can still bring it back.
PUSH_TIME = 2.0  # seconds
PUSH_FORCE = np.array([80.0, 0.0, 0.0])  # newtons
PUSH_DURATION = 0.2  # seconds
PUSH_BODY_CANDIDATES = ("torso_link", "torso", "pelvis")


def build_stack(
    args: argparse.Namespace,
) -> tuple[IndustrialHumanoidEnv, WholeBodyMPC, CBFFilter, PDController]:
    """Load the humanoid and construct the three controllers."""
    env = IndustrialHumanoidEnv(
        EnvConfig(
            model_path=args.model_path,
            control_dt=args.control_dt,
            # The demo owns the viewer; the env must not open a second one.
            render_mode=None,
            reset_joint_noise=0.0,
            episode_seconds=1.0e6,
        )
    )
    mpc = WholeBodyMPC(env.model, MPCConfig(horizon=args.horizon, dt=args.control_dt))
    cbf = CBFFilter(model=env.model, config=CBFConfig())
    pd = PDController(joint_names=env.joint_names)
    return env, mpc, cbf, pd


def sole_center_xy(env: IndustrialHumanoidEnv) -> np.ndarray:
    """Horizontal midpoint of the sole collision geoms, in the world frame.

    The home centre of mass sits almost over the heels. Holding that point
    leaves no room to sway backward, so a push recovery falls off the back
    of the feet. The middle of the soles is the point the stance can support.
    """
    import mujoco

    model, data = env.model, env.data
    ankles = []
    for body_id in range(int(model.nbody)):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        if "ankle" in name:
            ankles.append(np.asarray(data.xpos[body_id], dtype=float))
    if not ankles:
        return np.asarray(env.get_com_position()[:2], dtype=float)
    # The sole runs from about 3.5 cm behind the ankle to 14 cm in front, so
    # its middle is roughly 4 cm ahead of the ankle joint.
    center = np.mean(ankles, axis=0)
    center[0] += 0.04
    return center[:2]


def stand_reference(
    env: IndustrialHumanoidEnv, state: dict[str, np.ndarray]
) -> dict[str, np.ndarray]:
    """Hold the torso attitude, and hold the CoM over the middle of the feet."""
    com = np.asarray(state["com"], dtype=float).copy()
    com[:2] = sole_center_xy(env)
    return {
        "com": com,
        "torso_orientation": np.asarray(state["torso_rotation"], dtype=float).copy(),
    }


def control_step(
    env: IndustrialHumanoidEnv,
    mpc: WholeBodyMPC,
    cbf: CBFFilter,
    pd: PDController,
    reference: dict[str, np.ndarray],
) -> dict[str, np.ndarray | float | str]:
    """One control tick: MPC, PD, CBF, then torque into MuJoCo.

    Returns the quantities the demo logs. ``intervention`` is
    ``||tau_safe - tau_nominal||``; zero means the filter left the command
    unchanged.
    """
    state = env.get_state()
    solution = mpc.solve(state, reference)
    q_des = solution["q_des"]
    qd_des = solution["qd_des"]

    # One CBF solve per control tick. The modification ``delta`` is held across
    # the physics substeps; the PD and the gravity term are recomputed on every
    # MuJoCo step so the joints can actually hold the target.
    tau_gravity = np.asarray(env.data.qfrc_bias[env._actuator_dof_adr], dtype=float)
    tau_nominal = pd.compute_torque(
        q_des,
        state["joint_qpos"],
        state["joint_qvel"],
        desired_velocities=qd_des,
        torque_feedforward=tau_gravity,
    )
    tau_safe = cbf.safe_torque(tau_nominal, state["com"], state["com_velocity"])
    delta = tau_safe - tau_nominal

    def torque_at_substep() -> np.ndarray:
        q = env.data.qpos[env._actuator_qpos_adr]
        qd = env.data.qvel[env._actuator_dof_adr]
        gravity = np.asarray(env.data.qfrc_bias[env._actuator_dof_adr], dtype=float)
        torque = pd.compute_torque(
            q_des, q, qd, desired_velocities=qd_des, torque_feedforward=gravity
        )
        return torque + delta

    env.step_feedback(torque_at_substep)

    after = env.get_state()
    return {
        "time": float(np.asarray(after["time"]).reshape(-1)[0]),
        "com": after["com"].copy(),
        "com_velocity": after["com_velocity"].copy(),
        "intervention": float(np.linalg.norm(tau_safe - tau_nominal)),
        "cbf_status": cbf.status,
        "tau_nominal": tau_nominal.copy(),
        "tau_safe": tau_safe.copy(),
        "q_des": np.asarray(q_des, dtype=float).copy(),
        "joint_qpos": after["joint_qpos"].copy(),
        "joint_qvel": after["joint_qvel"].copy(),
        "base_rotation": after["base_rotation"].copy(),
        "barriers": cbf.com_barrier_values(after["com"]),
    }


def apply_push(env: IndustrialHumanoidEnv, force: np.ndarray) -> str:
    """Apply ``force`` (N, world frame) to the torso for ``PUSH_DURATION``.

    Tries the usual H1 body names and falls back to the floating-base root
    when the scene uses a different one.
    """
    for candidate in PUSH_BODY_CANDIDATES:
        try:
            env.apply_external_force(force, body_name=candidate, duration=PUSH_DURATION)
        except ValueError:
            continue
        else:
            return candidate
    env.apply_external_force(force, body_name=None, duration=PUSH_DURATION)
    return "root"


def save_log(path: Path, rows: list[dict]) -> None:
    """Write COM position, COM velocity and safety interventions to ``path``."""
    path.parent.mkdir(parents=True, exist_ok=True)
    time_s = np.array([row["time"] for row in rows])
    com = np.vstack([row["com"] for row in rows])
    com_velocity = np.vstack([row["com_velocity"] for row in rows])
    intervention = np.array([row["intervention"] for row in rows])
    status = np.array([row["cbf_status"] for row in rows])
    np.savez(
        path,
        time=time_s,
        com=com,
        com_velocity=com_velocity,
        intervention=intervention,
        cbf_status=status,
    )
    csv_path = path.with_suffix(".csv")
    header = "time,com_x,com_y,com_z,com_vx,com_vy,com_vz,intervention,cbf_status"
    lines = [header]
    for i in range(time_s.size):
        lines.append(
            f"{time_s[i]:.4f},{com[i, 0]:.6f},{com[i, 1]:.6f},{com[i, 2]:.6f},"
            f"{com_velocity[i, 0]:.6f},{com_velocity[i, 1]:.6f},{com_velocity[i, 2]:.6f},"
            f"{intervention[i]:.6f},{status[i]}"
        )
    csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_demo(args: argparse.Namespace) -> Path:
    """Open the MuJoCo viewer and run until the user closes it."""
    import mujoco.viewer

    env, mpc, cbf, pd = build_stack(args)
    env.reset(seed=args.seed)
    reference = stand_reference(env, env.get_state())
    log_path = Path(args.log_path)
    rows: list[dict] = []
    pushed = False
    next_report = 1.0

    LOG.info(
        "viewer open — close the window to stop. Push of %.0f N at t = %.1f s.",
        float(np.linalg.norm(args.push_force)),
        PUSH_TIME,
    )

    # launch_passive returns immediately. The loop below is what keeps the
    # window alive; is_running() goes false when the user closes it.
    viewer = mujoco.viewer.launch_passive(env.model, env.data)
    try:
        while viewer.is_running():
            tick = time.perf_counter()
            sim_time = float(env.data.time)

            if not pushed and sim_time >= PUSH_TIME:
                body = apply_push(env, np.asarray(args.push_force, dtype=float))
                pushed = True
                LOG.info(
                    "t = %.2f s: applied %s N for %.2f s on %s",
                    sim_time,
                    np.array2string(np.asarray(args.push_force), precision=0),
                    PUSH_DURATION,
                    body,
                )

            row = control_step(env, mpc, cbf, pd, reference)
            rows.append(row)
            viewer.sync()

            if row["time"] >= next_report:
                com = row["com"]
                vel = row["com_velocity"]
                LOG.info(
                    "t = %5.2f s | COM [%.3f %.3f %.3f] m | |v| = %.3f m/s | "
                    "intervention %.2f Nm (%s)",
                    row["time"],
                    com[0],
                    com[1],
                    com[2],
                    float(np.linalg.norm(vel)),
                    row["intervention"],
                    row["cbf_status"],
                )
                next_report += 1.0

            # Pace the loop to the control period so the viewer is watchable.
            remaining = args.control_dt - (time.perf_counter() - tick)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        viewer.close()
        if rows:
            save_log(log_path, rows)
            interventions = np.array([row["intervention"] for row in rows])
            LOG.info(
                "closed after %.2f s, %d steps. peak intervention %.2f Nm. log: %s",
                rows[-1]["time"],
                len(rows),
                float(interventions.max()),
                log_path,
            )
        env.close()
    return log_path


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="MPC + CBF demo for the Unitree H1")
    parser.add_argument("--horizon", type=int, default=20, help="MPC horizon length")
    parser.add_argument("--control-dt", type=float, default=0.02, help="control period in seconds")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--push-force",
        type=float,
        nargs=3,
        default=PUSH_FORCE.tolist(),
        metavar=("FX", "FY", "FZ"),
        help="world-frame force (N) applied at t = 2 s",
    )
    parser.add_argument(
        "--model-path",
        type=Path,
        default=root / "models" / "unitree_h1" / "scene.xml",
        help="MJCF scene for the H1",
    )
    parser.add_argument(
        "--log-path",
        type=Path,
        default=root / "data" / "demo_log.npz",
        help="where COM and intervention traces are written",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_demo(parse_args())
