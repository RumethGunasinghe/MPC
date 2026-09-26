"""Is poor CoM delivery a weight-tuning issue or a structural one?"""
import sys
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parent / "project"
sys.path.insert(0, str(PROJECT))

import mujoco  # noqa: E402

from controllers.mpc_controller import MPCConfig, WholeBodyMPC, _rotation_log  # noqa: E402
from env.industrial_humanoid_env import EnvConfig, IndustrialHumanoidEnv  # noqa: E402

SCENE = PROJECT / "models" / "unitree_h1" / "scene.xml"
STEPS, DT, SQUAT = 150, 0.02, 0.05


def run(w_ik_com, w_posture, pinned=True):
    env = IndustrialHumanoidEnv(
        EnvConfig(model_path=SCENE, min_base_height=-100.0, min_upright=-1.1,
                  episode_seconds=STEPS * DT, reset_joint_noise=0.0))
    mpc = WholeBodyMPC(env.model, MPCConfig(
        horizon=20, dt=DT, contact_relative=not pinned,
        w_ik_com=w_ik_com, w_posture=w_posture))
    env.reset(seed=0)
    base_qpos = env.data.qpos[:7].copy()
    com_start = env.get_state()["com"].copy()
    attitude_ref = env.get_state()["torso_rotation"].copy()

    def reference(t):
        return com_start + np.array([0.0, 0.0, -SQUAT * float(np.clip(t, 0.0, 1.0))])

    for k in range(STEPS):
        t = k * DT
        state = env.get_state()
        preview = np.array([reference(t + j * DT) for j in range(20)])
        sol = mpc.solve(state, {"com": preview, "torso_orientation": attitude_ref})
        env.step(sol["q_des"])
        if pinned:
            env.data.qpos[:7] = base_qpos
            env.data.qvel[:6] = 0.0
            mujoco.mj_forward(env.model, env.data)

    final = env.get_state()
    moved = com_start[2] - final["com"][2]
    err = np.linalg.norm(final["com"] - reference(STEPS * DT))
    theta = np.linalg.norm(_rotation_log(final["torso_rotation"] @ attitude_ref.T))
    env.close()
    return moved, err, theta


print("pinned base, 5 cm CoM squat commanded")
print(f"{'w_ik_com':>9} {'w_posture':>10} {'CoM moved':>11} {'delivered':>10} "
      f"{'CoM err':>9} {'attitude err':>13}")
for w_com, w_post in [
    (1.0, 1e-2),      # defaults
    (10.0, 1e-2),
    (100.0, 1e-2),
    (1000.0, 1e-2),
    (1.0, 1e-4),
    (100.0, 1e-4),
    (1000.0, 1e-5),
]:
    moved, err, theta = run(w_com, w_post)
    print(f"{w_com:9.1f} {w_post:10.0e} {1000 * moved:8.1f} mm {100 * moved / SQUAT:9.1f}% "
          f"{1000 * err:6.1f} mm {theta:13.5f}")
