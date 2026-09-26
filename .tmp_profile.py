"""Locate the per-step cost in the closed loop."""
import sys
import time
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parent / "project"
sys.path.insert(0, str(PROJECT))

from controllers.mpc_controller import MPCConfig, WholeBodyMPC  # noqa: E402
from env.industrial_humanoid_env import EnvConfig, IndustrialHumanoidEnv  # noqa: E402

SCENE = PROJECT / "models" / "unitree_h1" / "scene.xml"

env = IndustrialHumanoidEnv(EnvConfig(model_path=SCENE, min_base_height=-100.0, min_upright=-1.1))
mpc = WholeBodyMPC(env.model, MPCConfig(horizon=20, dt=0.02))
env.reset(seed=0)
target = env.get_state()["com"] + np.array([0.0, 0.0, -0.05])

rows = []
for k in range(100):
    t0 = time.perf_counter()
    state = env.get_state()
    t1 = time.perf_counter()
    sol = mpc.solve(state, {"com": target})
    t2 = time.perf_counter()
    env.step(sol["q_des"])
    t3 = time.perf_counter()
    rows.append((k, (t1 - t0) * 1e3, (t2 - t1) * 1e3, (t3 - t2) * 1e3,
                 sol["solve_info"]["solver"], sol["solve_info"].get("iterations", 0),
                 sol["solve_info"]["ik_solver"],
                 float(np.linalg.norm(env.get_com_position() - target))))

print(f"{'step':>5} {'get_state':>10} {'solve':>9} {'env.step':>9} {'solver':>11} "
      f"{'iters':>6} {'ik':>11} {'com_err':>8}")
for r in rows[::10] + rows[-3:]:
    print(f"{r[0]:5d} {r[1]:10.2f} {r[2]:9.2f} {r[3]:9.2f} {r[4]:>11} {r[5]:6d} {r[6]:>11} {r[7]:8.4f}")

arr = np.array([(r[1], r[2], r[3]) for r in rows])
print(f"\ntotals over 100 steps: get_state={arr[:, 0].sum() / 1000:.2f}s  "
      f"solve={arr[:, 1].sum() / 1000:.2f}s  env.step={arr[:, 2].sum() / 1000:.2f}s")
print(f"means (ms): get_state={arr[:, 0].mean():.2f}  solve={arr[:, 1].mean():.2f}  "
      f"env.step={arr[:, 2].mean():.2f}")
print(f"max solve  = {arr[:, 1].max():.1f} ms at step {int(arr[:, 1].argmax())}")
n_scipy = sum(1 for r in rows if r[4] == "lsq_linear")
print(f"steps using scipy for the horizon: {n_scipy}/100")
env.close()
