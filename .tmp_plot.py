"""Closed-loop CoM / orientation tracking under the simplified MPC.

Three scenarios, chosen to separate what the controller does from what the
missing contact-force constraints prevent:

  * pinned base   -- the pelvis is held by a test rig, so the legs act as a
                     fixed-base mechanism. This isolates the MPC + IK stack and
                     is the scenario in which CoM tracking is meaningful.
  * free, gravity -- the honest picture: the robot topples in well under a
                     second because the simplified model carries no ZMP or
                     contact-force constraints.
  * zero gravity  -- a control on the control: with no contact forces, angular
                     momentum conservation means joint motion cannot move the
                     CoM at all, whatever the controller asks for.
"""
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PROJECT = Path(__file__).resolve().parent / "project"
sys.path.insert(0, str(PROJECT))

import mujoco  # noqa: E402

from controllers.mpc_controller import MPCConfig, WholeBodyMPC, _rotation_log  # noqa: E402
from env.industrial_humanoid_env import EnvConfig, IndustrialHumanoidEnv  # noqa: E402

SCENE = PROJECT / "models" / "unitree_h1" / "scene.xml"
STEPS = 150
DT = 0.02
TIME = np.arange(STEPS) * DT
SQUAT = 0.05  # metres


def run(pinned=False, gravity=True):
    env = IndustrialHumanoidEnv(
        EnvConfig(model_path=SCENE, min_base_height=-100.0, min_upright=-1.1,
                  episode_seconds=STEPS * DT, reset_joint_noise=0.0)
    )
    if not gravity:
        env.model.opt.gravity[:] = 0.0
    # A pinned base is a fixed-base mechanism, so the CoM Jacobian must not be
    # made contact-relative.
    mpc = WholeBodyMPC(env.model, MPCConfig(horizon=20, dt=DT, contact_relative=not pinned))
    env.reset(seed=0)

    base_qpos = env.data.qpos[:7].copy()
    state = env.get_state()
    com_start = state["com"].copy()
    attitude_ref = state["torso_rotation"].copy()

    def com_reference(t):
        return com_start + np.array([0.0, 0.0, -SQUAT * float(np.clip(t, 0.0, 1.0))])

    log = {"com": [], "ref": [], "theta": [], "solver": [], "upright": []}
    for k in range(STEPS):
        t = k * DT
        state = env.get_state()
        preview = np.array([com_reference(t + j * DT) for j in range(mpc.config.horizon)])
        solution = mpc.solve(state, {"com": preview, "torso_orientation": attitude_ref})
        env.step(solution["q_des"])

        if pinned:
            # Test rig: hold the floating base, so only joint motion moves the CoM.
            env.data.qpos[:7] = base_qpos
            env.data.qvel[:6] = 0.0
            mujoco.mj_forward(env.model, env.data)

        after = env.get_state()
        log["com"].append(after["com"].copy())
        log["ref"].append(com_reference(t))
        log["theta"].append(_rotation_log(after["torso_rotation"] @ attitude_ref.T))
        log["solver"].append(solution["solve_info"]["solver"])
        log["upright"].append(float(after["torso_rotation"][2, 2]))
    env.close()
    return {k: (np.array(v) if k != "solver" else v) for k, v in log.items()}


scenarios = [
    ("pinned base", dict(pinned=True, gravity=True), "tab:blue"),
    ("free base, gravity", dict(pinned=False, gravity=True), "tab:red"),
    ("free base, zero-g", dict(pinned=False, gravity=False), "tab:green"),
]

results = {}
for label, kwargs, _ in scenarios:
    r = results[label] = run(**kwargs)
    err = np.linalg.norm(r["com"] - r["ref"], axis=1)
    tilted = r["upright"] < np.cos(np.deg2rad(25))
    r["fall"] = int(np.argmax(tilted)) if np.any(tilted) else STEPS
    # How much of the commanded 5 cm squat was actually delivered?
    delivered = (r["com"][0, 2] - r["com"][-1, 2]) / SQUAT
    print(f"\n{label}:")
    print(f"  upright for {r['fall'] * DT:.2f} s of {STEPS * DT:.2f} s")
    print(f"  CoM z moved {1000 * (r['com'][0, 2] - r['com'][-1, 2]):7.1f} mm of the "
          f"{1000 * SQUAT:.0f} mm commanded ({100 * delivered:.1f}%)")
    print(f"  CoM error: start {1000 * err[0]:.1f} mm -> steady state "
          f"{1000 * err[-25:].mean():.1f} mm")
    print(f"  peak torso attitude error: {np.max(np.linalg.norm(r['theta'], axis=1)):.4f} rad")
    print(f"  scipy needed on {sum(1 for s in r['solver'] if s == 'lsq_linear')}/{STEPS} ticks")

fig, axes = plt.subplots(3, 1, figsize=(9.5, 9), sharex=True)
fig.suptitle("Simplified MPC on the Unitree H1: 5 cm CoM squat + hold torso attitude",
             fontsize=13)

axes[0].plot(TIME, results["pinned base"]["ref"][:, 2], "k--", lw=2, label="CoM z reference")
for label, _, colour in scenarios:
    axes[0].plot(TIME, results[label]["com"][:, 2], lw=2, color=colour, label=label)
axes[0].set_ylabel("CoM height [m]")
axes[0].legend(loc="lower left", fontsize=9)
axes[0].grid(alpha=0.3)
axes[0].set_ylim(0.4, 1.02)

for label, _, colour in scenarios:
    err = 1000 * np.linalg.norm(results[label]["com"] - results[label]["ref"], axis=1)
    axes[1].plot(TIME, err, lw=2, color=colour, label=label)
axes[1].set_ylabel("CoM tracking error [mm]")
axes[1].set_yscale("log")
axes[1].legend(loc="upper left", fontsize=9)
axes[1].grid(alpha=0.3, which="both")

for label, _, colour in scenarios:
    axes[2].plot(TIME, np.linalg.norm(results[label]["theta"], axis=1), lw=2, color=colour,
                 label=label)
axes[2].set_ylabel(r"torso attitude error $\|\log(R R_{ref}^T)\|$  [rad]")
axes[2].set_xlabel("time [s]")
axes[2].set_yscale("log")
axes[2].legend(loc="upper left", fontsize=9)
axes[2].grid(alpha=0.3, which="both")

fall = results["free base, gravity"]["fall"]
for ax in axes:
    ax.axvline(fall * DT, color="tab:red", ls=":", lw=1.5)
axes[0].text(fall * DT + 0.03, 0.46, "free base has toppled\n(no ZMP / contact constraints)",
             fontsize=8.5, color="tab:red")

fig.tight_layout()
out = PROJECT / "data" / "mpc_com_tracking.png"
fig.savefig(out, dpi=130)
print(f"\nwrote {out}")
