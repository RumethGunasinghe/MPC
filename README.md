# Whole-Body Model Predictive Control for Industrial Humanoids

Research prototype for whole-body MPC on the Unitree H1 humanoid performing
industrial manipulation and locomotion tasks, with a control-barrier-function
safety filter and a safe reinforcement learning layer.

## Stack

| Component | Role |
| --- | --- |
| MuJoCo | Rigid-body simulation of the Unitree H1 and the industrial workcell |
| Unitree H1 | 19-DoF target platform (MJCF model lives in `models/unitree_h1/`) |
| CVXPY | Convex QP backend for the MPC and the CBF safety filter |
| Safe RL | Residual policy learning under hard safety constraints |

## Architecture

```
observation -> [ Safe RL policy ] -> reference / residual
                      |
                      v
              [ MPC controller ]  (CVXPY QP over a receding horizon)
                      |
                      v
              [ CBF safety filter ] (minimally invasive QP projection)
                      |
                      v
              [ PD controller ] -> joint torques -> MuJoCo
```

The MPC plans whole-body motion over a horizon, the CBF filter projects the
commanded action onto the safe set, and the PD layer tracks the resulting
joint targets at the simulation rate.

## Layout

```
project/
├── main_demo.py                      # End-to-end demo entry point
├── requirements.txt
├── README.md
├── env/
│   └── industrial_humanoid_env.py    # MuJoCo + Gymnasium environment
├── controllers/
│   ├── mpc_controller.py             # Whole-body MPC (CVXPY)
│   └── pd_controller.py              # Low-level joint PD tracking
├── safety/
│   └── cbf_filter.py                 # Control barrier function QP filter
├── rl/
│   └── safe_rl_training.py           # Safe RL training loop
├── utils/
│   ├── logger.py                     # Run logging / episode traces
│   └── plotting.py                   # Diagnostic plots
├── data/                             # Logs, traces, checkpoints (gitignored)
└── models/
    └── unitree_h1/                   # MJCF / meshes for the H1
```

## Setup

```bash
python -m venv .venv
.venv\Scripts\activate        # Windows
pip install -r requirements.txt
```

Place the Unitree H1 MJCF description in `models/unitree_h1/` (see the README
in that folder).

## Running

```bash
python main_demo.py --steps 2000 --render
```

## Status

Scaffold only. Every module exposes its intended interface with `TODO`
markers where the implementation goes.
