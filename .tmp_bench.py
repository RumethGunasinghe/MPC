"""Benchmark lsq_linear methods on the MPC's bounded horizon problem."""
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear

PROJECT = Path(__file__).resolve().parent / "project"
sys.path.insert(0, str(PROJECT))

from controllers.mpc_controller import MPCConfig, WholeBodyMPC  # noqa: E402

mpc = WholeBodyMPC(None, MPCConfig(horizon=20, dt=0.02))
M, lo, hi = mpc._lsq_matrix, mpc._bounds_low, mpc._bounds_high
print(f"problem size: M = {M.shape}, {lo.size} bounded variables")

# Representative right-hand side: a big CoM error, as when the robot has toppled.
x0 = np.zeros(12)
x0[0] = 0.9
rhs = np.concatenate((mpc._sqrt_w_state * (np.zeros(20 * 12) - mpc._Phi @ x0),
                      np.zeros(mpc._n_control_rows),
                      np.zeros(mpc._n_control_rows)))

reference = None
for label, kwargs in [
    ("bvls", dict(method="bvls")),
    ("trf (exact)", dict(method="trf", lsq_solver="exact")),
    ("trf (lsmr)", dict(method="trf", lsq_solver="lsmr")),
    ("trf tol=1e-6", dict(method="trf", lsq_solver="exact", tol=1e-6)),
]:
    t0 = time.perf_counter()
    for _ in range(10):
        res = lsq_linear(M, rhs, bounds=(lo, hi), **kwargs)
    ms = (time.perf_counter() - t0) * 100
    if reference is None:
        reference = res.x
    print(f"  {label:16s} {ms:8.2f} ms   cost={res.cost:.6e}  nit={res.nit:3d}  "
          f"max|x-bvls|={np.abs(res.x - reference).max():.3e}")

# Per-axis separability: the six task axes are fully decoupled, so the same
# optimum can be had from six N-variable problems instead of one 6N problem.
print("\nseparability check")
N = 20
rows_per_knot = 12
sub_idx = [np.arange(N) * 6 + a for a in range(6)]
t0 = time.perf_counter()
for _ in range(10):
    pieces = []
    for a in range(6):
        cols = sub_idx[a]
        rows = np.abs(M[:, cols]).sum(axis=1) > 0
        res_a = lsq_linear(M[np.ix_(rows, cols)], rhs[rows],
                           bounds=(lo[cols], hi[cols]), method="bvls")
        pieces.append(res_a.x)
ms = (time.perf_counter() - t0) * 100
combined = np.zeros(N * 6)
for a in range(6):
    combined[sub_idx[a]] = pieces[a]
print(f"  6 x per-axis bvls {ms:8.2f} ms   max|x - monolithic bvls|="
      f"{np.abs(combined - reference).max():.3e}")
