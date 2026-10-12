"""Compare one deterministic step with and without the touchdown residual.

    python -m rl.touchdown_eval --model data/runs/touchdown_policy.pt --trials 3
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from rl.touchdown import TouchdownAgent, TouchdownConfig, summarize_trial  # noqa: E402
from rl.touchdown_train import _build  # noqa: E402
from main_demo import walk_tick  # noqa: E402


def _run(seed: int, agent: TouchdownAgent | None, config: TouchdownConfig) -> dict[str, float]:
    env, mpc, cbf, pd, gait, ik, torso = _build(seed)
    rows: list[dict] = []
    for _ in range(1600):
        row = walk_tick(env, mpc, cbf, pd, gait, ik, torso, rows, touchdown_agent=agent)
        rows.append(row)
        if float(row["com"][2]) < config.fall_height:
            break
        if _past_double(rows, 2.0):
            break
    summary = summarize_trial(rows, config)
    summary["seed"] = float(seed)
    return summary


def _past_double(rows: list[dict], seconds: float) -> bool:
    times = [float(row["time"]) for row in rows if str(row.get("walk_state")) == "DOUBLE_SUPPORT"]
    if not times:
        return False
    return float(rows[-1]["time"]) - times[0] >= seconds


def evaluate(model: Path, trials: int, seed: int, out: Path) -> dict:
    config = TouchdownConfig(seed=seed)
    agent = TouchdownAgent(config, deterministic=True)
    agent.load(model)
    baseline = [_run(seed + index, None, config) for index in range(trials)]
    residual = [_run(seed + index, agent, config) for index in range(trials)]
    report = {"baseline": baseline, "rl": residual, "model": str(model)}
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    _plot(baseline, residual, out.with_suffix(".png"))
    print(json.dumps({
        "baseline_touchdown": float(np.mean([row["touchdown"] for row in baseline])),
        "rl_touchdown": float(np.mean([row["touchdown"] for row in residual])),
        "baseline_stable": float(np.mean([row["stable"] for row in baseline])),
        "rl_stable": float(np.mean([row["stable"] for row in residual])),
        "baseline_fall": float(np.mean([row["fell"] for row in baseline])),
        "rl_fall": float(np.mean([row["fell"] for row in residual])),
    }, indent=2))
    return report


def _plot(baseline: list[dict], residual: list[dict], path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = ["touchdown", "stable", "fell"]
    base = [float(np.mean([row[key] for row in baseline])) for key in labels]
    learned = [float(np.mean([row[key] for row in residual])) for key in labels]
    axis = np.arange(len(labels))
    plt.figure(figsize=(6, 3.5))
    plt.bar(axis - 0.15, base, width=0.3, label="deterministic")
    plt.bar(axis + 0.15, learned, width=0.3, label="touchdown residual")
    plt.xticks(axis, labels)
    plt.ylim(0.0, 1.05)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=120)
    plt.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate the touchdown residual against the deterministic step")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, default=Path("data/runs/touchdown_eval.json"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    evaluate(args.model, args.trials, args.seed, args.out)


if __name__ == "__main__":
    main()
