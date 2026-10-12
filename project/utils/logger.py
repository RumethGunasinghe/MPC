"""Run logging for simulation rollouts and training.

Collects per-step scalars and arrays in memory, then writes a CSV/NPZ pair
under ``data/runs/<run_id>/`` so the plotting helpers can reload them.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

DEFAULT_RUN_ROOT = Path(__file__).resolve().parents[1] / "data" / "runs"


def get_console_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """Stdlib logger with a single consistently formatted stream handler."""
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("[%(asctime)s] %(levelname)-7s %(name)s | %(message)s"))
        logger.addHandler(handler)
    logger.setLevel(level)
    return logger


@dataclass
class RunLogger:
    """Buffered logger for a single simulation or training run."""

    run_name: str = "run"
    root: Path = DEFAULT_RUN_ROOT
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.run_dir = self.root / f"{stamp}_{self.run_name}"
        self._records: list[dict[str, float]] = []
        self._arrays: dict[str, list[np.ndarray]] = {}
        self.console = get_console_logger(self.run_name)

    # ------------------------------------------------------------------
    def log_scalars(self, step: int, **values: float) -> None:
        """Append one row of scalar metrics (reward, cost, solve time, ...)."""
        self._records.append({"step": step, **{k: float(v) for k, v in values.items()}})

    def log_array(self, key: str, value: np.ndarray) -> None:
        """Append a per-step vector such as ``qpos``, ``tau`` or barrier values."""
        self._arrays.setdefault(key, []).append(np.asarray(value, dtype=float).copy())

    def flush(self) -> Path:
        """Write metadata, scalar CSV and stacked arrays to ``run_dir``.

        Scalar rows may grow extra keys over the run; missing entries are
        written as empty CSV fields. Array traces are stacked on axis 0 when
        every sample has the same shape.
        """
        self.run_dir.mkdir(parents=True, exist_ok=True)
        metadata = dict(self.metadata)
        metadata["n_records"] = len(self._records)
        metadata["arrays"] = sorted(self._arrays)
        (self.run_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, default=str), encoding="utf-8"
        )

        columns: list[str] = []
        for record in self._records:
            for key in record:
                if key not in columns:
                    columns.append(key)
        if "step" in columns:
            columns.remove("step")
            columns.insert(0, "step")
        csv_lines = [",".join(columns)]
        for record in self._records:
            csv_lines.append(",".join(_csv_field(record.get(key)) for key in columns))
        (self.run_dir / "scalars.csv").write_text("\n".join(csv_lines) + "\n", encoding="utf-8")

        stacked = {key: _stack_trace(samples) for key, samples in self._arrays.items()}
        np.savez(self.run_dir / "arrays.npz", **stacked)
        return self.run_dir

    def __enter__(self) -> "RunLogger":
        self.run_dir.mkdir(parents=True, exist_ok=True)
        (self.run_dir / "metadata.json").write_text(json.dumps(self.metadata, indent=2, default=str))
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.flush()


def load_run(run_dir: Path) -> dict[str, Any]:
    """Reload a flushed run for offline analysis and plotting.

    Returns metadata, a structured scalar array (empty if nothing was logged)
    and a dict of the stacked traces from ``arrays.npz``.
    """
    run_dir = Path(run_dir)
    metadata_path = run_dir / "metadata.json"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"no metadata.json in {run_dir}")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))

    scalars_path = run_dir / "scalars.csv"
    if scalars_path.is_file() and scalars_path.stat().st_size > 0:
        scalars = np.genfromtxt(scalars_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    else:
        scalars = np.empty(0)

    arrays_path = run_dir / "arrays.npz"
    arrays = dict(np.load(arrays_path)) if arrays_path.is_file() else {}
    return {"metadata": metadata, "scalars": scalars, "arrays": arrays, "run_dir": run_dir}


def _csv_field(value: Any) -> str:
    """Render one CSV cell. Missing values stay empty."""
    if value is None:
        return ""
    return str(value)


def _stack_trace(samples: list[np.ndarray]) -> np.ndarray:
    """Stack a per-step trace, falling back to an object array if shapes differ."""
    if not samples:
        return np.zeros((0,))
    try:
        return np.stack(samples, axis=0)
    except ValueError:
        return np.asarray(samples, dtype=object)
