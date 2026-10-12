"""Pack one demo rollout into an NPZ the report figures can reload.

The logger only stores values the controller has already computed. It does
not integrate the plant, and it does not change a torque, a reference or a
gain. Missing fields are omitted rather than filled with invented numbers.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

LOG_ROOT = Path(__file__).resolve().parents[1] / "data" / "logs"

# Nested dicts that are not numeric traces. Barriers are flattened separately.
_SKIP_DICT_KEYS = {"rl_reward_components"}


def write_experiment_log(
    path: Path,
    rows: list[dict[str, Any]],
    metadata: dict[str, Any] | None = None,
) -> Path:
    """Write ``rows`` and run metadata to ``path`` (an ``.npz`` file)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = pack_rows(rows)
    meta = dict(metadata or {})
    payload["metadata_json"] = np.asarray(json.dumps(_json_safe(meta)))
    for key in ("joint_names",):
        if key in meta and meta[key] is not None:
            payload[key] = np.asarray(list(meta[key]))
    for key in ("joint_low", "joint_high", "residual_bound"):
        if key in meta and meta[key] is not None:
            payload[key] = np.asarray(meta[key], dtype=float)
    np.savez(path, **payload)
    return path


def load_experiment_log(path: Path) -> dict[str, Any]:
    """Reload a log written by :func:`write_experiment_log`."""
    path = Path(path)
    with np.load(path, allow_pickle=True) as archive:
        data = {key: archive[key] for key in archive.files}
    raw = data.get("metadata_json")
    if raw is not None:
        text = raw.item() if np.asarray(raw).shape == () else str(raw)
        if isinstance(text, bytes):
            text = text.decode()
        data["metadata"] = json.loads(text)
    else:
        data["metadata"] = {}
    return data


def pack_rows(rows: list[dict[str, Any]]) -> dict[str, np.ndarray]:
    """Stack the fields that actually appear on ``rows``."""
    if not rows:
        return {}
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    payload: dict[str, np.ndarray] = {}
    barrier_names: list[str] = []
    for row in rows:
        barriers = row.get("barriers")
        if isinstance(barriers, dict):
            for name in barriers:
                if name not in barrier_names:
                    barrier_names.append(str(name))
    for name in barrier_names:
        payload[f"barrier_{name}"] = np.array(
            [
                float(row["barriers"][name])
                if isinstance(row.get("barriers"), dict) and name in row["barriers"]
                else np.nan
                for row in rows
            ],
            dtype=float,
        )
    for key in keys:
        if key in _SKIP_DICT_KEYS or key == "barriers":
            continue
        stacked = _stack_field([row.get(key) for row in rows])
        if stacked is not None:
            payload[key] = stacked
    return payload


def _stack_field(values: list[Any]) -> np.ndarray | None:
    """Stack one column. Return None when the column has no usable samples."""
    present = [value for value in values if value is not None]
    if not present:
        return None
    sample = present[0]
    if isinstance(sample, dict):
        return None
    if _is_text(sample):
        text = [_as_text(value) for value in values]
        return np.asarray(text)
    if _is_bool(sample):
        return np.array([float(bool(value)) if value is not None else np.nan for value in values])
    if _is_number(sample):
        return np.array(
            [float(value) if value is not None and _is_number(value) else np.nan for value in values],
            dtype=float,
        )
    arrays = []
    shape = np.asarray(sample).shape
    for value in values:
        if value is None:
            arrays.append(np.full(shape, np.nan))
            continue
        array = np.asarray(value, dtype=float)
        if array.shape != shape:
            return None
        arrays.append(array)
    return np.stack(arrays, axis=0)


def _is_text(value: Any) -> bool:
    return isinstance(value, (str, bytes, np.str_))


def _is_bool(value: Any) -> bool:
    return isinstance(value, (bool, np.bool_))


def _is_number(value: Any) -> bool:
    if isinstance(value, (bool, np.bool_)):
        return False
    return isinstance(value, (int, float, np.integer, np.floating))


def _as_text(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode()
    return str(value)


def _json_safe(value: Any) -> Any:
    """Replace non-finite numbers so the metadata string is valid JSON."""
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        number = float(value)
        return number if np.isfinite(number) else None
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    return value
