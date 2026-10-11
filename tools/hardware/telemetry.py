"""Strict, streaming telemetry with explicit provenance and one clock domain.

The schema intentionally records domain values rather than invented serial/CAN
frames. Hardware recordings require a user-supplied, verified transport adapter.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterator

MODES = {"mock_io", "replay", "real_sim", "hardware"}
KINDS = {"command", "state", "encoder", "imu", "fault", "receipt"}


def _finite_tree(value: Any) -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(value):
            raise ValueError("telemetry contains a nonfinite number")
    elif isinstance(value, list):
        for child in value:
            _finite_tree(child)
    elif isinstance(value, dict):
        if not all(isinstance(k, str) for k in value):
            raise ValueError("telemetry keys must be strings")
        for child in value.values():
            _finite_tree(child)
    else:
        raise ValueError("telemetry values must be JSON values")


def validate(record: dict, sequence: int, last_time: float, provenance: tuple | None) -> tuple:
    if not isinstance(record, dict) or record.get("schema") != 1:
        raise ValueError("unsupported telemetry schema")
    if type(record.get("sequence")) is not int or record["sequence"] != sequence:
        raise ValueError("missing, duplicate or reordered sequence")
    stamp = record.get("time_s")
    if type(stamp) not in (float, int) or not math.isfinite(stamp) or stamp < 0 or stamp < last_time:
        raise ValueError("time must be finite, nonnegative and monotonic")
    if record.get("kind") not in KINDS or not isinstance(record.get("payload"), dict):
        raise ValueError("invalid event kind or payload")
    source = record.get("source")
    if not isinstance(source, dict) or source.get("mode") not in MODES:
        raise ValueError("explicit source mode required")
    if not all(isinstance(source.get(k), str) and source[k].strip() for k in ("backend", "clock")):
        raise ValueError("backend and clock domain required")
    current = (source["mode"], source["backend"], source["clock"])
    if provenance is not None and current != provenance:
        raise ValueError("mixed provenance or clock domains in a recording")
    if not isinstance(record.get("units"), dict) or not all(isinstance(v, str) and v for v in record["units"].values()):
        raise ValueError("explicit units dictionary required")
    _finite_tree(record)
    return current


class Recorder:
    """Create-only JSONL writer; existing logs are never overwritten."""

    def __init__(self, path: str | Path, mode: str, backend: str, clock: str):
        self.path = Path(path)
        self.source = {"mode": mode, "backend": backend, "clock": clock}
        self.sequence, self.last_time = 0, -1.0
        self.provenance = None
        # Validate metadata before creating an output file.
        validate(self._record(0.0, "state", {}, {}), 0, -1.0, None)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("x", encoding="utf-8", newline="\n")

    def _record(self, time_s, kind, payload, units):
        return {"schema": 1, "source": self.source, "sequence": self.sequence,
                "time_s": time_s, "kind": kind, "payload": payload, "units": units}

    def write(self, time_s: float, kind: str, payload: dict, units: dict) -> None:
        record = self._record(time_s, kind, payload, units)
        provenance = validate(record, self.sequence, self.last_time, self.provenance)
        self.stream.write(json.dumps(record, allow_nan=False, separators=(",", ":")) + "\n")
        self.stream.flush()
        self.provenance = provenance
        self.last_time = time_s
        self.sequence += 1

    def close(self):
        self.stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def replay(path: str | Path) -> Iterator[dict]:
    """Read and validate; this iterator never sends commands to a device."""
    last_time, provenance = -1.0, None
    with Path(path).open(encoding="utf-8") as stream:
        for sequence, line in enumerate(stream):
            if len(line) > 1024 * 1024:
                raise ValueError("telemetry record exceeds 1 MiB")
            record = json.loads(line)
            provenance = validate(record, sequence, last_time, provenance)
            last_time = record["time_s"]
            yield record


def timing_summary(records: list[dict]) -> dict:
    """Nearest-rank quantiles; receipt time must share the command clock."""
    states = [r["time_s"] for r in records if r["kind"] == "state"]
    periods = [b-a for a, b in zip(states, states[1:])]
    commands, roundtrip = {}, []
    for record in records:
        payload = record["payload"]
        if record["kind"] == "command" and "command_id" in payload:
            cid = payload["command_id"]
            if cid in commands:
                raise ValueError("duplicate command ID")
            commands[cid] = record["time_s"]
        elif record["kind"] == "receipt":
            cid = payload.get("command_id")
            if cid not in commands or record["time_s"] < commands[cid]:
                raise ValueError("receipt has no matching prior command")
            roundtrip.append(record["time_s"]-commands.pop(cid))

    def stats(values):
        if not values:
            return None
        values = sorted(values)
        return {"samples": len(values), "p50_s": values[math.ceil(.5*len(values))-1],
                "p95_s": values[math.ceil(.95*len(values))-1],
                "p99_s": values[math.ceil(.99*len(values))-1],
                "min_s": values[0], "max_s": values[-1]}

    mean = sum(periods)/len(periods) if periods else None
    return {"state_period": stats(periods), "roundtrip": stats(roundtrip),
            "unacknowledged_commands": len(commands),
            "period_jitter_rms_s": math.sqrt(sum((x-mean)**2 for x in periods)/len(periods)) if periods else None,
            "statistics": "nearest-rank quantiles, RMS period deviation from sample mean"}
