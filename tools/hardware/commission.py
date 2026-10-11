"""Offline step-response preparation, measurement and replay CLI.

No hardware transport is implemented or guessed here. Generated plans must pass
through the project's safety supervisor and a verified adapter before execution.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .telemetry import Recorder, replay, timing_summary


def step_plan(amplitude: float, limit: float, dt: float, duration: float, onset: float = 1.0) -> list[tuple[float, float]]:
    if not all(math.isfinite(v) for v in (amplitude, limit, dt, duration, onset)):
        raise ValueError("step plan arguments must be finite")
    if limit <= 0 or abs(amplitude) > limit:
        raise ValueError("step exceeds declared software limit")
    if dt <= 0 or duration <= onset or onset < 0 or duration/dt > 1_000_000:
        raise ValueError("invalid sampling, onset or duration")
    count = int(math.floor(duration/dt))+1
    # Final zero command is part of every plan. Completion is not an E-stop.
    return [(i*dt, amplitude if onset <= i*dt < duration-dt/2 else 0.0) for i in range(count)]


def step_response(records: list[dict], channel: str) -> dict:
    commands = [(r["time_s"], r["payload"][channel]) for r in records if r["kind"] == "command" and channel in r["payload"]]
    states = [(r["time_s"], r["payload"][channel]) for r in records if r["kind"] == "state" and channel in r["payload"]]
    if len(commands) < 2 or len(states) < 5:
        raise ValueError("insufficient command/state samples")
    units = {r["units"].get(channel) for r in records if channel in r["payload"]}
    if len(units) != 1 or None in units:
        raise ValueError("channel units must exist and agree")
    base = commands[0][1]
    onset_index = next((i for i, (_, value) in enumerate(commands) if abs(value-base) > 1e-12), None)
    if onset_index is None:
        raise ValueError("no command step in recording")
    onset, target = commands[onset_index]
    end = next((t for t, value in commands[onset_index+1:] if abs(value-target) > 1e-12), float("inf"))
    before = [v for t, v in states if t < onset]
    after = [(t, v) for t, v in states if onset <= t < end]
    if not before or len(after) < 5:
        raise ValueError("need baseline and response samples before next command change")
    baseline = sum(before)/len(before)
    tail = after[-max(3, len(after)//5):]
    steady = sum(v for _, v in tail)/len(tail)
    delta = steady-baseline
    if abs(delta) < 1e-9:
        raise ValueError("response not identifiable: zero measured excursion")
    def crossing(fraction):
        return next((t-onset for t, v in after if (v-baseline)/delta >= fraction), None)
    t10, t90, t63 = crossing(.1), crossing(.9), crossing(1-math.exp(-1))
    return {"channel": channel, "unit": next(iter(units)), "samples": len(after),
            "onset_s": onset, "baseline": baseline, "tail_mean": steady,
            "gain_estimate": delta/(target-base), "delay_to_10pct_s": t10,
            "rise_10_to_90_s": t90-t10 if t10 is not None and t90 is not None else None,
            "time_to_63pct_s": t63,
            "max_overshoot_fraction": max(0.0, max((v-baseline)/delta for _, v in after)-1),
            "method": "sample threshold crossings; tail mean is an estimate, not a fitted actuator model",
            "provenance": records[0]["source"],
            "physical_validation": "PENDING_HARDWARE" if records[0]["source"]["mode"] != "hardware" else "recording_only_pending_review"}


def mock_record(path: str | Path, amplitude: float = .1, limit: float = .2, dt: float = .01,
                duration: float = 4., tau: float = .15, delay: float = .05,
                channel: str = "joint_rad", unit: str = "rad"):
    if not math.isfinite(tau) or tau <= 0 or not math.isfinite(delay) or delay < 0:
        raise ValueError("invalid mock time constant or delay")
    plan = step_plan(amplitude, limit, dt, duration)
    response = 0.0
    with Recorder(path, "mock_io", "explicit_first_order_fixture", "fixture_simulation_seconds") as writer:
        for index, (time, command) in enumerate(plan):
            delayed = plan[max(0, index-math.ceil(delay/dt))][1] if index >= math.ceil(delay/dt) else 0.0
            response += (1-math.exp(-dt/tau))*(delayed-response)
            writer.write(time, "command", {"command_id": index, channel: command}, {channel: unit})
            writer.write(time, "state", {channel: response}, {channel: unit})


def inject_fault(records: list[dict], kind: str, start: float, duration: float, delay: float = .1) -> list[dict]:
    if kind not in {"drop_state", "stale_state", "command_timeout"}:
        raise ValueError("unsupported fault")
    if not all(math.isfinite(v) for v in (start, duration, delay)) or start < 0 or duration <= 0 or delay <= 0:
        raise ValueError("invalid fault interval")
    out = []
    for record in records:
        record = json.loads(json.dumps(record))
        time = record["time_s"]
        active = start <= time < start+duration
        if active and ((kind == "drop_state" and record["kind"] == "state") or
                       (kind == "command_timeout" and record["kind"] == "command")):
            continue
        if active and kind == "stale_state" and record["kind"] == "state":
            record["payload"]["measurement_time_s"] = max(0., time-delay)
            record["units"]["measurement_time_s"] = "s"
        record["source"] = {"mode": "replay", "backend": "fault_injection_offline", "clock": record["source"]["clock"]}
        record["sequence"] = len(out)
        out.append(record)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation", required=True)
    mock = sub.add_parser("mock-step", help="generate explicitly labelled fixture data")
    mock.add_argument("output", type=Path)
    mock.add_argument("--channel", default="joint_rad")
    mock.add_argument("--unit", default="rad")
    measure = sub.add_parser("measure")
    measure.add_argument("input", type=Path)
    measure.add_argument("--channel", default="joint_rad")
    view = sub.add_parser("replay", help="validate and summarize without applying commands")
    view.add_argument("input", type=Path)
    fault = sub.add_parser("inject-fault")
    fault.add_argument("input", type=Path)
    fault.add_argument("output", type=Path)
    fault.add_argument("--kind", choices=("drop_state", "stale_state", "command_timeout"), required=True)
    fault.add_argument("--start", type=float, default=1.)
    fault.add_argument("--duration", type=float, default=.2)
    args = parser.parse_args()
    if args.operation == "mock-step":
        mock_record(args.output, channel=args.channel, unit=args.unit)
        print(json.dumps({"output": str(args.output), "status": "PASS_MOCK", "physical_validation": "PENDING_HARDWARE"}))
    elif args.operation == "measure":
        print(json.dumps(step_response(list(replay(args.input)), args.channel), indent=2))
    elif args.operation == "replay":
        records = list(replay(args.input))
        if not records:
            raise ValueError("empty recording")
        print(json.dumps({"source": records[0]["source"], "events": len(records), "timing": timing_summary(records)}, indent=2))
    else:
        records = inject_fault(list(replay(args.input)), args.kind, args.start, args.duration)
        if not records:
            raise ValueError("fault removes all events")
        source = records[0]["source"]
        with Recorder(args.output, **{k: source[k] for k in ("mode", "backend", "clock")}) as writer:
            for r in records:
                writer.write(r["time_s"], r["kind"], r["payload"], r["units"])
        print(json.dumps({"output": str(args.output), "source": source, "events": len(records)}))


if __name__ == "__main__":
    main()
