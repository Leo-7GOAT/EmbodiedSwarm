"""Measure a real subprocess on Linux/WSL; never infer target-platform HIL.

CPU totals come from kernel wait accounting. Memory is sampled summed RSS across
the process tree and can double count shared pages; it is not a hard peak bound.
"""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import time


def profile_command(command, output, interval=.01):
    if os.name != "posix":
        raise RuntimeError("use this profiler in Linux/WSL")
    if not command or not 0 < interval <= 1:
        raise ValueError("command and sampling interval in (0, 1] required")
    import resource
    import psutil
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(output)
    log = output.with_suffix(".stdout.txt")
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    samples, max_rss, max_processes = 0, 0, 0
    start = time.perf_counter()
    with log.open("x", encoding="utf-8") as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        tree = psutil.Process(process.pid)
        while True:
            rss, count = 0, 0
            try:
                children = [tree]+tree.children(recursive=True)
                for child in children:
                    try:
                        rss += child.memory_info().rss
                        count += 1
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
            max_rss, max_processes = max(max_rss, rss), max(max_processes, count)
            samples += 1
            try:
                result = process.wait(timeout=interval)
                break
            except subprocess.TimeoutExpired:
                pass
    wall = time.perf_counter()-start
    after = resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu_user, cpu_system = after.ru_utime-before.ru_utime, after.ru_stime-before.ru_stime
    report = {"status": "PASS_PROCESS" if result == 0 else "FAIL", "command": command,
              "exit_code": result, "wall_s": wall, "cpu_user_s": cpu_user,
              "cpu_system_s": cpu_system, "cpu_percent_one_core": (cpu_user+cpu_system)/wall*100,
              "sampled_tree_rss_max_bytes": max_rss, "max_observed_processes": max_processes,
              "samples": samples, "requested_sample_interval_s": interval,
              "platform": platform.platform(), "processor": platform.processor(),
              "logical_cpus": os.cpu_count(), "stdout": str(log),
              "physical_validation": "PENDING_HARDWARE",
              "method": "kernel waited-child CPU time; sampled process-tree summed RSS (shared pages may repeat); no target HIL claim"}
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=.01)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    report = profile_command(command, args.output, args.interval)
    print(json.dumps(report, indent=2))
    raise SystemExit(report["exit_code"])


if __name__ == "__main__":
    main()
