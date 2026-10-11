"""Stage tested, committed software releases and roll back local selection.

Selection writes a manifest only. No physical device or remote host is accessed.
"""
from __future__ import annotations

import argparse
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile


def _git(repo: Path, *args: str) -> bytes:
    return subprocess.check_output(["git", "-C", str(repo), *args], stderr=subprocess.STDOUT)


def _pointer(root: Path, data: dict):
    temporary = root/"selection.new.json"
    with temporary.open("x", encoding="utf-8") as output:
        json.dump(data, output, indent=2)
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary, root/"selection.json")


def stage(repo: Path, root: Path, verify: bool = True) -> dict:
    repo, root = repo.resolve(), root.resolve()
    if repo == root or repo in root.parents:
        raise ValueError("release directory must be outside source repository")
    if _git(repo, "status", "--porcelain").strip():
        raise ValueError("commit and verify changes before staging a release")
    sha = _git(repo, "rev-parse", "HEAD").decode().strip()
    destination = root/"releases"/sha
    destination.mkdir(parents=True, exist_ok=False)
    archive = _git(repo, "archive", "--format=tar", "HEAD")
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:") as bundle:
        for entry in bundle.getmembers():
            target = (destination/entry.name).resolve()
            if destination not in target.parents or entry.issym() or entry.islnk():
                raise ValueError("unsafe archive path or symlink")
        bundle.extractall(destination)
    metadata = {"commit": sha, "source": str(repo), "release": str(destination),
                "hardware_enabled": False, "status": "STAGED_UNVERIFIED",
                "physical_validation": "PENDING_HARDWARE"}
    metadata_path = destination/"release.json"
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    if verify:
        venv_path = destination/".venv"
        subprocess.run([sys.executable, "-m", "venv", "--system-site-packages", str(venv_path)], check=True)
        executable = venv_path/("Scripts/python.exe" if os.name == "nt" else "bin/python")
        subprocess.run([str(executable), "-m", "pip", "install", "-e", str(destination)+"[dev,profiling]"], check=True)
        subprocess.run([str(executable), "-m", "pip", "check"], check=True)
        with (destination/"deployment_tests.txt").open("w", encoding="utf-8") as log:
            result = subprocess.run([str(executable), "-m", "pytest", "-q"], cwd=destination,
                                    stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError(f"release tests failed; selection unchanged; inspect {destination}")
        metadata["status"] = "STAGED_TESTED"
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return metadata


def select(root: Path, commit: str, rollback: bool = False) -> dict:
    root = root.resolve()
    if len(commit) != 40 or any(x not in "0123456789abcdef" for x in commit):
        raise ValueError("selection requires a full Git commit SHA")
    release = root/"releases"/commit
    data = json.loads((release/"release.json").read_text(encoding="utf-8"))
    if data["commit"] != commit or data["status"] != "STAGED_TESTED" or data["hardware_enabled"] is not False:
        raise ValueError("only tested, hardware-disabled software releases can be selected")
    previous = None
    pointer = root/"selection.json"
    if pointer.exists():
        previous = json.loads(pointer.read_text(encoding="utf-8")).get("commit")
    selection = {"commit": commit, "previous_commit": previous,
                 "operation": "rollback" if rollback else "select",
                 "hardware_enabled": False, "process_started": False,
                 "physical_validation": "PENDING_HARDWARE"}
    _pointer(root, selection)
    return selection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="local software release directory outside the source repo")
    commands = parser.add_subparsers(dest="operation", required=True)
    create = commands.add_parser("stage")
    create.add_argument("--repo", type=Path, default=Path.cwd())
    create.add_argument("--skip-tests", action="store_true", help="unverified releases cannot be selected")
    activate = commands.add_parser("select")
    activate.add_argument("commit")
    commands.add_parser("rollback")
    args = parser.parse_args()
    if args.operation == "stage":
        result = stage(args.repo, args.root, not args.skip_tests)
    elif args.operation == "select":
        result = select(args.root, args.commit)
    else:
        current = json.loads((args.root/"selection.json").read_text(encoding="utf-8"))
        if not current.get("previous_commit"):
            raise ValueError("no previous software selection")
        result = select(args.root, current["previous_commit"], rollback=True)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
