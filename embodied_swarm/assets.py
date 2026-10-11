"""Resolve repository assets or wheel-installed model data."""
from pathlib import Path
import sys


def asset_path(relative):
    for base in (Path(__file__).parents[1]/"assets", Path(sys.prefix)/"share/embodied_swarm/assets"):
        path = base/relative
        if path.exists():
            return path
    raise FileNotFoundError(f"EmbodiedSwarm asset not installed: {relative}")
