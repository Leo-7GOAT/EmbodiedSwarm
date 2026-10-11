"""Download the licensed upstream Panda assets, pinned by commit SHA.

Run on a machine with GitHub access. The bundled model is sufficient offline.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import urllib.request
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parents[1] / "assets/franka_emika_panda"
    root.mkdir(parents=True, exist_ok=True)
    sha = json.load(urllib.request.urlopen(
        "https://api.github.com/repos/google-deepmind/mujoco_menagerie/commits/main", timeout=30))["sha"]
    base = f"https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{sha}/franka_emika_panda/"
    def fetch(name):
        dest = root/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        if not dest.exists():
            for attempt in range(3):
                try:
                    dest.write_bytes(urllib.request.urlopen(base+name, timeout=60).read())
                    break
                except Exception:
                    if attempt == 2:
                        raise
        return name
    for name in ("panda.xml", "LICENSE", "README.md"):
        fetch(name)
    tree = ET.parse(root/"panda.xml")
    files = sorted({"assets/"+el.attrib["file"] for el in tree.findall("./asset/mesh")})
    with ThreadPoolExecutor(max_workers=8) as pool:
        for name in pool.map(fetch, files):
            print(name, flush=True)
    (root/"UPSTREAM.json").write_text(json.dumps({"repository":
        "https://github.com/google-deepmind/mujoco_menagerie", "commit": sha,
        "model": "franka_emika_panda", "modified": False}, indent=2))


if __name__ == "__main__":
    main()
