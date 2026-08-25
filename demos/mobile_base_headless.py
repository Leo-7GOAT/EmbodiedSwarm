from pathlib import Path

import mujoco
import numpy as np

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "mjcf"
    / "mobile_manipulator.xml"
)

# Python handles the Unicode path.
xml_text = MODEL_PATH.read_text(encoding="utf-8")

# MuJoCo parses XML content instead of opening the Windows path itself.
model = mujoco.MjModel.from_xml_string(xml_text)
data = mujoco.MjData(model)


print("=== Model Info ===")
print("nq:", model.nq)
print("nv:", model.nv)
print("nu:", model.nu)

print()
print("=== Initial qpos ===")
print(data.qpos)

print()
print("=== Joints ===")

for i in range(model.njnt):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        i,
    )
    print(i, name)

print()
print("=== Set Base Pose ===")

data.qpos[0] = 1.0
data.qpos[1] = 2.0
data.qpos[2] = np.pi / 2

mujoco.mj_forward(model, data)

print("Updated qpos:")
print(data.qpos)