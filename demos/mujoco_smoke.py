"""Finite-duration interactive viewer smoke for the W02 MJCF model."""

import argparse
import math
from pathlib import Path
import time

import mujoco
import mujoco.viewer


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "mjcf"
    / "mobile_manipulator.xml"
)


def _joint_address(model: mujoco.MjModel, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"viewer smoke joint not found: {name}")
    return int(model.jnt_qposadr[joint_id])


def run_visual_smoke(duration: float) -> None:
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be a positive finite number")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    base_x = _joint_address(model, "base_x")
    base_y = _joint_address(model, "base_y")
    base_yaw = _joint_address(model, "base_yaw")

    start = time.perf_counter()
    last_frame = start
    frame_count = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            now = time.perf_counter()
            elapsed = now - start
            if elapsed >= duration:
                break

            # Animate the planar base and two arm targets so frame and actuator
            # direction errors are visible during the smoke.
            data.qpos[base_x] = 0.35 * math.sin(0.8 * elapsed)
            data.qpos[base_y] = 0.20 * math.sin(0.4 * elapsed)
            data.qpos[base_yaw] = 0.35 * math.sin(0.6 * elapsed)
            data.ctrl[1] = 0.35 * math.sin(elapsed)
            data.ctrl[3] = -0.30 * math.sin(elapsed)

            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            viewer.sync()
            frame_count += 1

            frame_elapsed = time.perf_counter() - last_frame
            time.sleep(max(0.0, model.opt.timestep - frame_elapsed))
            last_frame = time.perf_counter()

    if frame_count == 0:
        raise RuntimeError("MuJoCo viewer closed before rendering a frame")

    print(f"Rendered frames: {frame_count}")
    print("MUJOCO VISUAL SMOKE PASS")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Open the W02 mobile-manipulator viewer and auto-close."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="viewer duration in seconds (default: 5)",
    )
    args = parser.parse_args()
    run_visual_smoke(args.duration)


if __name__ == "__main__":
    main()
