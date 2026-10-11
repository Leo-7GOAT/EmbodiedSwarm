#!/usr/bin/env python3
"""Explicit ROS JointState <-> real MuJoCo Panda position-actuator bridge.

Physics qpos is initialized once from the model home keyframe. Every subsequent
state comes from mj_step; ROS commands only write actuator ctrl, never qpos.
Gripper/pick-place physics is accepted separately by the project demo.
"""
import argparse
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
import xacro
import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


def arm_initial_positions(config_path=None):
    """Read the same ros2_control initial pose used by the hardware plugin."""
    config_path = config_path or Path(get_package_share_directory('embodied_swarm_hardware')) / 'config/panda.urdf.xacro'
    robot = ET.fromstring(xacro.process_file(str(config_path), mappings={'backend': 'mujoco'}).toxml())
    joints = robot.find("ros2_control[@name='EmbodiedArm']").findall('joint')
    initial = {joint.attrib['name']: float(joint.find("param[@name='initial']").text) for joint in joints}
    return np.array([initial[f'panda_joint{i}'] for i in range(1, 8)])


def initialize_arm(model, data, jids, aids, initial):
    """Set the documented initial condition once, before the first physics step.

    Project scenes may have no keyframe. Never let their zero default pose or
    a different model keyframe become a controller's measured startup hold.
    All later state changes remain exclusively actuator-driven mj_step.
    """
    initial = np.asarray(initial, dtype=float)
    bounds = model.jnt_range[jids]
    if initial.shape != (7,) or not np.isfinite(initial).all() or np.any(initial < bounds[:, 0]) or np.any(initial > bounds[:, 1]):
        raise ValueError('Invalid ros2_control initial arm configuration')
    data.qpos[model.jnt_qposadr[jids]] = initial
    data.qvel[model.jnt_dofadr[jids]] = 0
    data.ctrl[aids] = initial
    mujoco.mj_forward(model, data)


class Bridge(Node):
    def __init__(self, path):
        super().__init__('mujoco_panda_bridge')
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.names = [f'panda_joint{i}' for i in range(1, 8)]
        self.jids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f'joint{i}') for i in range(1, 8)]
        if min(self.jids) < 0:
            self.jids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.names]
        if min(self.jids) < 0:
            raise ValueError('Model lacks Panda seven joint contract')
        self.qa = self.model.jnt_qposadr[self.jids]
        self.va = self.model.jnt_dofadr[self.jids]
        self.aids = []
        for jid in self.jids:
            candidates = np.flatnonzero((self.model.actuator_trnid[:, 0] == jid) &
                                       (self.model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT))
            if len(candidates) != 1:
                raise ValueError('Exactly one position actuator per arm joint required')
            self.aids.append(int(candidates[0]))
        initialize_arm(self.model, self.data, self.jids, self.aids, arm_initial_positions())
        self.target = self.data.qpos[self.qa].copy()
        self.last = time.monotonic()
        self.paused = False
        self.pub = self.create_publisher(JointState, '/mujoco/joint_states', 10)
        self.create_subscription(JointState, '/mujoco/joint_commands', self.command, 10)
        self.create_service(SetBool, '/mujoco/pause_state', self.pause)
        self.create_timer(0.01, self.tick)

    def pause(self, request, response):
        self.paused = request.data
        response.success = True
        return response

    def command(self, msg):
        age = (self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        if not -0.05 <= age <= 0.25 or len(msg.position) != len(msg.name):
            return
        try:
            target = np.array([msg.position[msg.name.index(n)] for n in self.names])
        except ValueError:
            return
        bounds = self.model.jnt_range[self.jids]
        if np.all(np.isfinite(target)) and np.all(target >= bounds[:, 0]) and np.all(target <= bounds[:, 1]):
            self.target = target
            self.last = time.monotonic()

    def tick(self):
        if time.monotonic() - self.last > 0.25:
            self.target = self.data.qpos[self.qa].copy()
        self.data.ctrl[self.aids] = self.target
        for _ in range(max(1, round(0.01 / self.model.opt.timestep))):
            # Arm feed-forward gravity/Coriolis compensation; object remains free physics.
            self.data.qfrc_applied[self.va] = self.data.qfrc_bias[self.va]
            mujoco.mj_step(self.model, self.data)
        if not self.paused:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.names
            msg.position = self.data.qpos[self.qa].tolist()
            msg.velocity = self.data.qvel[self.va].tolist()
            msg.effort = self.data.qfrc_actuator[self.va].tolist()
            self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Bridge(args.model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
