#!/usr/bin/env python3
"""Owns only processes it starts; no pkill, existing workspaces unchanged."""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['mock', 'mujoco'], default='mock')
    parser.add_argument('--evidence-dir', type=Path, help='Optional separate directory for a regression repeat')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    output = args.evidence_dir or root / 'evidence' / args.backend
    output.mkdir(parents=True, exist_ok=True)
    # A failed CI job must not publish an old passing JSON checked out with the
    # repository. Logs/bags remain available, but result files belong to this run.
    for filename in ('acceptance.json', 'replay.json', 'failure.json'):
        (output / filename).unlink(missing_ok=True)
    run_status = {'status': 'RUNNING', 'backend': args.backend, 'started_unix_ns': time.time_ns()}
    def status(value, **details):
        run_status.update(status=value, **details)
        (output / 'run_status.json').write_text(json.dumps(run_status, indent=2))
    status('RUNNING')
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '73'
    env['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.update({'ROS_DOMAIN_ID': env['ROS_DOMAIN_ID'], 'ROS_LOCALHOST_ONLY': '1'})
    processes = []
    streams = []

    def start(command, logfile):
        stream = open(output / logfile, 'w')
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)

    try:
        if args.backend == 'mujoco':
            start(['ros2', 'run', 'embodied_swarm_hardware', 'mujoco_bridge.py', '--model', str(root.parent / 'assets/mjcf/panda_pick_place.xml')], 'bridge.log')
        launch = start(['ros2', 'launch', 'embodied_swarm_hardware', 'bringup.launch.py', f'backend:={args.backend}'], 'launch.log')
        # Native acceptance waits on complete fresh state, active controllers,
        # TF and MoveIt's state validity instead of assuming a fixed sleep.
        bag_path = output / ('bag_' + str(time.time_ns()))
        bag = start(['ros2', 'bag', 'record', '-o', str(bag_path), '/joint_states', '/tf', '/tf_static', '/mujoco/joint_states', '/mujoco/joint_commands', '/hardware/safety_status', '/hardware/control_period'], 'bag_record.log')
        command = ['ros2', 'run', 'embodied_swarm_hardware', 'acceptance.py', '--backend', args.backend, '--output', str(output / 'acceptance.json')]
        acceptance = start(command, 'acceptance.log')
        acceptance.wait(timeout=80)
        if acceptance.returncode:
            print((output / 'acceptance.log').read_text())
            raise RuntimeError(f'Acceptance exited {acceptance.returncode}')
        stop(bag)
        info = subprocess.run(['ros2', 'bag', 'info', str(bag_path)], capture_output=True, text=True, env=env, check=True)
        (output / 'bag_info.txt').write_text(info.stdout)
        # Replay after shutting down publishers, to a real rclpy subscriber.
        for process in reversed(processes):
            stop(process)
        import rclpy
        from sensor_msgs.msg import JointState
        rclpy.init()
        node = rclpy.create_node('bag_replay_acceptance')
        count = [0]
        node.create_subscription(JointState, '/joint_states', lambda msg: count.__setitem__(0, count[0] + 1), 100)
        replay = start(['ros2', 'bag', 'play', str(bag_path), '--rate', '2'], 'bag_replay.log')
        deadline = time.monotonic() + 30
        while replay.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        stop(replay)
        node.destroy_node()
        rclpy.shutdown()
        assert count[0] > 100, f'Insufficient replay messages {count[0]}'
        (output / 'replay.json').write_text(json.dumps({'status': 'PASS_REAL_ROSBAG', 'joint_state_messages_received': count[0], 'bag_path': bag_path.name}, indent=2))
        print((output / 'acceptance.json').read_text())
        print(f'rosbag replay JointState messages: {count[0]}')
        status('PASS', finished_unix_ns=time.time_ns())
    except Exception as exc:
        status('FAIL', error=f'{type(exc).__name__}: {exc}', finished_unix_ns=time.time_ns())
        (output / 'failure.json').write_text(json.dumps(run_status, indent=2))
        raise
    finally:
        for process in reversed(processes):
            stop(process)
        for stream in streams:
            stream.close()


if __name__ == '__main__':
    main()
