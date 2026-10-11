#!/usr/bin/env python3
"""Genuine ROS action/service/TF acceptance; emits measured evidence JSON."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from std_srvs.srv import SetBool, Trigger
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener


class Acceptance(Node):
    def __init__(self, backend='mock'):
        super().__init__('embodied_ros_acceptance')
        self.state = {}
        self.backend = backend
        self.physics_state = {}
        self.latencies, self.arrivals, self.status = [], [], ''
        self.periods = []
        self.create_subscription(Float64, '/hardware/control_period', lambda m: self.periods.append(m.data * 1000) if m.data > 0 else None, 100)
        self.create_subscription(JointState, '/joint_states', self.joints, 100)
        self.create_subscription(JointState, '/mujoco/joint_states', lambda m: self.physics_state.update(dict(zip(m.name, m.position))), 100)
        self.create_subscription(String, '/hardware/safety_status', lambda m: setattr(self, 'status', m.data), 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def joints(self, msg):
        self.state.update(dict(zip(msg.name, msg.position)))
        self.arrivals.append(time.monotonic())
        self.latencies.append((self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds) / 1e6)

    def wait(self, future, timeout=30):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS request did not complete')
        return future.result()

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def service(self, typ, name, request):
        client = self.create_client(typ, name)
        assert client.wait_for_service(timeout_sec=10), name
        return self.wait(client.call_async(request))

    def action(self, typ, name, goal, timeout=30):
        client = ActionClient(self, typ, name)
        assert client.wait_for_server(timeout_sec=20), name
        handle = self.wait(client.send_goal_async(goal))
        assert handle.accepted, f'Goal rejected: {name}'
        return self.wait(handle.get_result_async(), timeout)

    def run(self):
        self.pump(3)
        names = [f'panda_joint{i}' for i in range(1, 8)]
        assert all(n in self.state for n in names), 'Seven real JointState interfaces required'
        assert self.service(Trigger, '/hardware/reset_safety', Trigger.Request()).success
        self.pump(0.1)
        transform = self.tf.lookup_transform('panda_link0', 'panda_link8', rclpy.time.Time())
        assert transform.child_frame_id == 'panda_link8'
        goal = MoveGroup.Goal()
        goal.request.group_name = 'panda_arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 1
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.start_state.is_diff = True
        target = [self.state[n] for n in names]
        target[0] += 0.25
        constraints = Constraints()
        constraints.joint_constraints = [JointConstraint(joint_name=n, position=q, tolerance_above=0.001, tolerance_below=0.001, weight=1.0) for n, q in zip(names, target)]
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        t0 = time.perf_counter()
        planned = self.action(MoveGroup, '/move_action', goal).result
        plan_wall_ms = (time.perf_counter() - t0) * 1000
        assert planned.error_code.val == 1, f'MoveIt planning failed {planned.error_code.val}'
        assert len(planned.planned_trajectory.joint_trajectory.points) > 1
        t0 = time.perf_counter()
        executed = self.action(ExecuteTrajectory, '/execute_trajectory', ExecuteTrajectory.Goal(trajectory=planned.planned_trajectory)).result
        execution_wall_s = time.perf_counter() - t0
        assert executed.error_code.val == 1, f'Trajectory failed {executed.error_code.val}'
        self.pump(0.5)
        error = [self.state[n] - q for n, q in zip(names, target)]
        assert max(abs(e) for e in error) < 0.05, error
        nominal_latencies = self.latencies.copy()
        nominal_intervals = np.diff(self.arrivals) * 1000
        # Controller rejects a malformed joint set instead of accepting unsafe I/O.
        bad = FollowJointTrajectory.Goal()
        bad.trajectory.joint_names = ['nonexistent_joint']
        bad.trajectory.points = [JointTrajectoryPoint(positions=[0.2], time_from_start=Duration(sec=1))]
        client = ActionClient(self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory')
        assert client.wait_for_server(timeout_sec=5)
        handle = self.wait(client.send_goal_async(bad))
        assert not handle.accepted, 'Malformed action should be rejected'
        assert self.service(SetBool, '/hardware/software_estop', SetBool.Request(data=True)).success
        self.pump(0.15)
        assert self.status == 'SOFTWARE_ESTOP_HOLD', self.status
        stopped_goal = FollowJointTrajectory.Goal()
        stopped_goal.trajectory.joint_names = names
        stopped_target = [self.state[n] for n in names]
        stopped_target[0] += 0.5
        stopped_goal.trajectory.points = [JointTrajectoryPoint(positions=stopped_target, time_from_start=Duration(sec=1))]
        failed = self.action(FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory', stopped_goal, timeout=10)
        assert failed.result.error_code != 0, 'Estop trajectory must fail, not report success'
        assert not self.service(SetBool, '/hardware/software_estop', SetBool.Request(data=False)).success
        assert self.service(Trigger, '/hardware/reset_safety', Trigger.Request()).success
        self.pump(0.15)
        stale_state_pass = None
        physics_match = None
        if self.backend == 'mujoco':
            assert all(n in self.physics_state for n in names)
            physics_match = max(abs(self.state[n] - self.physics_state[n]) for n in names)
            assert physics_match < 0.01
            assert self.service(SetBool, '/mujoco/pause_state', SetBool.Request(data=True)).success
            self.pump(0.4)
            assert self.status == 'SOFTWARE_ESTOP_HOLD'
            assert not self.service(Trigger, '/hardware/reset_safety', Trigger.Request()).success
            assert self.service(SetBool, '/mujoco/pause_state', SetBool.Request(data=False)).success
            self.pump(0.2)
            assert self.service(Trigger, '/hardware/reset_safety', Trigger.Request()).success
            stale_state_pass = True
        intervals = np.diff(self.arrivals) * 1000
        stats = lambda values: {f'p{p}': float(np.percentile(values, p)) for p in (50, 95, 99)}
        return {'status': 'PASS_REAL_SIM_ROS' if self.backend == 'mujoco' else 'PASS_REAL_ROS_MOCK_HARDWARE', 'backend': self.backend,
                'planner': 'OMPL geometric::RRTConnect', 'plan_error_code': planned.error_code.val,
                'execute_error_code': executed.error_code.val, 'trajectory_points': len(planned.planned_trajectory.joint_trajectory.points),
                'plan_wall_ms': plan_wall_ms, 'execution_wall_s': execution_wall_s,
                'final_joint_error_rad': error, 'max_joint_error_rad': max(abs(e) for e in error),
                'tf2_pass': True, 'invalid_action_rejected': True, 'software_estop_reset_pass': True,
                'estop_trajectory_failure_code': failed.result.error_code, 'state_timeout_reset_rejection_recovery': stale_state_pass,
                'ros_vs_physics_joint_max_error_rad': physics_match,
                'latency_ms': stats(self.latencies), 'sample_count': len(self.latencies),
                'nominal_latency_ms': stats(nominal_latencies), 'nominal_sample_count': len(nominal_latencies),
                'nominal_interval_ms': stats(nominal_intervals), 'nominal_jitter_std_ms': float(np.std(nominal_intervals)),
                'joint_state_interval_ms': stats(intervals), 'interval_jitter_std_ms': float(np.std(intervals)),
                'controller_manager_period_ms': stats(self.periods), 'control_period_jitter_std_ms': float(np.std(self.periods)),
                'measurement': 'Same WSL host wall clock ROS stamp-to-callback; interarrival includes DDS/executor scheduling. Not RT/HIL.'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--backend', choices=['mock', 'mujoco'], default='mock')
    args = parser.parse_args()
    rclpy.init()
    node = Acceptance(args.backend)
    try:
        result = node.run()
        Path(args.output).write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
