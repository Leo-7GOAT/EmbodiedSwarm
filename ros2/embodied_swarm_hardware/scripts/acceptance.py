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
from moveit_msgs.srv import GetStateValidity
from controller_manager_msgs.srv import ListControllers
from mujoco_bridge import arm_initial_positions
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener


class Acceptance(Node):
    def __init__(self, backend='mock'):
        super().__init__('embodied_ros_acceptance')
        self.state = {}
        self.last_joint_message = None
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
        self.last_joint_message = msg
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

    def wait_until(self, condition, description, timeout=30):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Not ready: {description}')
            rclpy.spin_once(self, timeout_sec=0.02)

    def fresh_complete_state(self, names):
        msg = self.last_joint_message
        if msg is None or len(msg.name) != len(msg.position) or not all(n in msg.name for n in names):
            return False
        age = (self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        return 0 <= age <= 0.25 and np.isfinite(msg.position).all()

    def wait_active_controllers(self, names, timeout=30):
        client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        assert client.wait_for_service(timeout_sec=timeout), 'Controller manager required'
        deadline = time.monotonic() + timeout
        states = {}
        while time.monotonic() < deadline:
            result = self.wait(client.call_async(ListControllers.Request()), timeout=max(.1, deadline-time.monotonic()))
            states = {controller.name: controller.state for controller in result.controller}
            if all(states.get(name) == 'active' for name in names):
                return
            self.pump(.05)
        raise TimeoutError(f'Controllers not active: {states}')

    def service(self, typ, name, request):
        client = self.create_client(typ, name)
        assert client.wait_for_service(timeout_sec=10), name
        return self.wait(client.call_async(request))

    def wait_startup_reset(self, timeout=10):
        # JointStateBroadcaster can publish the initialized pose before the
        # hardware's separate physics subscription receives its first packet.
        # Only the hardware service can confirm its own accepted fresh state.
        client = self.create_client(Trigger, '/hardware/reset_safety')
        deadline = time.monotonic() + timeout
        assert client.wait_for_service(timeout_sec=timeout), 'Hardware reset service required'
        attempts, rejected = 0, []
        while time.monotonic() < deadline:
            result = self.wait(client.call_async(Trigger.Request()), timeout=max(.01, deadline-time.monotonic()))
            attempts += 1
            if result.success:
                return {'attempts': attempts, 'rejected_responses': rejected}
            rejected.append(result.message)
            self.pump(min(.02, max(0, deadline-time.monotonic())))
        raise TimeoutError(f'Hardware startup reset never accepted fresh physics state: {rejected}')

    def action(self, typ, name, goal, timeout=30):
        client = ActionClient(self, typ, name)
        assert client.wait_for_server(timeout_sec=20), name
        handle = self.wait(client.send_goal_async(goal))
        assert handle.accepted, f'Goal rejected: {name}'
        return self.wait(handle.get_result_async(), timeout)

    def run(self):
        names = [f'panda_joint{i}' for i in range(1, 8)]
        full_names = names + ['panda_finger_joint1', 'panda_finger_joint2']
        self.wait_until(lambda: self.fresh_complete_state(full_names), 'fresh complete Panda JointState')
        self.wait_active_controllers(('joint_state_broadcaster', 'panda_arm_controller'))
        startup_reset = self.wait_startup_reset()
        self.wait_until(lambda: self.status == f'ACTIVE_{self.backend}' and self.fresh_complete_state(full_names), 'fresh active startup hold')
        self.wait_until(lambda: self.tf.can_transform('panda_link0', 'panda_link8', rclpy.time.Time()), 'Panda TF chain')
        transform = self.tf.lookup_transform('panda_link0', 'panda_link8', rclpy.time.Time())
        assert transform.child_frame_id == 'panda_link8'
        initial = arm_initial_positions()
        startup_error = max(abs(self.state[n] - q) for n, q in zip(names, initial))
        assert startup_error < 0.01, f'Startup must match the shared initial pose: {startup_error}'
        valid_request = GetStateValidity.Request(group_name='panda_arm')
        valid_request.robot_state.joint_state = JointState(
            header=self.last_joint_message.header, name=full_names,
            position=[self.state[n] for n in full_names])
        validity = self.service(GetStateValidity, '/check_state_validity', valid_request)
        assert validity.valid, f'Measured startup state invalid: {validity.contacts}'
        # Shared Xacro loading and readiness service work are startup work,
        # not the planning/execution measurement window. Keep them in the
        # whole-run distribution but identify nominal samples separately.
        self.pump(0.1)
        nominal_start = len(self.latencies)
        nominal_arrival_start = len(self.arrivals)
        goal = MoveGroup.Goal()
        goal.request.group_name = 'panda_arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 1
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.start_state = valid_request.robot_state
        goal.request.start_state.is_diff = False
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
        nominal_latencies = self.latencies[nominal_start:].copy()
        nominal_intervals = np.diff(self.arrivals[nominal_arrival_start:]) * 1000
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
                'startup_state_validity_pass': validity.valid, 'startup_home_max_error_rad': startup_error,
                'startup_hardware_reset': startup_reset,
                'estop_trajectory_failure_code': failed.result.error_code, 'state_timeout_reset_rejection_recovery': stale_state_pass,
                'ros_vs_physics_joint_max_error_rad': physics_match,
                'latency_ms': stats(self.latencies), 'sample_count': len(self.latencies),
                'nominal_latency_ms': stats(nominal_latencies), 'nominal_sample_count': len(nominal_latencies),
                'startup_sample_count': nominal_start,
                'nominal_measurement_window': 'After complete fresh state/controller/TF/home/state-validity readiness, through planning, execution and 0.5 s settled state; before fault injection',
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
