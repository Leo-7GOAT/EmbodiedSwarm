"""Latched software safety policy in simulation time (seconds, SI units)."""
from dataclasses import dataclass
import numpy as np
from embodied_swarm.core.robot_command import RobotCommand


@dataclass(frozen=True)
class SafetyLimits:
    max_speed: float = 0.6
    max_yaw_rate: float = 1.2
    timeout: float = 0.25
    geofence: tuple = (-4., 4., -4., 4.)
    max_joint_velocity: float = 1.5
    control_period: float = .02


class SafetySupervisor:
    def __init__(self, limits=SafetyLimits(), joint_limits=None):
        numeric = [limits.max_speed, limits.max_yaw_rate, limits.timeout,
                   limits.max_joint_velocity, limits.control_period]
        if not np.isfinite(numeric).all() or min(numeric) <= 0:
            raise ValueError("safety limits must be positive")
        fence = np.asarray(limits.geofence)
        if fence.shape != (4,) or not np.isfinite(fence).all() or fence[0] >= fence[1] or fence[2] >= fence[3]:
            raise ValueError("geofence must contain finite ordered bounds")
        self.joint_limits = None if joint_limits is None else np.asarray(joint_limits,dtype=float).copy()
        if self.joint_limits is not None and (self.joint_limits.shape != (7,2) or not np.isfinite(self.joint_limits).all() or np.any(self.joint_limits[:,0] >= self.joint_limits[:,1])):
            raise ValueError("joint_limits must be finite (7,2) ordered bounds")
        self.limits = limits
        self.latched_reason = None
        self.interventions = 0
        self.manual_override = False

    def emergency_stop(self, reason="software_estop"):
        self.latched_reason = reason

    def reset(self, state, now):
        # Recovery requires a fresh stationary state inside the fence.
        if self._invalid_state(state, now) or np.linalg.norm(state.base_twist) > 0.02:
            return False
        self.latched_reason = None
        return True

    def _invalid_state(self, state, now):
        x, y = state.base_pose[:2]
        xmin, xmax, ymin, ymax = self.limits.geofence
        if not np.isfinite(now) or not 0 <= now-state.timestamp <= self.limits.timeout:
            return "stale_state"
        if not xmin <= x <= xmax or not ymin <= y <= ymax:
            return "geofence"
        if self.joint_limits is not None and np.any((state.joint_positions < self.joint_limits[:,0]) | (state.joint_positions > self.joint_limits[:,1])):
            return "joint_state_limit"
        return None

    def filter(self, state, command, now):
        reason = self._invalid_state(state, now)
        if not 0 <= now-command.timestamp <= self.limits.timeout:
            reason = reason or "command_timeout"
        if self.manual_override:
            reason = reason or "manual_override"
        if reason:
            self.emergency_stop(reason)
        if self.latched_reason:
            self.interventions += 1
            return RobotCommand(timestamp=now, joint_position_targets=state.joint_positions)
        twist = command.base_twist.copy()
        speed = np.linalg.norm(twist[:2])
        if speed > self.limits.max_speed:
            twist[:2] *= self.limits.max_speed/speed
        twist[2] = np.clip(twist[2], -self.limits.max_yaw_rate, self.limits.max_yaw_rate)
        if not np.array_equal(twist, command.base_twist):
            self.interventions += 1
        target = command.joint_position_targets
        if target is not None and self.joint_limits is not None:
            if np.any((target < self.joint_limits[:,0]) | (target > self.joint_limits[:,1])):
                self.emergency_stop("joint_command_limit")
                self.interventions += 1
                return RobotCommand(timestamp=now, joint_position_targets=state.joint_positions)
            delta = self.limits.max_joint_velocity*self.limits.control_period
            bounded = np.clip(target,state.joint_positions-delta,state.joint_positions+delta)
            if not np.array_equal(bounded,target):
                self.interventions += 1
            target = bounded
        return RobotCommand(timestamp=now, base_twist=twist,
                            joint_position_targets=target,
                            gripper_width_target=command.gripper_width_target)
