"""Cartesian impedance torque with bias compensation and actuator saturation."""
import numpy as np


def cartesian_impedance_torque(jacobian, error, joint_velocity, bias,
                               stiffness=150., damping=25., torque_limits=None):
    J = np.asarray(jacobian, dtype=float)
    e, dq, bias = [np.asarray(v, dtype=float) for v in (error, joint_velocity, bias)]
    if J.ndim != 2 or e.shape != (J.shape[0],) or dq.shape != (J.shape[1],) or bias.shape != dq.shape:
        raise ValueError("impedance input dimensions mismatch")
    if not all(np.isfinite(x).all() for x in (J, e, dq, bias)) or stiffness <= 0 or damping <= 0:
        raise ValueError("finite inputs and positive gains required")
    tau = J.T@(stiffness*e-damping*(J@dq)) + bias
    if torque_limits is not None:
        lim = np.asarray(torque_limits)
        if lim.shape != dq.shape or np.any(lim <= 0):
            raise ValueError("invalid torque limits")
        tau = np.clip(tau, -lim, lim)
    return tau
