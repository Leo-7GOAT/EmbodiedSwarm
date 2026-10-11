"""Independent serial-chain FK, geometric Jacobian and bounded DLS IK.

MuJoCo provides static model geometry only. NumPy computes each transform;
tests cross-check against mj_forward/mj_jacSite and finite differences.
"""
from dataclasses import dataclass
import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K@K)


def quat_matrix(quat):
    w, x, y, z = np.asarray(quat) / np.linalg.norm(quat)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


@dataclass
class IKResult:
    q: np.ndarray
    success: bool
    iterations: int
    position_error: float
    orientation_error: float
    reason: str


class SerialKinematics:
    def __init__(self, model, site_name="grasp_site", joint_names=None):
        import mujoco
        self.site = model.site(site_name).id
        names = joint_names or [f"joint{i}" for i in range(1, 8)]
        self.ids = [model.joint(name).id for name in names]
        self.limits = model.jnt_range[self.ids].copy()
        self.qpos = model.jnt_qposadr[self.ids].copy()
        self.dofs = model.jnt_dofadr[self.ids].copy()
        body = int(model.site_bodyid[self.site])
        chain = []
        while body:
            chain.append(body)
            body = int(model.body_parentid[body])
        self.chain = []
        for body in reversed(chain):
            idx = next((i for i, j in enumerate(self.ids) if model.jnt_bodyid[j] == body), None)
            self.chain.append((model.body_pos[body].copy(), quat_matrix(model.body_quat[body]),
                               idx, None if idx is None else model.jnt_axis[self.ids[idx]].copy()))
        self.site_pos = model.site_pos[self.site].copy()
        self.site_rot = quat_matrix(model.site_quat[self.site])

    def _evaluate(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.ids),) or not np.isfinite(q).all():
            raise ValueError("joint vector has wrong shape or non-finite values")
        p, R = np.zeros(3), np.eye(3)
        origins, axes = [], []
        for offset, fixed_rot, idx, axis in self.chain:
            p = p + R@offset
            R = R@fixed_rot
            if idx is not None:
                origins.append(p.copy())
                axes.append(R@axis)
                R = R@rotation(axis, q[idx])
        p = p + R@self.site_pos
        return p, R@self.site_rot, origins, axes

    def fk(self, q):
        p, R, _, _ = self._evaluate(q)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, p
        return T

    def jacobian(self, q):
        p, _, origins, axes = self._evaluate(q)
        return np.vstack((np.array([np.cross(a, p-o) for a, o in zip(axes, origins)]).T,
                          np.asarray(axes).T))

    def manipulability(self, q):
        singular = np.linalg.svd(self.jacobian(q), compute_uv=False)
        return float(np.prod(singular))

    def ik(self, target_position, seed, target_rotation=None, max_iterations=300,
           tolerance=2e-4, damping=0.01):
        target = np.asarray(target_position, dtype=float)
        seed = np.asarray(seed, dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("target must be finite xyz")
        self.fk(seed)  # validates before clamping
        if max_iterations <= 0 or damping <= 0 or tolerance <= 0:
            raise ValueError("positive solver settings required")
        if target_rotation is not None:
            target_rotation = np.asarray(target_rotation, dtype=float)
            if target_rotation.shape != (3, 3) or not np.allclose(target_rotation.T@target_rotation, np.eye(3), atol=1e-6) or np.linalg.det(target_rotation) < 0:
                raise ValueError("target_rotation must be a proper rotation")
        q = np.clip(seed, self.limits[:, 0], self.limits[:, 1])
        for iteration in range(max_iterations):
            T = self.fk(q)
            ep = target-T[:3, 3]
            er = np.zeros(3)
            J = self.jacobian(q)[:3]
            if target_rotation is not None:
                # Log SO(3), valid at pi where cross-product error alone vanishes.
                from scipy.spatial.transform import Rotation
                er = Rotation.from_matrix(target_rotation@T[:3, :3].T).as_rotvec()
                J = self.jacobian(q)
            e = ep if target_rotation is None else np.r_[ep, er]
            pe, re = float(np.linalg.norm(ep)), float(np.linalg.norm(er))
            if pe < tolerance and re < tolerance*5:
                return IKResult(q, True, iteration, pe, re, "converged")
            sigma = np.linalg.svd(J, compute_uv=False)[-1]
            lam = damping + max(0., 0.03-sigma)
            dq = J.T @ np.linalg.solve(J@J.T + lam*lam*np.eye(len(e)), e)
            dq *= min(1., 0.12/max(1e-12, np.linalg.norm(dq)))
            q = np.clip(q+dq, self.limits[:, 0], self.limits[:, 1])
        T = self.fk(q)
        return IKResult(q, False, max_iterations, float(np.linalg.norm(target-T[:3, 3])),
                        float(np.linalg.norm(er)), "iteration_limit_or_unreachable")
