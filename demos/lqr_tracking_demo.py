import math

import numpy as np

from embodied_swarm.control.lqr_tracker import (
    build_error_dynamics,
    compute_lqr_gain,
    compute_tracking_error,
    lqr_control,
)


def main() -> None:
    # ==========================================
    # Controller configuration
    # ==========================================

    linear_velocity = 1.0
    dt = 0.1

    A, B = build_error_dynamics(
        linear_velocity=linear_velocity,
        dt=dt,
    )

    Q = np.diag([
        2.0,
        1.0,
    ])

    R = np.array([
        [0.5],
    ])

    K = compute_lqr_gain(
        A,
        B,
        Q,
        R,
    )

    print("K:", K)

    # ==========================================
    # Straight reference path
    #
    # Desired path:
    #
    # y = 5
    # yaw = 0
    #
    # Robot starts 1 m above the path.
    # ==========================================

    pose = (
        0.0,
        6.0,
        0.0,
    )

    reference_pose = (
        0.0,
        5.0,
        0.0,
    )

    print()
    print("initial pose:", pose)

    lateral_error, yaw_error = (
        compute_tracking_error(
            pose,
            reference_pose,
        )
    )

    print(
        "initial lateral error:",
        lateral_error,
    )

    print(
        "initial yaw error:",
        yaw_error,
    )

    # ==========================================
    # Closed-loop simulation
    # ==========================================

    history = [pose]

    for step in range(100):

        omega = lqr_control(
            pose=pose,
            reference_pose=reference_pose,
            K=K,
            max_angular_velocity=1.5,
        )

        x, y, yaw = pose

        x_new = (
            x
            + linear_velocity
            * math.cos(yaw)
            * dt
        )

        y_new = (
            y
            + linear_velocity
            * math.sin(yaw)
            * dt
        )

        yaw_new = (
            yaw
            + omega * dt
        )

        yaw_new = math.atan2(
            math.sin(yaw_new),
            math.cos(yaw_new),
        )

        pose = (
            x_new,
            y_new,
            yaw_new,
        )

        history.append(pose)

        # Print every second
        if step % 10 == 0:
            lateral_error, yaw_error = (
                compute_tracking_error(
                    pose,
                    reference_pose,
                )
            )

            print(
                f"step={step:3d}",
                f"y={pose[1]:.4f}",
                f"yaw={math.degrees(pose[2]):.3f} deg",
                f"e_y={lateral_error:.4f}",
                f"e_yaw={math.degrees(yaw_error):.3f} deg",
                f"omega={omega:.4f}",
            )

    # ==========================================
    # Final validation
    # ==========================================

    final_lateral_error, final_yaw_error = (
        compute_tracking_error(
            pose,
            reference_pose,
        )
    )

    print()
    print("final pose:", pose)

    print(
        "final lateral error:",
        final_lateral_error,
    )

    print(
        "final yaw error:",
        final_yaw_error,
    )

    assert abs(final_lateral_error) < 0.05
    assert abs(final_yaw_error) < math.radians(2.0)

    print()
    print("LQR tracking convergence: PASS")


if __name__ == "__main__":
    main()