import math

import numpy as np

from embodied_swarm.control.lqr_tracker import (
    build_error_dynamics,
    compute_lqr_gain,
    compute_tracking_error,
    lqr_control,
)
from embodied_swarm.control.pure_pursuit import (
    pure_pursuit_control,
)


Pose = tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def simulate_step(
    pose: Pose,
    linear_velocity: float,
    angular_velocity: float,
    dt: float,
) -> Pose:
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

    yaw_new = normalize_angle(
        yaw
        + angular_velocity * dt
    )

    return (
        x_new,
        y_new,
        yaw_new,
    )


def compute_metrics(
    lateral_errors: list[float],
    yaw_errors: list[float],
    angular_velocities: list[float],
    dt: float,
) -> dict[str, float | None]:

    lateral_array = np.asarray(
        lateral_errors,
        dtype=float,
    )

    yaw_array = np.asarray(
        yaw_errors,
        dtype=float,
    )

    omega_array = np.asarray(
        angular_velocities,
        dtype=float,
    )

    rms_lateral_error = float(
        np.sqrt(
            np.mean(
                lateral_array ** 2
            )
        )
    )

    max_lateral_error = float(
        np.max(
            np.abs(
                lateral_array
            )
        )
    )

    final_lateral_error = float(
        lateral_array[-1]
    )

    rms_yaw_error = float(
        np.sqrt(
            np.mean(
                yaw_array ** 2
            )
        )
    )

    control_effort = float(
        np.sqrt(
            np.mean(
                omega_array ** 2
            )
        )
    )

    settling_time = None

    for i in range(
        len(lateral_array)
    ):
        remaining_lateral = (
            np.abs(
                lateral_array[i:]
            )
        )

        remaining_yaw = (
            np.abs(
                yaw_array[i:]
            )
        )

        if (
            np.all(
                remaining_lateral < 0.05
            )
            and np.all(
                remaining_yaw
                < math.radians(2.0)
            )
        ):
            settling_time = i * dt
            break

    return {
        "rms_lateral_error":
            rms_lateral_error,
        "max_lateral_error":
            max_lateral_error,
        "final_lateral_error":
            final_lateral_error,
        "rms_yaw_error":
            rms_yaw_error,
        "control_effort":
            control_effort,
        "settling_time":
            settling_time,
    }


def run_lqr(
    initial_pose: Pose,
    linear_velocity: float,
    dt: float,
    steps: int,
    max_angular_velocity: float,
) -> tuple[
    list[Pose],
    dict[str, float | None],
]:
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

    pose = initial_pose

    poses = [pose]
    lateral_errors = []
    yaw_errors = []
    angular_velocities = []

    for _ in range(steps):
        # Straight reference:
        #
        # y = 5
        # yaw = 0
        #
        # x follows robot progress because
        # longitudinal error is not part of
        # this baseline LQR state.
        reference_pose = (
            pose[0],
            5.0,
            0.0,
        )

        lateral_error, yaw_error = (
            compute_tracking_error(
                pose,
                reference_pose,
            )
        )

        omega = lqr_control(
            pose=pose,
            reference_pose=reference_pose,
            K=K,
            max_angular_velocity=(
                max_angular_velocity
            ),
        )

        lateral_errors.append(
            lateral_error
        )

        yaw_errors.append(
            yaw_error
        )

        angular_velocities.append(
            omega
        )

        pose = simulate_step(
            pose=pose,
            linear_velocity=linear_velocity,
            angular_velocity=omega,
            dt=dt,
        )

        poses.append(pose)

    # Record final state error.
    reference_pose = (
        pose[0],
        5.0,
        0.0,
    )

    final_lateral, final_yaw = (
        compute_tracking_error(
            pose,
            reference_pose,
        )
    )

    lateral_errors.append(
        final_lateral
    )

    yaw_errors.append(
        final_yaw
    )

    # Metrics need equal control/error length.
    angular_velocities.append(
        angular_velocities[-1]
    )

    metrics = compute_metrics(
        lateral_errors,
        yaw_errors,
        angular_velocities,
        dt,
    )

    return poses, metrics


def run_pure_pursuit(
    initial_pose: Pose,
    linear_velocity: float,
    dt: float,
    steps: int,
    max_angular_velocity: float,
    lookahead_distance: float,
) -> tuple[
    list[Pose],
    dict[str, float | None],
]:

    # Dense straight reference path:
    #
    # x: 0 -> 20 m
    # y: 5 m
    path = [
        (
            i * 0.1,
            5.0,
        )
        for i in range(201)
    ]

    pose = initial_pose

    poses = [pose]
    lateral_errors = []
    yaw_errors = []
    angular_velocities = []

    for _ in range(steps):
        reference_pose = (
            pose[0],
            5.0,
            0.0,
        )

        lateral_error, yaw_error = (
            compute_tracking_error(
                pose,
                reference_pose,
            )
        )

        _, omega = pure_pursuit_control(
            robot_pose=pose,
            path=path,
            lookahead_distance=(
                lookahead_distance
            ),
            linear_speed=linear_velocity,
        )

        # Fair comparison:
        # both controllers get the same
        # angular velocity limit.
        omega = float(
            np.clip(
                omega,
                -max_angular_velocity,
                max_angular_velocity,
            )
        )

        lateral_errors.append(
            lateral_error
        )

        yaw_errors.append(
            yaw_error
        )

        angular_velocities.append(
            omega
        )

        pose = simulate_step(
            pose=pose,
            linear_velocity=linear_velocity,
            angular_velocity=omega,
            dt=dt,
        )

        poses.append(pose)

    reference_pose = (
        pose[0],
        5.0,
        0.0,
    )

    final_lateral, final_yaw = (
        compute_tracking_error(
            pose,
            reference_pose,
        )
    )

    lateral_errors.append(
        final_lateral
    )

    yaw_errors.append(
        final_yaw
    )

    angular_velocities.append(
        angular_velocities[-1]
    )

    metrics = compute_metrics(
        lateral_errors,
        yaw_errors,
        angular_velocities,
        dt,
    )

    return poses, metrics


def print_metrics(
    name: str,
    metrics: dict[str, float | None],
) -> None:

    print(name)

    print(
        "RMS lateral error :",
        round(
            metrics[
                "rms_lateral_error"
            ],
            4,
        ),
        "m",
    )

    print(
        "max lateral error :",
        round(
            metrics[
                "max_lateral_error"
            ],
            4,
        ),
        "m",
    )

    print(
        "final lat. error  :",
        round(
            metrics[
                "final_lateral_error"
            ],
            6,
        ),
        "m",
    )

    print(
        "RMS yaw error     :",
        round(
            math.degrees(
                metrics[
                    "rms_yaw_error"
                ]
            ),
            4,
        ),
        "deg",
    )

    print(
        "RMS control effort:",
        round(
            metrics[
                "control_effort"
            ],
            4,
        ),
        "rad/s",
    )

    settling_time = metrics[
        "settling_time"
    ]

    if settling_time is None:
        print(
            "settling time     :",
            "NOT SETTLED",
        )
    else:
        print(
            "settling time     :",
            round(
                settling_time,
                3,
            ),
            "s",
        )


def main() -> None:
    linear_velocity = 1.0
    dt = 0.1
    steps = 100

    max_angular_velocity = 1.5

    # Same disturbance for both controllers:
    #
    # 1 m lateral error
    # +15 deg yaw error
    initial_pose = (
        0.0,
        6.0,
        math.radians(15.0),
    )

    lqr_poses, lqr_metrics = run_lqr(
        initial_pose=initial_pose,
        linear_velocity=linear_velocity,
        dt=dt,
        steps=steps,
        max_angular_velocity=(
            max_angular_velocity
        ),
    )

    pp_poses, pp_metrics = (
        run_pure_pursuit(
            initial_pose=initial_pose,
            linear_velocity=linear_velocity,
            dt=dt,
            steps=steps,
            max_angular_velocity=(
                max_angular_velocity
            ),
            lookahead_distance=1.0,
        )
    )

    print(
        "=== Pure Pursuit vs LQR ==="
    )

    print()
    print_metrics(
        "Pure Pursuit",
        pp_metrics,
    )

    print()
    print_metrics(
        "LQR",
        lqr_metrics,
    )

    print()
    print("=== Comparison ===")

    if (
        lqr_metrics["rms_lateral_error"]
        <
        pp_metrics["rms_lateral_error"]
    ):
        print(
            "RMS lateral error : LQR LOWER"
        )
    else:
        print(
            "RMS lateral error : Pure Pursuit LOWER"
        )

    if (
        lqr_metrics["control_effort"]
        <
        pp_metrics["control_effort"]
    ):
        print(
            "Control effort    : LQR LOWER"
        )
    else:
        print(
            "Control effort    : Pure Pursuit LOWER"
        )

    print()
    print(
        "Pure Pursuit final pose:",
        pp_poses[-1],
    )

    print(
        "LQR final pose         :",
        lqr_poses[-1],
    )

    assert (
        abs(
            lqr_metrics[
                "final_lateral_error"
            ]
        )
        < 0.05
    )

    assert (
        abs(
            pp_metrics[
                "final_lateral_error"
            ]
        )
        < 0.05
    )

    print()
    print(
        "Controller benchmark: PASS"
    )


if __name__ == "__main__":
    main()