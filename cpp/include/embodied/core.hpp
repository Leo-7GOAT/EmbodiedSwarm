#pragma once
#include <Eigen/Dense>
#include <array>
namespace embodied {
using JointVector = Eigen::Matrix<double,7,1>;
using Jacobian = Eigen::Matrix<double,6,7>;
struct Pose { Eigen::Vector3d position; Eigen::Matrix3d rotation; };
Pose panda_fk(const JointVector& q);
Jacobian panda_jacobian(const JointVector& q);
struct Command { double speed; double yaw_rate; };
Command pursuit(double x, double y, double yaw, double target_x, double target_y,
                double speed, double max_yaw_rate);
JointVector impedance(const Jacobian& J, const Eigen::Matrix<double,6,1>& error,
                      const JointVector& velocity, const JointVector& bias,
                      double stiffness, double damping, const JointVector& limits);
} // namespace embodied
