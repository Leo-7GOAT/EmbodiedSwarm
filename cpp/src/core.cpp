#include "embodied/core.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace embodied {
namespace {
constexpr double pi=3.14159265358979323846;
const std::array<Eigen::Vector3d,7> offsets={Eigen::Vector3d(0,0,.333),
 Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,-.316,0),Eigen::Vector3d(.0825,0,0),
 Eigen::Vector3d(-.0825,.384,0),Eigen::Vector3d(0,0,0),Eigen::Vector3d(.088,0,0)};
const std::array<double,7> fixed_x={0,-pi/2,pi/2,pi/2,-pi/2,pi/2,pi/2};
struct Evaluation { Pose pose; std::array<Eigen::Vector3d,7> origins,axes; };
Evaluation evaluate(const JointVector& q) {
 if(!q.allFinite()) throw std::invalid_argument("non-finite joint state");
 Evaluation e;
 Eigen::Vector3d p=Eigen::Vector3d::Zero();
 Eigen::Matrix3d R=Eigen::Matrix3d::Identity();
 for(int i=0;i<7;++i) {
  p+=R*offsets[i];
  R=R*Eigen::AngleAxisd(fixed_x[i],Eigen::Vector3d::UnitX()).toRotationMatrix();
  e.origins[i]=p; e.axes[i]=R*Eigen::Vector3d::UnitZ();
  R=R*Eigen::AngleAxisd(q[i],Eigen::Vector3d::UnitZ()).toRotationMatrix();
 }
 p+=R*Eigen::Vector3d(0,0,.107);
 // Upstream rounded quaternion is normalized, avoiding hidden parity drift.
 Eigen::Quaterniond hand(.9238795,0,0,-.3826834);
 R=R*hand.normalized().toRotationMatrix();
 p+=R*Eigen::Vector3d(0,0,.1034);
 e.pose={p,R}; return e;
}
}
Pose panda_fk(const JointVector& q) { return evaluate(q).pose; }
Jacobian panda_jacobian(const JointVector& q) {
 const auto e=evaluate(q); Jacobian J;
 for(int i=0;i<7;++i) {
  J.block<3,1>(0,i)=e.axes[i].cross(e.pose.position-e.origins[i]);
  J.block<3,1>(3,i)=e.axes[i];
 }
 return J;
}
Command pursuit(double x,double y,double yaw,double tx,double ty,double speed,double max_rate) {
 if(!std::isfinite(x+y+yaw+tx+ty+speed+max_rate)||max_rate<=0)
  throw std::invalid_argument("invalid pursuit input");
 const double dx=tx-x,dy=ty-y,d2=dx*dx+dy*dy;
 if(d2<1e-12) return {0,0};
 const double lateral=-std::sin(yaw)*dx+std::cos(yaw)*dy;
 return {speed,std::clamp(2*speed*lateral/d2,-max_rate,max_rate)};
}
JointVector impedance(const Jacobian& J,const Eigen::Matrix<double,6,1>& error,
 const JointVector& velocity,const JointVector& bias,double kp,double kd,const JointVector& limits) {
 if(!J.allFinite()||!error.allFinite()||!velocity.allFinite()||!bias.allFinite()||
    !limits.allFinite()||limits.minCoeff()<=0||kp<=0||kd<=0)
  throw std::invalid_argument("invalid impedance input");
 const JointVector tau=J.transpose()*(kp*error-kd*(J*velocity))+bias;
 return tau.cwiseMax(-limits).cwiseMin(limits);
}
}
