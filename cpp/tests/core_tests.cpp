#include "embodied/core.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace embodied;
TEST(Kinematics, HomeAndFiniteDifference) {
 JointVector q; q<<0,0,0,-1.57079,0,1.57079,-.7853;
 auto p=panda_fk(q);
 EXPECT_NEAR(p.position.x(),.5545,1e-4);
 EXPECT_NEAR(p.position.y(),0,1e-12);
 auto J=panda_jacobian(q);
 for(int i=0;i<7;++i) {
  auto plus=q,minus=q; plus[i]+=1e-6;minus[i]-=1e-6;
  EXPECT_LT(((panda_fk(plus).position-panda_fk(minus).position)/2e-6-J.block<3,1>(0,i)).norm(),1e-8);
 }
 EXPECT_NEAR(p.rotation.determinant(),1,1e-12);
}
TEST(Kinematics, InvalidJointState) {
 auto q=JointVector::Zero().eval();q[0]=std::numeric_limits<double>::quiet_NaN();
 EXPECT_THROW(panda_fk(q),std::invalid_argument);
}
TEST(Control, PursuitSaturationAndGoal) {
 EXPECT_DOUBLE_EQ(pursuit(0,0,0,0,0,.3,1).speed,0);
 EXPECT_DOUBLE_EQ(pursuit(0,0,0,0,.01,.3,1).yaw_rate,1);
 EXPECT_DOUBLE_EQ(pursuit(0,0,0,1,0,.3,1).yaw_rate,0);
 EXPECT_THROW(pursuit(0,0,0,1,1,.3,0),std::invalid_argument);
}
TEST(Control, ImpedanceBiasDampingAndLimits) {
 Jacobian J=Jacobian::Zero();J.block<6,6>(0,0)=Eigen::Matrix<double,6,6>::Identity();
 auto e=Eigen::Matrix<double,6,1>::Zero().eval();
 auto dq=JointVector::Ones().eval(),bias=JointVector::Zero().eval(),limits=JointVector::Ones().eval();
 EXPECT_LT(impedance(J,e,dq,bias,150,25,limits).dot(dq),0);
 e.setOnes();dq.setZero();
 EXPECT_LE(impedance(J,e,dq,bias,150,25,limits).cwiseAbs().maxCoeff(),1);
 limits[0]=0;EXPECT_THROW(impedance(J,e,dq,bias,150,25,limits),std::invalid_argument);
}
