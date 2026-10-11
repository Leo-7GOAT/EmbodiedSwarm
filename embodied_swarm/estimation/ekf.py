"""Planar encoder/IMU EKF with causal delayed-state extrapolation.

State is [world x, world y, yaw, body vx, body vy, gyro_z bias]. No pose truth
or commanded velocity is accepted by this estimator. Accepted measurements
advance the filter at sensor time; controller queries extrapolate a copy.
"""
from dataclasses import dataclass
import numpy as np
from embodied_swarm.core.robot_state import RobotState


@dataclass(frozen=True)
class SensorPacket:
    stamp: float
    sequence: int
    left_ticks: int
    right_ticks: int
    gyro_z: float
    accel_x: float
    accel_y: float
    # Cumulative sampled increments, including samples lost during transport.
    # A sampling outage is different: it cannot be repaired by this packet.
    gyro_integral: float
    accel_x_integral: float
    accel_y_integral: float


@dataclass(frozen=True)
class EncoderGeometry:
    wheel_radius_m: float = .05
    track_m: float = .32
    ticks_per_revolution: int = 4096

    def __post_init__(self):
        if not np.isfinite([self.wheel_radius_m,self.track_m]).all() or min(self.wheel_radius_m,self.track_m)<=0 or type(self.ticks_per_revolution) is not int or self.ticks_per_revolution<=0:
            raise ValueError("finite positive encoder geometry required")


def transition(x, gyro, acceleration, dt):
    out=x.copy()
    omega=gyro-x[5]
    a=np.asarray(acceleration)-omega*np.array([-x[4],x[3]])
    vmid=x[3:5]+.5*dt*a
    mid=x[2]+.5*omega*dt
    R=np.array([[np.cos(mid),-np.sin(mid)],[np.sin(mid),np.cos(mid)]])
    out[:2]+=dt*(R@vmid)
    out[2]+=dt*omega
    out[3:5]+=dt*a
    return out


class EncoderImuEKF:
    def __init__(self,geometry=EncoderGeometry(),gyro_bias=0.,max_age=.15,lateral_time_constant_s=.1):
        if not np.isfinite([gyro_bias,max_age,lateral_time_constant_s]).all() or max_age<=0 or lateral_time_constant_s<0:
            raise ValueError("finite bias and positive age limit required")
        self.geometry=geometry
        self.max_age=max_age
        self.lateral_time_constant=lateral_time_constant_s
        self.x=np.array([0.,0.,0.,0.,0.,gyro_bias])
        self.P=np.diag([1e-6,1e-6,1e-5,.01,.01,1e-4])
        self.last_packet=None
        self.last_gyro=gyro_bias
        self.last_accel=np.zeros(2)
        self.accepted=0
        self.rejected={}

    def reject(self,reason):
        self.rejected[reason]=self.rejected.get(reason,0)+1
        return False,reason

    def ingest(self,packet,now):
        values=[packet.stamp,packet.gyro_z,packet.accel_x,packet.accel_y,
                packet.gyro_integral,packet.accel_x_integral,packet.accel_y_integral,now]
        if not np.isfinite(values).all():
            return self.reject("nonfinite")
        integers=(packet.sequence,packet.left_ticks,packet.right_ticks)
        if packet.stamp<0 or any(type(value) is not int or abs(value)>2**63-1 for value in integers) or packet.sequence<0:
            return self.reject("invalid_encoding")
        if abs(packet.gyro_z)>10. or np.linalg.norm([packet.accel_x,packet.accel_y])>15.:
            return self.reject("sensor_outlier")
        if packet.stamp>now+1e-8:
            return self.reject("future_timestamp")
        if now-packet.stamp>self.max_age:
            return self.reject("stale_packet")
        if self.last_packet is not None:
            if packet.stamp<=self.last_packet.stamp or packet.sequence<=self.last_packet.sequence:
                return self.reject("duplicate_or_out_of_order")
            dt=packet.stamp-self.last_packet.stamp
            g=self.geometry
            meter_per_tick=2*np.pi*g.wheel_radius_m/g.ticks_per_revolution
            left=(packet.left_ticks-self.last_packet.left_ticks)*meter_per_tick/dt
            right=(packet.right_ticks-self.last_packet.right_ticks)*meter_per_tick/dt
            vx=.5*(left+right)
            omega=(right-left)/g.track_m
            if abs(vx)>2. or abs(omega)>6. or np.linalg.norm([packet.accel_x,packet.accel_y])>15.:
                return self.reject("sensor_outlier")
            # Cumulative IMU integrals are computed by the sensor sampling
            # front end, so transport loss does not erase past increments.
            gyro=(packet.gyro_integral-self.last_packet.gyro_integral)/dt
            acceleration=np.array([packet.accel_x_integral-self.last_packet.accel_x_integral,
                                   packet.accel_y_integral-self.last_packet.accel_y_integral])/dt
            if abs(gyro)>10. or np.linalg.norm(acceleration)>15.:
                return self.reject("sensor_outlier")
            self._update(np.array([gyro-omega]),np.array([[0,0,0,0,0,1.]]),np.array([[4e-4]]))
            eps=1e-6
            F=np.column_stack([(transition(self.x+np.eye(6)[i]*eps,gyro,acceleration,dt)-transition(self.x-np.eye(6)[i]*eps,gyro,acceleration,dt))/(2*eps) for i in range(6)])
            self.x=transition(self.x,gyro,acceleration,dt)
            self.P=F@self.P@F.T+np.diag([1e-7,1e-7,2e-6,1e-3,1e-3,1e-7])*dt
            self._update(np.array([vx]),np.array([[0,0,0,1.,0,0]]),np.array([[1e-5]]))
            # Planar velocity-servo model prior, not a false no-slip wheel
            # constraint. Under steady turning vy ~= -tau*omega*vx. Tau is
            # independently identified from physical encoder step response.
            lateral=-self.lateral_time_constant*(packet.gyro_z-self.x[5])*vx
            self._update(np.array([lateral]),np.array([[0,0,0,0,1.,0]]),np.array([[1e-4]]))
        self.last_packet=packet
        self.last_gyro=packet.gyro_z
        self.last_accel=np.array([packet.accel_x,packet.accel_y])
        self.accepted+=1
        return True,"accepted"

    def _update(self,z,H,R):
        innovation=z-H@self.x
        S=H@self.P@H.T+R
        K=np.linalg.solve(S,(self.P@H.T).T).T
        self.x+=K@innovation
        I=np.eye(len(self.x))
        # Joseph form preserves covariance PSD under finite precision.
        self.P=(I-K@H)@self.P@(I-K@H).T+K@R@K.T
        self.P=.5*(self.P+self.P.T)

    def measurement_age(self,now):
        return float("inf") if self.last_packet is None else now-self.last_packet.stamp

    def state(self,now):
        if not np.isfinite(now) or self.last_packet is None or now<self.last_packet.stamp:
            raise ValueError("finite query time at or after last measurement required")
        predicted=transition(self.x,self.last_gyro,self.last_accel,now-self.last_packet.stamp)
        return RobotState(timestamp=now,base_pose=predicted[:3],
                          base_twist=[predicted[3],predicted[4],self.last_gyro-predicted[5]])
