"""Noisy virtual encoders and IMU from actual MuJoCo sensor outputs.

The planar base has no physical wheels. Encoder counts are an explicitly
virtual differential-drive encoding of its measured body velocity and yaw
rate, with quantization. IMU gyro/acceleration are built-in MuJoCo sensors.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from embodied_swarm.assets import asset_path
from embodied_swarm.estimation.ekf import SensorPacket,EncoderGeometry


def sensor_model_file(destination):
    tree=ET.parse(asset_path("mjcf/physical_base.xml"))
    body=tree.getroot().find(".//body[@name='base_link']")
    ET.SubElement(body,"site",name="imu_site",pos="0 0 0",size=".005")
    sensors=ET.SubElement(tree.getroot(),"sensor")
    for kind,name in (("velocimeter","ideal_body_velocity"),("gyro","ideal_gyro"),("accelerometer","ideal_acceleration")):
        ET.SubElement(sensors,kind,name=name,site="imu_site")
    path=Path(destination);path.parent.mkdir(parents=True,exist_ok=True)
    tree.write(path,encoding="unicode")
    return path


class SimulatedSensors:
    def __init__(self,seed=7,geometry=EncoderGeometry(),gyro_bias=.025,gyro_sigma=.004,accel_sigma=.008):
        if not np.isfinite([gyro_bias,gyro_sigma,accel_sigma]).all() or min(gyro_sigma,accel_sigma)<0:
            raise ValueError("finite bias and nonnegative sensor noise required")
        self.rng=np.random.default_rng(seed);self.geometry=geometry
        self.gyro_bias,self.gyro_sigma,self.accel_sigma=gyro_bias,gyro_sigma,accel_sigma
        self.phase=np.zeros(2);self.sequence=0;self.last_stamp=0.
        self.imu_integral=np.zeros(3)

    def sample(self,model,data):
        stamp=float(data.time);dt=stamp-self.last_stamp
        if dt<0:
            raise ValueError("sensor clock moved backwards")
        velocity=data.sensor("ideal_body_velocity").data.copy()
        gyro=float(data.sensor("ideal_gyro").data[2])
        acceleration=data.sensor("ideal_acceleration").data[:2].copy()
        g=self.geometry
        self.phase+=dt*np.array([velocity[0]-.5*g.track_m*gyro,velocity[0]+.5*g.track_m*gyro])/g.wheel_radius_m
        ticks=np.rint(self.phase/(2*np.pi)*g.ticks_per_revolution+self.rng.normal(0,.2,2)).astype(np.int64)
        noisy=np.r_[gyro+self.gyro_bias+self.rng.normal(0,self.gyro_sigma),
                    acceleration+self.rng.normal(0,self.accel_sigma,2)]
        self.imu_integral+=dt*noisy
        packet=SensorPacket(stamp,self.sequence,int(ticks[0]),int(ticks[1]),
                            *[float(x) for x in noisy],*[float(x) for x in self.imu_integral])
        self.sequence+=1;self.last_stamp=stamp
        return packet


def stationary_gyro_calibration(packets,geometry=EncoderGeometry()):
    if len(packets)<100:
        raise ValueError("at least 100 stationary packets required")
    stamps=np.array([p.stamp for p in packets])
    gyro=np.array([p.gyro_z for p in packets])
    ticks=np.array([[p.left_ticks,p.right_ticks] for p in packets])
    if not np.isfinite(stamps).all() or not np.isfinite(gyro).all() or np.any(np.diff(stamps)<=0):
        raise ValueError("finite strictly ordered calibration data required")
    rates=np.diff(ticks,axis=0)/(np.diff(stamps)[:,None])*(2*np.pi*geometry.wheel_radius_m/geometry.ticks_per_revolution)
    if np.max(np.abs(rates))>.03 or np.std(gyro)>.02:
        raise ValueError("calibration data does not satisfy stationary criteria")
    median=float(np.median(gyro));mad=float(np.median(np.abs(gyro-median)))
    inliers=np.abs(gyro-median)<=max(.003,4.5*1.4826*mad)
    if np.count_nonzero(inliers)<.9*len(packets):
        raise ValueError("excessive calibration outliers")
    estimate=float(np.mean(gyro[inliers]));sigma=float(np.std(gyro[inliers],ddof=1))
    return dict(status="PASS_SIMULATED_CALIBRATION",gyro_z_bias_rad_s=estimate,
                gyro_z_noise_std_rad_s=sigma,standard_error_rad_s=sigma/np.sqrt(np.count_nonzero(inliers)),
                samples=len(packets),inliers=int(np.count_nonzero(inliers)),
                provenance="MuJoCo IMU plus injected simulated noise; not a hardware calibration")
