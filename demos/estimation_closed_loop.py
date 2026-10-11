"""Actual MuJoCo motion controlled only by an encoder/IMU estimate."""
import argparse
from dataclasses import asdict,replace
import json
from pathlib import Path
from time import perf_counter_ns
import mujoco
import numpy as np
from embodied_swarm.hal.physical_mujoco_adapter import PhysicalMujocoAdapter
from embodied_swarm.core.robot_command import RobotCommand
from embodied_swarm.core.safety import SafetySupervisor
from embodied_swarm.control.pure_pursuit import pure_pursuit_control
from embodied_swarm.estimation.ekf import EncoderImuEKF
from embodied_swarm.estimation.simulation_sensors import sensor_model_file,SimulatedSensors,stationary_gyro_calibration
from embodied_swarm.estimation.identification import identify_velocity_step


def strict_sensor_event(event):
    """Preserve malformed sensor provenance in strict, reversible JSON."""
    nonfinite={}
    def clean(value,path=""):
        if isinstance(value,dict):
            return {key:clean(item,f"{path}.{key}" if path else key) for key,item in value.items()}
        if isinstance(value,list):
            return [clean(item,f"{path}[{i}]") for i,item in enumerate(value)]
        if isinstance(value,float) and not np.isfinite(value):
            nonfinite[path]="NaN" if np.isnan(value) else "+Infinity" if value>0 else "-Infinity"
            return None
        return value
    encoded=clean(event)
    if nonfinite:
        encoded["nonfinite_fields"]=nonfinite
    return json.dumps(encoded,allow_nan=False)


def run(seed=7,scenario="nominal",output_root="artifacts/estimation"):
    if scenario not in ("nominal","delay","dropout","anomalies","outage"):
        raise ValueError("unknown sensor transport scenario")
    out=Path(output_root);out.mkdir(parents=True,exist_ok=True)
    sim=PhysicalMujocoAdapter(sensor_model_file(out/"physical_base_sensors.xml"))
    actuator_scale=float(np.random.default_rng(seed+500).uniform(.8,1.2))
    sim.model.actuator_gainprm[:3,0]*=actuator_scale
    sim.model.actuator_biasprm[:3,2]=-sim.model.actuator_gainprm[:3,0]
    step_sensors=SimulatedSensors(seed+50)
    last=step_sensors.sample(sim.model,sim.data);step_records=[];step_packets=[]
    for _ in range(200):
        sim.send_command(RobotCommand(timestamp=sim.data.time,base_twist=[.3,0.,0.]))
        sim.step(.01)
        sample=step_sensors.sample(sim.model,sim.data)
        g=step_sensors.geometry
        vx=(sample.left_ticks+sample.right_ticks-last.left_ticks-last.right_ticks)*.5*(2*np.pi*g.wheel_radius_m/g.ticks_per_revolution)/(sample.stamp-last.stamp)
        step_records.append([sample.stamp,vx]);step_packets.append(asdict(sample));last=sample
    identification=identify_velocity_step(step_records,.3)
    sim.reset()
    sensors=SimulatedSensors(seed)
    calibration_packets=[]
    for _ in range(200):
        sim.send_command(RobotCommand(timestamp=sim.data.time))
        sim.step(.01)
        calibration_packets.append(sensors.sample(sim.model,sim.data))
    calibration=stationary_gyro_calibration(calibration_packets)
    estimate=EncoderImuEKF(gyro_bias=calibration["gyro_z_bias_rad_s"],lateral_time_constant_s=identification["time_constant_s"])
    estimate.ingest(calibration_packets[-1],sim.data.time)
    begin=float(sim.data.time)
    safety=SafetySupervisor()
    xs=np.linspace(0,2.8,120)
    path=np.column_stack([xs,-.4*np.sin(np.pi*xs/2.8)])
    queue=[];records=[];raw=[];times=[];contacts=0;dropped=0;stop_time=None;goal_stop=None
    rng=np.random.default_rng(seed+1000)
    for step in range(1800):
        now=float(sim.data.time)
        # Controller receives only an estimator snapshot and sensor freshness.
        state=estimate.state(now)
        if estimate.measurement_age(now)>.15:
            safety.emergency_stop("estimator_sensor_timeout")
            if stop_time is None:
                stop_time=now
        v,w=pure_pursuit_control(tuple(state.base_pose),path.tolist(),.25,.3,.08)
        if v==0. and w==0. and goal_stop is None:
            goal_stop=now
        if goal_stop is not None:
            v,w=0.,0.
        command=safety.filter(state,RobotCommand(timestamp=now,base_twist=[v,0.,w]),now)
        if safety.latched_reason and stop_time is None:
            stop_time=now
        sim.send_command(RobotCommand(timestamp=command.timestamp,base_twist=command.base_twist))
        sim.step(.01)  # Returned ground truth state is deliberately ignored.
        now=float(sim.data.time)
        packet=sensors.sample(sim.model,sim.data)
        if scenario=="anomalies" and step in (120,220):
            packet=replace(packet,stamp=now+1.)
        if scenario=="anomalies" and step==320:
            packet=replace(packet,left_ticks=packet.left_ticks+5000)
        if scenario=="anomalies" and step==420:
            packet=replace(packet,gyro_z=float("nan"))
        loss=(scenario=="dropout" and rng.random()<.2) or (scenario=="outage" and 2.<now-begin<2.5)
        delay=max(0.,.05+rng.uniform(-.01,.01)) if scenario=="delay" else 0.
        raw.append(dict(event="sensor_generated",receipt=now,packet=asdict(packet),dropped=loss,delay_s=delay))
        if loss:
            dropped+=1
        else:
            queue.append((now+delay,packet))
        queue.sort(key=lambda item:item[0])
        before=perf_counter_ns()
        while queue and queue[0][0]<=now+1e-9:
            delivery,received=queue.pop(0)
            accepted,reason=estimate.ingest(received,now)
            raw.append(dict(event="sensor_received",receipt=now,stamp=received.stamp,sequence=received.sequence,
                            accepted=accepted,reason=reason))
            if scenario=="anomalies" and step==520:
                accepted,reason=estimate.ingest(received,now)
                raw.append(dict(event="duplicate_received",receipt=now,accepted=accepted,reason=reason))
        predicted=estimate.state(now)
        times.append((perf_counter_ns()-before)/1000)
        # Only the evidence writer/evaluator reads engine poses, never control.
        truth=sim.data.qpos[:3].copy()
        ground_velocity=sim.data.qvel[:3].copy()
        obstacle=sim.model.geom("obstacle").id
        contact=any(obstacle in (sim.data.contact[c].geom1,sim.data.contact[c].geom2) for c in range(sim.data.ncon))
        contacts+=int(contact)
        records.append(dict(time=now,estimate=predicted.base_pose.tolist(),estimated_twist=predicted.base_twist.tolist(),
                            truth=truth.tolist(),truth_world_velocity=ground_velocity.tolist(),
                            gyro_bias=estimate.x[5],measurement_age_s=estimate.measurement_age(now),
                            command=command.base_twist.tolist(),safety=safety.latched_reason,obstacle_contact=contact))
        if safety.latched_reason and now-stop_time>.6:
            break
        if goal_stop is not None and now-goal_stop>.6:
            break
    poses=np.array([r["truth"] for r in records]);est=np.array([r["estimate"] for r in records])
    position_rmse=float(np.sqrt(np.mean(np.sum((poses[:,:2]-est[:,:2])**2,axis=1))))
    yaw_error=np.arctan2(np.sin(poses[:,2]-est[:,2]),np.cos(poses[:,2]-est[:,2]))
    final_error=float(np.linalg.norm(poses[-1,:2]-path[-1]))
    a,b=path[:-1],path[1:];cte=[]
    for pose in poses:
        fraction=np.clip(np.sum((pose[:2]-a)*(b-a),axis=1)/np.maximum(1e-12,np.sum((b-a)**2,axis=1)),0,1)
        cte.append(np.min(np.linalg.norm(pose[:2]-(a+fraction[:,None]*(b-a)),axis=1)))
    cte_rmse=float(np.sqrt(np.mean(np.square(cte))))
    stopped_speed=float(np.linalg.norm(ground_velocity[:2]))
    navigation_pass=final_error<.085 and cte_rmse<.08 and contacts==0 and stopped_speed<.01
    fault_pass=scenario=="outage" and safety.latched_reason=="estimator_sensor_timeout" and stopped_speed<.01 and contacts==0
    report=dict(status="PASS_REAL_SIM" if (fault_pass if scenario=="outage" else navigation_pass and position_rmse<.06) else "FAIL",
                navigation_status="PASS_REAL_SIM" if navigation_pass else "ABORTED" if safety.latched_reason else "FAIL",
                seed=seed,scenario=scenario,mujoco=mujoco.__version__,controller_feedback="EncoderImuEKF only; truth offline evaluation",
                sensor_provenance="MuJoCo gyro/accelerometer/velocimeter with simulated noise, virtual encoder count quantization",
                steps=len(records),position_estimation_rmse_m=position_rmse,yaw_estimation_rmse_rad=float(np.sqrt(np.mean(yaw_error**2))),
                path_tracking_rmse_m=cte_rmse,final_goal_error_m=final_error,final_speed_m_s=stopped_speed,
                obstacle_contact_steps=contacts,packets_accepted=estimate.accepted,packets_rejected=estimate.rejected,dropped_packets=dropped,
                safety_latched_reason=safety.latched_reason,gyro_bias_final_rad_s=float(estimate.x[5]),calibration=calibration,
                actuator_identification=identification,
                plant_actuator_gain_scale=actuator_scale,
                simulation_time_s=sim.data.time-begin,
                estimator_runtime_us=dict(zip(["p50","p95","p99"],np.percentile(times,[50,95,99]))),
                acceptance=dict(navigation_goal_m=.085,tracking_rmse_m=.08,estimation_position_rmse_m=.06,outage_stop_speed_m_s=.01))
    stem=out/f"{scenario}_seed_{seed}"
    stem.with_suffix(".json").write_text(json.dumps(report,indent=2,allow_nan=False))
    stem.with_suffix(".jsonl").write_text("".join(json.dumps(r,allow_nan=False)+"\n" for r in records))
    # Only serialization changes; the EKF still receives and rejects NaN.
    rawtext="".join(strict_sensor_event(r)+"\n" for r in raw)
    (out/f"{scenario}_seed_{seed}_sensors.jsonl").write_text(rawtext)
    (out/f"{scenario}_seed_{seed}_calibration.jsonl").write_text("".join(json.dumps(asdict(p))+"\n" for p in calibration_packets))
    (out/f"{scenario}_seed_{seed}_step_response.jsonl").write_text("".join(json.dumps(dict(time=t,encoder_speed_m_s=v,packet=p))+"\n" for (t,v),p in zip(step_records,step_packets)))
    sim.shutdown();print(json.dumps(report,indent=2,allow_nan=False));return report


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--seed",type=int,default=7)
    parser.add_argument("--scenario",choices=["nominal","delay","dropout","anomalies","outage"],default="nominal")
    args=parser.parse_args();r=run(args.seed,args.scenario);raise SystemExit(0 if r["status"]=="PASS_REAL_SIM" else 1)
