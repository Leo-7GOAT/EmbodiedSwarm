from dataclasses import replace
import json
import numpy as np
import pytest
from embodied_swarm.estimation.ekf import EncoderImuEKF,SensorPacket,EncoderGeometry,transition
from embodied_swarm.estimation.simulation_sensors import stationary_gyro_calibration
from embodied_swarm.estimation.identification import identify_velocity_step


def packet(t,sequence,v=.3,w=.2,bias=.025):
    g=EncoderGeometry();unit=2*np.pi*g.wheel_radius_m/g.ticks_per_revolution
    return SensorPacket(t,sequence,round((v-.5*g.track_m*w)*t/unit),round((v+.5*g.track_m*w)*t/unit),
                        w+bias,0.,w*v,(w+bias)*t,0.,w*v*t)


def test_transition_constant_velocity_and_circular_motion():
    x=np.array([0.,0.,0.,.3,0.,.025])
    out=transition(x,.025,[0,0],.1)
    assert np.allclose(out[:2],[.03,0]) and np.array_equal(x,[0,0,0,.3,0,.025])
    out=transition(x,.225,[0,.06],.1)
    assert abs(out[2]-.02)<1e-12 and np.allclose(out[3:5],[.3,0])


def test_ekf_bias_covariance_and_numeric_circle():
    e=EncoderImuEKF(gyro_bias=.025,lateral_time_constant_s=0.)
    e.ingest(packet(0,0),0.)
    for i in range(1,201):
        assert e.ingest(packet(i*.01,i),i*.01)[0]
        assert np.min(np.linalg.eigvalsh(e.P))>=-1e-12
        assert np.max(np.abs(e.P-e.P.T))<1e-12
    state=e.state(2.)
    expected=np.array([1.5*np.sin(.4),1.5*(1-np.cos(.4))])
    assert np.linalg.norm(state.base_pose[:2]-expected)<.006
    assert abs(state.base_pose[2]-.4)<.001
    assert abs(e.x[5]-.025)<.001


def test_delay_extrapolation_is_causal_and_does_not_mutate_filter():
    e=EncoderImuEKF(gyro_bias=.025,lateral_time_constant_s=0.)
    e.ingest(packet(0.,0,w=0.),.05)
    for i in range(1,51):
        e.ingest(packet(i*.01,i,w=0.),i*.01+.05)
    before=e.x.copy();cov=e.P.copy();last=e.last_packet
    current=e.state(.55)
    assert current.timestamp==.55 and current.base_pose[0]>e.x[0]
    assert np.array_equal(before,e.x) and np.array_equal(cov,e.P) and e.last_packet==last
    assert abs(e.measurement_age(.55)-.05)<1e-12


@pytest.mark.parametrize("bad,now,reason",[
    (replace(packet(.02,2),stamp=1.),.02,"future_timestamp"),
    (packet(0.,0),.02,"duplicate_or_out_of_order"),
    (replace(packet(.02,2),gyro_z=np.nan),.02,"nonfinite"),
    (replace(packet(.02,2),gyro_z=100.),.02,"sensor_outlier"),
    (replace(packet(.02,2),left_ticks=900000),.02,"sensor_outlier"),
    (replace(packet(.02,2),gyro_integral=100.),.02,"sensor_outlier"),
    (replace(packet(.02,2),sequence=-1),.02,"invalid_encoding"),
    (replace(packet(.02,2),sequence=True),.02,"invalid_encoding"),
    (replace(packet(.02,2),left_ticks=10**1000),.02,"invalid_encoding"),
    (packet(.01,1),1.,"stale_packet"),
])
def test_bad_packets_leave_estimator_state_unchanged(bad,now,reason):
    e=EncoderImuEKF();e.ingest(packet(0.,0),0.)
    before,cov=e.x.copy(),e.P.copy()
    ok,result=e.ingest(bad,now)
    assert not ok and result==reason
    assert np.array_equal(before,e.x) and np.array_equal(cov,e.P)
    assert e.last_packet.stamp==0.


def test_sensor_integrals_cover_transport_loss():
    complete=EncoderImuEKF(gyro_bias=.025,lateral_time_constant_s=0.)
    sparse=EncoderImuEKF(gyro_bias=.025,lateral_time_constant_s=0.)
    for i in range(101):
        p=packet(i*.01,i,w=0.)
        complete.ingest(p,p.stamp)
        if i%5==0:
            sparse.ingest(p,p.stamp)
    assert abs(complete.x[2]-sparse.x[2])<1e-3
    assert np.linalg.norm(complete.x[:2]-sparse.x[:2])<.008


def test_gyro_calibration_bias_and_moving_data_rejection():
    packets=[packet(.01*(i+1),i,v=0,w=0) for i in range(200)]
    result=stationary_gyro_calibration(packets)
    assert abs(result["gyro_z_bias_rad_s"]-.025)<1e-12
    with pytest.raises(ValueError):
        stationary_gyro_calibration(packets[:10])
    with pytest.raises(ValueError):
        stationary_gyro_calibration([packet(.01*(i+1),i,v=.3,w=0) for i in range(200)])
    with pytest.raises(ValueError):
        stationary_gyro_calibration([replace(p,stamp=0.) for p in packets])


def test_response_identification_numeric_and_failure_paths():
    t=np.arange(1,201)*.01
    result=identify_velocity_step(np.column_stack([t,.299*(1-np.exp(-t/.1))]),.3)
    assert abs(result["time_constant_s"]-.1)<1e-6 and abs(result["gain"]-.299/.3)<1e-6
    for samples in (np.column_stack([t,np.zeros(200)]),np.column_stack([t,np.full(200,.3)]),np.column_stack([t,np.full(200,np.nan)])):
        with pytest.raises(ValueError):
            identify_velocity_step(samples,.3)
    with pytest.raises(ValueError):
        identify_velocity_step(np.column_stack([t[::-1],np.zeros(200)]),.3)


def test_configuration_and_uninitialized_queries_reject_invalid_inputs():
    for kwargs in ({"wheel_radius_m":np.nan},{"track_m":0.},{"ticks_per_revolution":True}):
        with pytest.raises(ValueError):
            EncoderGeometry(**kwargs)
    for kwargs in ({"gyro_bias":np.inf},{"max_age":0.},{"lateral_time_constant_s":-1.}):
        with pytest.raises(ValueError):
            EncoderImuEKF(**kwargs)
    e=EncoderImuEKF()
    with pytest.raises(ValueError):
        e.state(0.)
    e.ingest(packet(1.,1),1.)
    for query in (np.nan,.9):
        with pytest.raises(ValueError):
            e.state(query)
    from demos.estimation_closed_loop import run
    with pytest.raises(ValueError):
        run(scenario="unrecognized")


@pytest.mark.parametrize("scenario",["nominal","delay","dropout","anomalies","outage"])
def test_actual_physics_controller_uses_estimation_and_handles_faults(scenario,tmp_path):
    from demos.estimation_closed_loop import run
    report=run(7,scenario,tmp_path)
    assert report["status"]=="PASS_REAL_SIM"
    assert report["actuator_identification"]["residual_rmse_m_s"]<.01
    if scenario=="outage":
        assert report["navigation_status"]=="ABORTED" and report["final_speed_m_s"]<.01
    else:
        assert report["final_goal_error_m"]<.085 and report["position_estimation_rmse_m"]<.06
    if scenario=="anomalies":
        assert set(report["packets_rejected"])=={"future_timestamp","sensor_outlier","nonfinite","duplicate_or_out_of_order"}
        def reject_nonfinite_constant(value):
            raise AssertionError(f"nonstandard JSON numeric token: {value}")
        events=[json.loads(line,parse_constant=reject_nonfinite_constant) for line in (tmp_path/"anomalies_seed_7_sensors.jsonl").read_text().splitlines()]
        malformed=[e for e in events if "nonfinite_fields" in e]
        assert len(malformed)==1 and malformed[0]["packet"]["gyro_z"] is None
        assert malformed[0]["nonfinite_fields"]=={"packet.gyro_z":"NaN"}
        json.loads((tmp_path/"anomalies_seed_7.json").read_text(),parse_constant=reject_nonfinite_constant)
