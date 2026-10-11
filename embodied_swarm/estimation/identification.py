"""Identify a simulated velocity-servo response from virtual encoder counts."""
import numpy as np
from scipy.optimize import least_squares


def identify_velocity_step(samples,commanded_speed):
    values=np.asarray(samples,dtype=float)
    if values.ndim!=2 or values.shape[1]!=2 or len(values)<100 or not np.isfinite(values).all() or not np.isfinite(commanded_speed) or commanded_speed<=0:
        raise ValueError("at least 100 finite time/encoder-speed samples required")
    t,v=values.T
    if t[0]<=0 or np.any(np.diff(t)<=0) or t[-1]<1.:
        raise ValueError("ordered positive time and >= 1 s observation required")
    if np.mean(v[-30:])<.2*commanded_speed:
        raise ValueError("insufficient actuator response")
    steady=float(np.mean(v[-30:]))
    if np.count_nonzero((v>.1*steady)&(v<.9*steady))<3 or np.std(v[-30:])>.1*steady:
        raise ValueError("transient or stationary tail insufficient for identification")
    result=least_squares(lambda p:p[0]*(1-np.exp(-t/p[1]))-v,
                         [commanded_speed,.1],bounds=([.01,.002],[2*commanded_speed,2.]))
    residual=v-result.x[0]*(1-np.exp(-t/result.x[1]))
    if not result.success or result.x[1]>.25*t[-1]:
        raise ValueError("response window insufficient or identification failed")
    if np.sqrt(np.mean(residual**2))>.01:
        raise ValueError("response is inconsistent with the first-order model")
    return dict(status="PASS_SIMULATED_IDENTIFICATION",steady_speed_m_s=float(result.x[0]),
                commanded_speed_m_s=commanded_speed,gain=float(result.x[0]/commanded_speed),
                time_constant_s=float(result.x[1]),residual_rmse_m_s=float(np.sqrt(np.mean(residual**2))),
                samples=len(t),duration_s=float(t[-1]),
                provenance="Actual MuJoCo force actuator response, virtual encoder counts; no physical hardware identified")
