import numpy as np
import pytest
from embodied_swarm.control.residual_contact_env import ResidualContactEnv


@pytest.mark.parametrize("action",[-100.,0.,100.])
def test_physical_residual_bounds_and_finite_state(action):
    env=ResidualContactEnv(7)
    obs=env.reset()
    before=env.sim.data.time
    new,reward,done,info=env.step(action)
    assert obs.shape==new.shape==(3,) and np.isfinite(new).all()
    assert np.isfinite(reward) and abs(info["residual_m"])<=.03
    assert env.sim.data.time>before
    with pytest.raises(ValueError):
        env.step(np.nan)


def test_residual_invalid_stiffness():
    with pytest.raises(ValueError):
        ResidualContactEnv(stiffness=np.inf)
