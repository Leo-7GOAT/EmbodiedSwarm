import time

import mujoco
import mujoco.viewer


XML = """
<mujoco model="embodied_swarm_smoke">

    <worldbody>

        <light pos="0 0 3"/>

        <geom
            name="floor"
            type="plane"
            size="5 5 0.1"
            rgba="0.8 0.8 0.8 1"
        />

        <body
            name="test_body"
            pos="0 0 0.5"
        >

            <freejoint/>

            <geom
                type="box"
                size="0.2 0.2 0.2"
                rgba="0.2 0.4 0.8 1"
            />

        </body>

    </worldbody>

</mujoco>
"""


model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)


with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        mujoco.mj_step(model, data)

        viewer.sync()

        time.sleep(model.opt.timestep)