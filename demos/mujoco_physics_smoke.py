import mujoco


XML = """
<mujoco model="physics_smoke">
    <option timestep="0.002" gravity="0 0 -9.81"/>

    <worldbody>
        <geom
            name="floor"
            type="plane"
            size="5 5 0.1"
        />

        <body
            name="box"
            pos="0 0 1"
        >
            <freejoint/>

            <geom
                type="box"
                size="0.2 0.2 0.2"
                mass="1"
            />
        </body>
    </worldbody>
</mujoco>
"""


model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

print("Initial z:", data.qpos[2])

for _ in range(500):
    mujoco.mj_step(model, data)

print("Final z:", data.qpos[2])
print("Simulation time:", data.time)