import mujoco
import mujoco.viewer

from Controller.PID import compute_control

# LOAD MODEL

model = mujoco.MjModel.from_xml_path(
    "h1Model/mjcf/scene.xml"
)

data = mujoco.MjData(model)

# ACTUATOR IDS

joints = {

    "LEFT_HIP_ROLL": 1,
    "LEFT_HIP": 2,
    "LEFT_KNEE": 3,
    "LEFT_ANKLE": 4,

    "RIGHT_HIP_ROLL": 6,
    "RIGHT_HIP": 7,
    "RIGHT_KNEE": 8,
    "RIGHT_ANKLE": 9,
}

# MAIN LOOP

with mujoco.viewer.launch_passive(
    model,
    data
) as viewer:

    while viewer.is_running():

        ctrl = compute_control(
            data,
            joints
        )

        data.ctrl[:] = 0

        for jid, torque in ctrl.items():

            data.ctrl[jid] = torque

        mujoco.mj_step(
            model,
            data
        )

        viewer.sync()