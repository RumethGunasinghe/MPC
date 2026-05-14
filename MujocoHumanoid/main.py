import mujoco
import mujoco.viewer

from Controller.PID import compute_control

# LOAD MODEL

model = mujoco.MjModel.from_xml_path(
    "h1Model/mjcf/scene.xml"
)

data = mujoco.MjData(model)

# INITIAL STANDING POSE

# LEFT LEG

data.qpos[10] = 0.25     # left hip pitch
data.qpos[11] = 0.55     # left knee
data.qpos[12] = -0.30    # left ankle

# RIGHT LEG

data.qpos[15] = 0.25     # right hip pitch
data.qpos[16] = 0.55     # right knee
data.qpos[17] = -0.30    # right ankle



# ACTIVE CONTROL JOINTS

# ONLY sagittal joints remain active

joints = {

    "LEFT_HIP": 3,
    "LEFT_KNEE": 4,
    "LEFT_ANKLE": 5,

    "RIGHT_HIP": 8,
    "RIGHT_KNEE": 9,
    "RIGHT_ANKLE": 10,
}

# ARM ACTUATORS

LEFT_ARM = [12, 13, 14, 15]
RIGHT_ARM = [16, 17, 18]


# MAIN LOOP

with mujoco.viewer.launch_passive(
    model,
    data
) as viewer:

    while viewer.is_running():

        # RESET CONTROLS

        data.ctrl[:] = 0

        # LOCK UNUSED DOFs

        # LEFT HIP YAW
        data.ctrl[1] = -300 * data.qpos[8]

        # LEFT HIP ROLL
        data.ctrl[2] = -300 * data.qpos[9]

        # RIGHT HIP YAW
        data.ctrl[6] = -300 * data.qpos[13]

        # RIGHT HIP ROLL
        data.ctrl[7] = -300 * data.qpos[14]

        # TORSO
        data.ctrl[11] = -200 * data.qpos[18]

        # LOCK ARMS
        for aid in LEFT_ARM:

            data.ctrl[aid] = 0

        for aid in RIGHT_ARM:

            data.ctrl[aid] = 0

        # COMPUTE BALANCE CONTROLLER

        ctrl = compute_control(
            data,
            joints
        )

        # APPLY ACTIVE CONTROLS

        for jid, torque in ctrl.items():

            if jid < model.nu:

                data.ctrl[jid] += torque

        # STEP PHYSICS

        mujoco.mj_step(
            model,
            data
        )

        viewer.sync()