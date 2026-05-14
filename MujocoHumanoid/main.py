import mujoco
import mujoco.viewer

from Controller.PID import compute_control

# LOAD MODEL

model = mujoco.MjModel.from_xml_path(
    "h1Model/mjcf/scene.xml"
)

data = mujoco.MjData(model)

startup_steps = 0

# INITIAL H1 STANDING POSE
# NVIDIA / ISAAC STYLE

# LEFT LEG

data.qpos[10] = 0.28
data.qpos[11] = -0.79
data.qpos[12] = 0.52

data.qpos[15] = 0.28
data.qpos[16] = -0.79
data.qpos[17] = 0.52

# ACTIVE JOINTS

joints = {

    "LEFT_HIP": 3,
    "LEFT_KNEE": 4,
    "LEFT_ANKLE": 5,

    "RIGHT_HIP": 8,
    "RIGHT_KNEE": 9,
    "RIGHT_ANKLE": 10,
}

# MAIN LOOP

with mujoco.viewer.launch_passive(
    model,
    data
) as viewer:

    while viewer.is_running():

        data.ctrl[:] = 0

        # LOCK UNUSED DOFs

        # LEFT HIP YAW
        data.ctrl[1] = -150 * data.qpos[8]

        # LEFT HIP ROLL
        data.ctrl[2] = -150 * data.qpos[9]

        # RIGHT HIP YAW
        data.ctrl[6] = -150 * data.qpos[13]

        # RIGHT HIP ROLL
        data.ctrl[7] = -150 * data.qpos[14]

        # TORSO
        data.ctrl[11] = -80 * data.qpos[18]

        # FREEZE ARMS

        for aid in [12, 13, 14, 15]:

            data.ctrl[aid] = 0

        for aid in [16, 17, 18]:

            data.ctrl[aid] = 0

        # STARTUP STEPS
        startup_steps += 1

        # BALANCE CONTROLLER

        ctrl = compute_control(
            data,
            joints
        )

        # APPLY CONTROLS

        for jid, torque in ctrl.items():

            if jid < model.nu:

                data.ctrl[jid] += torque

        # STEP PHYSICS

        mujoco.mj_step(
            model,
            data
        )

        viewer.sync()