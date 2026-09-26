import mujoco
import mujoco.viewer

from Controller.PID import compute_walking_control, get_torso_angles
from walking.walking_gait import WalkingGait
from walking.foot_trajectory import FootTrajectory
from walking.inverse_kinematics import leg_ik
from walking.walking_controller import get_leg_targets


# LOAD MODEL

model = mujoco.MjModel.from_xml_path(
    "h1Model/mjcf/scene.xml"
)

data = mujoco.MjData(model)
gait = WalkingGait()
foot_traj = FootTrajectory()

# INITIAL H1 CROUCHED POSE

def set_initial_pose(data):

    # LEFT LEG

    data.qpos[10] = 0.28
    data.qpos[11] = -0.79
    data.qpos[12] = 0.52

    # RIGHT LEG

    data.qpos[15] = 0.28
    data.qpos[16] = -0.79
    data.qpos[17] = 0.52

# apply startup pose

set_initial_pose(data)

# ACTIVE JOINTS

joints = {

    "LEFT_HIP": 3,
    "LEFT_KNEE": 4,
    "LEFT_ANKLE": 5,

    "RIGHT_HIP": 8,
    "RIGHT_KNEE": 9,
    "RIGHT_ANKLE": 10,
}

# STARTUP TIMER
startup_steps = 0

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

        # TORSO STABILIZATION

        TORSO_TARGET = 0.0

        torso_pos = data.qpos[18]
        torso_vel = data.qvel[17]

        if startup_steps < 300:

            # strong startup torso stabilization

            data.ctrl[11] = (

                -250 * data.qpos[18]
                -40 * data.qvel[17]
            )

        else:

            # softer continuous torso stabilization

            data.ctrl[11] = (

                -220 * (torso_pos - TORSO_TARGET)
                -45 * torso_vel
            )

        # FREEZE ARMS

        for aid in [12, 13, 14, 15]:

            data.ctrl[aid] = 0

        for aid in [16, 17, 18]:

            data.ctrl[aid] = 0

        phase = gait.update(

            model.opt.timestep

        )

        progress = gait.get_phase_progress()


        targets = get_leg_targets(
            phase,
            progress
        )
        # STARTUP STEPS
        startup_steps += 1

        # BALANCE CONTROLLER

        ctrl = compute_walking_control(
            data,
            joints,
            targets
        )
        # APPLY CONTROLS

        for jid, torque in ctrl.items():

            if jid < model.nu:

                data.ctrl[jid] += torque

        # STEP PHYSICS
        if data.time < 0.05:
            set_initial_pose(data)

        mujoco.mj_step(
            model,
            data
        )

        roll, pitch = get_torso_angles(data)

        com_x = data.qpos[0]

        print(
            f"t={data.time:.2f} | "
            f"pitch={pitch:.3f} | "
            f"com_x={com_x:.3f}"
        )

        viewer.sync()