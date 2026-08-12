import mujoco
import mujoco.viewer
import numpy as np
import time

# LOAD MODEL
model = mujoco.MjModel.from_xml_path(
    "h1Model/mjcf/scene.xml"
)
data = mujoco.MjData(model)

# ---------------------------------------------------
# JOINT IDS
# ---------------------------------------------------
JOINTS = {
    "LEFT_HIP": 3,
    "LEFT_KNEE": 4,
    "LEFT_ANKLE": 5,
    "RIGHT_HIP": 8,
    "RIGHT_KNEE": 9,
    "RIGHT_ANKLE": 10,
}

# ---------------------------------------------------
# HELPER FUNCTION
# ---------------------------------------------------
def set_joint_position(actuator_id, target, kp=250):
    qpos_index = actuator_id + 7
    current = data.qpos[qpos_index]
    error = target - current
    data.ctrl[actuator_id] = kp * error

# ---------------------------------------------------
# INITIAL CROUCHED POSE
# ---------------------------------------------------
def crouch_pose():
    set_joint_position(JOINTS["LEFT_HIP"], 0.28)
    set_joint_position(JOINTS["LEFT_KNEE"], -0.79)
    set_joint_position(JOINTS["LEFT_ANKLE"], 0.52)
    set_joint_position(JOINTS["RIGHT_HIP"], 0.28)
    set_joint_position(JOINTS["RIGHT_KNEE"], -0.79)
    set_joint_position(JOINTS["RIGHT_ANKLE"], 0.52)

# ---------------------------------------------------
# WAVE MOTION
# ---------------------------------------------------
def sway_motion(t):
    sway = 0.08 * np.sin(2 * t)
    set_joint_position(
        JOINTS["LEFT_HIP"],
        0.28 + sway
    )
    set_joint_position(
        JOINTS["RIGHT_HIP"],
        0.28 - sway
    )

# ---------------------------------------------------
# SIMPLE STEP MOTION
# ---------------------------------------------------
def stepping_motion(t):
    step = 0.25 * np.sin(3 * t)
    knee = -0.79 + 0.2 * np.sin(3 * t)
    set_joint_position(
        JOINTS["LEFT_HIP"],
        0.28 + step
    )
    set_joint_position(
        JOINTS["LEFT_KNEE"],
        knee
    )

# ---------------------------------------------------
# MAIN LOOP
# ---------------------------------------------------
start_time = time.time()
with mujoco.viewer.launch_passive(
    model,
    data
) as viewer:
    while viewer.is_running():
        t = time.time() - start_time
        data.ctrl[:] = 0

        # LOCK UNUSED JOINTS
        data.ctrl[1] = -150 * data.qpos[8]
        data.ctrl[2] = -150 * data.qpos[9]
        data.ctrl[6] = -150 * data.qpos[13]
        data.ctrl[7] = -150 * data.qpos[14]
        data.ctrl[11] = (
            -150 * data.qpos[18]
            - 20 * data.qvel[17]
        )

        # FREEZE ARMS
        for aid in [12, 13, 14, 15, 16, 17, 18]:
            if aid < model.nu:
                data.ctrl[aid] = 0

        # ---------------------------------------------------
        # DEMO MOTIONS
        # ---------------------------------------------------
        crouch_pose()
        sway_motion(t)
        stepping_motion(t)
        # ---------------------------------------------------

        mujoco.mj_step(
            model,
            data
        )
        viewer.sync()
