import mujoco
import mujoco.viewer
import numpy as np

# =========================================
# LOAD MODEL
# =========================================

model = mujoco.MjModel.from_xml_path(
    "h1Model/mjcf/scene.xml"
)

data = mujoco.MjData(model)

# =========================================
# ACTUATOR IDS
# =========================================

LEFT_HIP = 2
LEFT_KNEE = 3
LEFT_ANKLE = 4

RIGHT_HIP = 7
RIGHT_KNEE = 8
RIGHT_ANKLE = 9

# =========================================
# CONTROL GAINS
# =========================================

KP_POSTURE = 80
KD_POSTURE = 8

KP_BALANCE = 120
KD_BALANCE = 70

# =========================================
# STANDING POSTURE
# =========================================

targets = {

    # slight forward lean
    LEFT_HIP: -0.10,
    RIGHT_HIP: -0.10,

    # slight knee bend
    LEFT_KNEE: 0.25,
    RIGHT_KNEE: 0.25,

    # ankle compensation
    LEFT_ANKLE: 0.15,
    RIGHT_ANKLE: 0.15,
}

# =========================================
# MAIN LOOP
# =========================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        q = data.qpos
        qd = data.qvel

        # =====================================
        # RESET CONTROLS
        # =====================================

        data.ctrl[:] = 0

        # =====================================
        # TORSO BALANCE
        # =====================================

        quat = q[3:7]

        torso_pitch = quat[1]
        torso_pitch_vel = qd[4]

        balance_torque = (
            -KP_BALANCE * torso_pitch
            -KD_BALANCE * torso_pitch_vel
        )

        # =====================================
        # POSTURE CONTROL
        # =====================================

        for actuator_id, target in targets.items():

            pos = q[actuator_id + 7]
            vel = qd[actuator_id + 6]

            torque = (
                -KP_POSTURE * (pos - target)
                -KD_POSTURE * vel
            )

            data.ctrl[actuator_id] = torque

        # =====================================
        # ANKLE STRATEGY
        # =====================================

        data.ctrl[LEFT_ANKLE] += 0.4 * balance_torque
        data.ctrl[RIGHT_ANKLE] += 0.4 * balance_torque

        # =====================================
        # HIP STRATEGY
        # =====================================

        data.ctrl[LEFT_HIP] -= 0.2 * balance_torque
        data.ctrl[RIGHT_HIP] -= 0.2 * balance_torque

        # =====================================
        # STEP SIMULATION
        # =====================================

        mujoco.mj_step(model, data)

        viewer.sync()