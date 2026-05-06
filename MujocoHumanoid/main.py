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
# GAINS
# =========================================

KP_POSTURE = 120
KD_POSTURE = 15

KP_BALANCE = 120
KD_BALANCE = 70

# =========================================
# STANDING TARGETS
# =========================================

targets = {

    # slight forward lean
    LEFT_HIP: -0.12,
    RIGHT_HIP: -0.12,

    # stronger leg extension
    LEFT_KNEE: 0.20,
    RIGHT_KNEE: 0.20,

    # ankle compensation
    LEFT_ANKLE: -0.08,
    RIGHT_ANKLE: -0.08,
}

# =========================================
# MAIN LOOP
# =========================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        q = data.qpos
        qd = data.qvel

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
        # JOINT POSTURE CONTROL
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
        # ANKLE BALANCE
        # =====================================

        data.ctrl[LEFT_ANKLE] += 0.6 * balance_torque
        data.ctrl[RIGHT_ANKLE] += 0.6 * balance_torque

        # =====================================
        # HIP RECOVERY
        # =====================================

        data.ctrl[LEFT_HIP] -= 0.2 * balance_torque
        data.ctrl[RIGHT_HIP] -= 0.2 * balance_torque

        # =====================================
        # KNEE SUPPORT FORCE
        # =====================================

        knee_support = 25

        data.ctrl[LEFT_KNEE] += knee_support
        data.ctrl[RIGHT_KNEE] += knee_support

        # =====================================
        # STEP SIMULATION
        # =====================================

        mujoco.mj_step(model, data)

        viewer.sync()