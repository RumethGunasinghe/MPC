import numpy as np
import mujoco

from Controller.MPC import MPCController
from Controller.COM import compute_com

print("PID FILE LOADED")

# BALANCE GAINS

Kp_balance = 25
Kd_balance = 55

# JOINT GAINS

Kp_joint = 220
Kd_joint = 60

# H1 CROUCHED STANCE

DESIRED_HIP = 0.28
DESIRED_KNEE = -0.79
DESIRED_ANKLE = 0.52

# MPC

mpc = MPCController()

# TORSO ORIENTATION

def get_torso_angles(data):

    quat = data.qpos[3:7]

    # approximate orientation

    roll = quat[0]
    pitch = quat[1]

    return roll, pitch

# PD CONTROLLER

def pd_control(

    pos,
    vel,
    desired,
    kp,
    kd
):

    return (

        -kp * (pos - desired)
        -kd * vel
    )

# MAIN CONTROLLER

def compute_control(data, joints):

    q = data.qpos
    qd = data.qvel

    ctrl = {}

    # =====================================
    # BODY ORIENTATION
    # =====================================

    roll, pitch = get_torso_angles(data)

    torso_pitch_vel = qd[4]

    # =====================================
    # COM ESTIMATION
    # =====================================

    com = compute_com(data)

    com_x = com["x"]
    com_vx = com["vx"]

    # =====================================
    # MPC BALANCE
    # =====================================

    mpc_balance = mpc.compute_balance(

        pitch,
        torso_pitch_vel
    )

    # =====================================
    # COM BALANCE
    # =====================================

    balance_pitch = (

        -55 * com_x
        -20 * com_vx
        + 0.05 * mpc_balance
    )

    # =====================================
    # HIP CONTROL
    # =====================================

    for joint in [

        "LEFT_HIP",
        "RIGHT_HIP"
    ]:

        jid = joints[joint]

        pos = q[jid + 7]
        vel = qd[jid + 6]

        ctrl[jid] = (

            pd_control(

                pos,
                vel,
                DESIRED_HIP,
                Kp_joint,
                Kd_joint
            )

            - 0.03 * balance_pitch
        )

    # =====================================
    # KNEE CONTROL
    # =====================================

    for joint in [

        "LEFT_KNEE",
        "RIGHT_KNEE"
    ]:

        jid = joints[joint]

        pos = q[jid + 7]
        vel = qd[jid + 6]

        ctrl[jid] = (

            pd_control(

                pos,
                vel,
                DESIRED_KNEE,
                Kp_joint,
                Kd_joint
            )

            + 140
        )

    # =====================================
    # ANKLE CONTROL
    # =====================================

    for joint in [

        "LEFT_ANKLE",
        "RIGHT_ANKLE"
    ]:

        jid = joints[joint]

        pos = q[jid + 7]
        vel = qd[jid + 6]

        ankle_correction = (

            -45 * pitch
            -18 * torso_pitch_vel
        )

        ctrl[jid] = (

            pd_control(

                pos,
                vel,
                DESIRED_ANKLE,
                Kp_joint,
                Kd_joint
            )

            + ankle_correction

            - 25 * vel
        )

    # =====================================
    # FALL DETECTION
    # =====================================

    if abs(pitch) > 0.8:

        for jid in ctrl:

            ctrl[jid] = 0

    return ctrl