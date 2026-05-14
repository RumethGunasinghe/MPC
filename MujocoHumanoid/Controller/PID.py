import numpy as np
import mujoco

from Controller.MPC import MPCController
from Controller.COM import compute_com

print("PID FILE LOADED")

# BALANCE GAINS

Kp_balance = 35
Kd_balance = 40

# JOINT GAINS

Kp_joint = 200
Kd_joint = 45

# H1 NOMINAL STANDING POSE

DESIRED_HIP = 0.18
DESIRED_KNEE = -0.79
DESIRED_ANKLE = 0.52

# MPC

mpc = MPCController()

# QUATERNION → APPROX ORIENTATION

def get_torso_angles(data):

    quat = data.qpos[3:7]

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

    # TORSO ORIENTATION

    roll, pitch = get_torso_angles(data)

    torso_pitch_vel = qd[4]

    # COM ESTIMATION

    com = compute_com(data)

    com_x = com["x"]
    com_vx = com["vx"]

    # MPC BALANCE

    mpc_balance = mpc.compute_balance(

        pitch,
        torso_pitch_vel
    )

    # COM BALANCE

    balance_pitch = (

        -40 * com_x
        -10 * com_vx
        + 0.15 * mpc_balance
    )

    # HIP CONTROL

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

            - 0.05 * balance_pitch
        )

    # KNEE CONTROL

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

            + + 140
        )

    # ANKLE CONTROL

    for joint in [

        "LEFT_ANKLE",
        "RIGHT_ANKLE"
    ]:

        jid = joints[joint]

        pos = q[jid + 7]
        vel = qd[jid + 6]

        ankle_correction = (

            -30 * pitch
            -10 * torso_pitch_vel
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
        )

    # FALL DETECTION

    if abs(pitch) > 0.7:

        for jid in ctrl:

            ctrl[jid] = 0

    return ctrl