import numpy as np
import mujoco

from Controller.MPC import MPCController
from Controller.COM import compute_com

print("PID FILE LOADED")

# BALANCE GAINS
Kp_joint = 220
Kd_joint = 60

mpc = MPCController()


def get_torso_angles(data):

    quat = data.qpos[3:7]

    roll = quat[0]
    pitch = quat[1]

    return roll, pitch


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


def compute_walking_control(
    data,
    joints,
    targets
):

    q = data.qpos
    qd = data.qvel

    ctrl = {}

    roll, pitch = get_torso_angles(data)

    torso_pitch_vel = qd[4]

    com = compute_com(data)

    com_x = com["x"]
    com_vx = com["vx"]

    mpc_balance = mpc.compute_balance(

        pitch,
        torso_pitch_vel

    )

    balance_pitch = (

        -10 * com_x
        -5 * com_vx
        + 0.05 * mpc_balance

    )

    # --------------------------
    # HIPS
    # --------------------------

    for joint in [

        "LEFT_HIP",
        "RIGHT_HIP"

    ]:

        print(
            targets["LEFT_HIP"],
            targets["RIGHT_HIP"]
        )

        jid = joints[joint]

        pos = q[jid + 7]
        vel = qd[jid + 6]

        ctrl[jid] = (

            pd_control(

                pos,
                vel,
                targets[joint],
                Kp_joint,
                Kd_joint

            )

            - 0.03 * balance_pitch

        )

    # --------------------------
    # KNEES
    # --------------------------

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
                targets[joint],
                Kp_joint,
                Kd_joint

            )

            + 140

        )

    # --------------------------
    # ANKLES
    # --------------------------

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
                targets[joint],
                Kp_joint,
                Kd_joint

            )

            + ankle_correction
            - 25 * vel

        )

    # --------------------------
    # FALL DETECTION
    # --------------------------

    if abs(pitch) > 0.8:

        for jid in ctrl:

            ctrl[jid] = 0

    return ctrl