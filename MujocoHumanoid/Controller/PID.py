import numpy as np

# BALANCE GAINS

Kp_balance = 300
Kd_balance = 80

# JOINT POSTURE GAINS

Kp_joint = 200
Kd_joint = 25

# DESIRED STANDING POSE

DESIRED_HIP = 0.0
DESIRED_KNEE = 0.2
DESIRED_ANKLE = -0.1
DESIRED_HIP_ROLL = 0.0


# TORSO ORIENTATION

def get_torso_angles(data):

    quat = data.qpos[3:7]

    # simple approximations
    roll = quat[0]
    pitch = quat[1]

    return roll, pitch


# GENERIC PD CONTROLLER

def pd_control(pos, vel, desired, kp, kd):

    return -kp * (pos - desired) - kd * vel


# MAIN CONTROL

def compute_control(data, joints):

    q = data.qpos
    qd = data.qvel


    # TORSO BALANCE


    roll, pitch = get_torso_angles(data)

    torso_roll_vel = qd[3]
    torso_pitch_vel = qd[4]

    balance_roll = -Kp_balance * roll - Kd_balance * torso_roll_vel
    balance_pitch = -Kp_balance * pitch - Kd_balance * torso_pitch_vel

    ctrl = {}


    # HIP ROLL (SIDE BALANCE)


    for joint in ["LEFT_HIP_ROLL", "RIGHT_HIP_ROLL"]:

        jid = joints[joint]

        ctrl[jid] = (
            pd_control(
                q[jid],
                qd[jid],
                DESIRED_HIP_ROLL,
                Kp_joint,
                Kd_joint
            )
        )

    # apply roll stabilization
    ctrl[joints["LEFT_HIP_ROLL"]] += balance_roll
    ctrl[joints["RIGHT_HIP_ROLL"]] -= balance_roll


    # HIP PITCH


    for joint in ["LEFT_HIP", "RIGHT_HIP"]:

        jid = joints[joint]

        ctrl[jid] = (
            pd_control(
                q[jid],
                qd[jid],
                DESIRED_HIP,
                Kp_joint,
                Kd_joint
            )
            + balance_pitch
        )


    # KNEES


    for joint in ["LEFT_KNEE", "RIGHT_KNEE"]:

        jid = joints[joint]

        ctrl[jid] = pd_control(
            q[jid],
            qd[jid],
            DESIRED_KNEE,
            Kp_joint,
            Kd_joint
        )


    # ANKLES


    for joint in ["LEFT_ANKLE", "RIGHT_ANKLE"]:

        jid = joints[joint]

        ctrl[jid] = pd_control(
            q[jid],
            qd[jid],
            DESIRED_ANKLE,
            Kp_joint,
            Kd_joint
        )

    return ctrl