import numpy as np
import mujoco

# BALANCE GAINS

Kp_balance = 120
Kd_balance = 180

# JOINT POSTURE GAINS

Kp_joint = 400
Kd_joint = 60

# DESIRED STANDING POSE

DESIRED_HIP = 0.01
DESIRED_KNEE = 0.03
DESIRED_ANKLE = 0.05

DESIRED_HIP_ROLL = 0.0

# GET TORSO ORIENTATION

def get_torso_angles(data):

    quat = data.qpos[3:7]

    # convert quaternion to angular representation
    euler = np.zeros(3)

    mujoco.mju_quat2Vel(
        euler,
        quat,
        1.0
    )

    roll = euler[0]
    pitch = euler[1]

    return roll, pitch

# GENERIC PD CONTROLLER

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

# MAIN BALANCE CONTROLLER

def compute_control(data, joints):

    q = data.qpos
    qd = data.qvel

    ctrl = {}

    # TORSO BALANCE

    roll, pitch = get_torso_angles(data)

    torso_roll_vel = qd[3]
    torso_pitch_vel = qd[4]

    balance_roll = (
        -Kp_balance * roll
        -Kd_balance * torso_roll_vel
    )

    balance_pitch = (
        Kp_balance * pitch
        + Kd_balance * torso_pitch_vel
    )

    # HIP ROLL CONTROL

    for joint in [

        "LEFT_HIP_ROLL",
        "RIGHT_HIP_ROLL"
    ]:

        jid = joints[joint]

        pos = q[jid + 7]
        vel = qd[jid + 6]

        ctrl[jid] = pd_control(
            pos,
            vel,
            DESIRED_HIP_ROLL,
            Kp_joint,
            Kd_joint
        )

    # side balance correction

    ctrl[joints["LEFT_HIP_ROLL"]] += balance_roll
    ctrl[joints["RIGHT_HIP_ROLL"]] -= balance_roll

    # HIP PITCH CONTROL

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

            + balance_pitch
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

            + 140
        )

    # ANKLE CONTROL

    for joint in [

        "LEFT_ANKLE",
        "RIGHT_ANKLE"
    ]:

        jid = joints[joint]

        pos = q[jid + 7]
        vel = qd[jid + 6]

        ctrl[jid] = (

            pd_control(
                pos,
                vel,
                DESIRED_ANKLE,
                Kp_joint,
                Kd_joint
            )

            + 0.35 * balance_pitch
        )


    # MICRO STEP RECOVERY

    STEP_TRIGGER = 0.08

    if pitch < -STEP_TRIGGER:

        # move right leg slightly backward

        ctrl[joints["RIGHT_HIP"]] += 25
        ctrl[joints["RIGHT_KNEE"]] -= 10

    elif pitch > STEP_TRIGGER:

        # move left leg slightly forward

        ctrl[joints["LEFT_HIP"]] += 25
        ctrl[joints["LEFT_KNEE"]] -= 10

    # HIP RECOVERY STRATEGY

    ctrl[joints["LEFT_HIP"]] -= 0.03 * balance_pitch
    ctrl[joints["RIGHT_HIP"]] -= 0.03 * balance_pitch

    if abs(pitch) > 0.35:

        for jid in ctrl:
            ctrl[jid] = 0

    return ctrl