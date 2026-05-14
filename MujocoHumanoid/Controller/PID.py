import numpy as np
from Controller.MPC import MPCController
from Controller.COM import compute_com
import mujoco

print("PID FILE LOADED") 

# BALANCE GAINS

Kp_balance = 120
Kd_balance = 180

# JOINT POSTURE GAINS

Kp_joint = 900
Kd_joint = 120

# DESIRED STANDING POSE

DESIRED_HIP = 0.18
DESIRED_KNEE = 0.40
DESIRED_ANKLE = -0.12

DESIRED_HIP_ROLL = 0.0


mpc = MPCController()

step_timer = 0
step_duration = 40
stepping = False
step_leg = None


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

    pitch = quat[1]
    roll = quat[0]

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

    # print("compute_control running")

    global step_timer
    global stepping
    global step_leg

    q = data.qpos
    qd = data.qvel

    ctrl = {}

    # CENTER OF MASS

    com = compute_com(data)

    com_x = com["x"]
    com_vx = com["vx"]


    # TORSO BALANCE

    roll, pitch = get_torso_angles(data)

    torso_roll_vel = qd[3]
    torso_pitch_vel = qd[4]

    balance_roll = (
        -Kp_balance * roll
        -Kd_balance * torso_roll_vel
    )

    mpc_balance = mpc.compute_balance(
    pitch,
    torso_pitch_vel
)

    desired_com = 0.0

    com_error = com_x - desired_com

    balance_pitch = (
        -250 * com_error
        -80 * com_vx
    )

    # # HIP ROLL CONTROL

    # for joint in [

    #     "LEFT_HIP_ROLL",
    #     "RIGHT_HIP_ROLL"
    # ]:

    #     jid = joints[joint]

    #     pos = q[jid + 7]
    #     vel = qd[jid + 6]

    #     ctrl[jid] = pd_control(
    #         pos,
    #         vel,
    #         DESIRED_HIP_ROLL,
    #         Kp_joint,
    #         Kd_joint
    #     )

    # # side balance correction

    # ctrl[joints["LEFT_HIP_ROLL"]] += balance_roll
    # ctrl[joints["RIGHT_HIP_ROLL"]] -= balance_roll

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

            - 0.3 * balance_pitch
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

            + 90
        )

    # ANKLE CONTROL

    for joint in [

        "LEFT_ANKLE",
        "RIGHT_ANKLE"
    ]:

        jid = joints[joint]

        pos = q[jid + 7]
        vel = qd[jid + 6]

        # ankle stabilization

        ankle_correction = (
            -120 * pitch
            -40 * torso_pitch_vel
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

    # MPC STEP PLANNING

    future_pitch = mpc.predict_fall(
        pitch,
        torso_pitch_vel
    )

    STEP_TRIGGER = 0.20

    # START STEP

    if not stepping:

        if future_pitch < -STEP_TRIGGER:

            stepping = True
            step_leg = "RIGHT"

            step_timer = 0

        elif future_pitch > STEP_TRIGGER:

            stepping = True
            step_leg = "LEFT"

            step_timer = 0

    # EXECUTE STEP

    if stepping:

        step_timer += 1

        phase = step_timer / step_duration

        # smooth swing trajectory
        hip_offset = 0.4 * np.sin(np.pi * phase)

        knee_offset = -0.5 * np.sin(np.pi * phase)

        if step_leg == "RIGHT":

            ctrl[joints["RIGHT_HIP"]] += hip_offset * 300
            ctrl[joints["RIGHT_KNEE"]] += knee_offset * 300

        elif step_leg == "LEFT":

            ctrl[joints["LEFT_HIP"]] -= hip_offset * 300
            ctrl[joints["LEFT_KNEE"]] -= knee_offset * 300

        # step finished
        if step_timer >= step_duration:

            stepping = False

    

    # FALL DETECTION

    if abs(pitch) > 0.35:

        for jid in ctrl:
            ctrl[jid] = 0

    return ctrl