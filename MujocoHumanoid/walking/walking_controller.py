from walking.foot_trajectory import FootTrajectory
from walking.inverse_kinematics import leg_ik

traj = FootTrajectory()


def get_leg_targets(phase, progress):

    

    x, z = traj.generate(progress)

    swing_hip, swing_knee, swing_ankle = leg_ik(
        x,
        -0.75 + z
    )

    print(
        f"{phase}",
        f"hip={swing_hip:.2f}",
        f"knee={swing_knee:.2f}",
        f"ankle={swing_ankle:.2f}"
    )

    print(
        phase,
        f"x={x:.3f}",
        f"z={z:.3f}"
    )

    stance_hip = 0.28
    stance_knee = -0.79
    stance_ankle = 0.52

    if phase == "LEFT_SUPPORT":

        return {

            "LEFT_HIP": stance_hip,
            "LEFT_KNEE": stance_knee,
            "LEFT_ANKLE": stance_ankle,

            "RIGHT_HIP": swing_hip,
            "RIGHT_KNEE": swing_knee,
            "RIGHT_ANKLE": swing_ankle,
        }

    else:

        return {

            "RIGHT_HIP": stance_hip,
            "RIGHT_KNEE": stance_knee,
            "RIGHT_ANKLE": stance_ankle,

            "LEFT_HIP": swing_hip,
            "LEFT_KNEE": swing_knee,
            "LEFT_ANKLE": swing_ankle,
        }

    
    