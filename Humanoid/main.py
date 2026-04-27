from Environments.HumanoidEnv import HumanoidEnv
from controllers.pid import PIDHumanoid
# from controllers.mpc import MPCHumanoid   # switch later

env = HumanoidEnv()

# 🔥 start with PID
controller = PIDHumanoid()

while True:
    # get torso state
    angle, velocity = env.get_state()

    # compute control
    torque = controller.compute(angle, velocity)

    # apply to BOTH hips + ankles
    env.apply_torque(env.hip_joints, [torque * 0.4, torque * 0.4])
    env.apply_torque(env.knee_joints, [-torque * 0.3, -torque * 0.3])  # 🔥 NEW
    env.apply_torque(env.ankle_joints, [torque, torque])

    env.step()