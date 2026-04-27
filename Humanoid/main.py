from Environments.HumanoidEnv import HumanoidEnv
from controllers.pid import PIDHumanoid
# from controllers.mpc import MPCHumanoid   # switch later

env = HumanoidEnv()

#  start with PID
controller = PIDHumanoid()

while True:
    angle, ang_vel, lin_vel = env.get_state()

    torque = controller.compute(angle, ang_vel, lin_vel)

    # improved distribution
    env.apply_torque(env.ankle_joints, [torque, torque])            # primary balance
    env.apply_torque(env.knee_joints, [-torque * 0.5, -torque * 0.5])  # support
    env.apply_torque(env.hip_joints, [torque * 0.7, torque * 0.7])      # recovery
    env.step()