from Environments.HumanoidEnv import HumanoidEnv
from controllers.pid import PIDHumanoid
from controllers.mpc import MPCHumanoid
import matplotlib.pyplot as plt
import pybullet as p
from utils.logger import GraphLogger

# from controllers.mpc import MPCHumanoid   # switch later

env = HumanoidEnv()
logger = GraphLogger()

#  start with PID
controller = MPC()



#For printing data

time_data = []

hip_angle = []
hip_vel = []
hip_torque = []

knee_angle = []
knee_vel = []
knee_torque = []

ankle_angle = []
ankle_vel = []
ankle_torque = []



for step in range(2000):

    angle, ang_vel, lin_vel, com_error = env.get_state()

    torque = controller.compute(angle, ang_vel, lin_vel, com_error)

    env.apply_torque(env.ankle_joints, [torque, torque])
    env.apply_torque(env.knee_joints, [-torque * 0.5, -torque * 0.5])
    env.apply_torque(env.hip_joints, [torque * 0.7, torque * 0.7])

    # read right side joints
    hip = p.getJointState(env.robot, 9)
    knee = p.getJointState(env.robot, 10)
    ankle = p.getJointState(env.robot, 11)

    logger.log(step * env.dt, hip, knee, ankle, torque)

    env.step()

logger.plot()