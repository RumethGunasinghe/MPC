from Environments.HumanoidEnv import HumanoidEnv
from controllers.pid import PIDHumanoid
import pybullet as p


env = HumanoidEnv()

for i in range(env.num_joints):
    print(i, p.getJointInfo(env.robot, i)[1])

    
controller = PIDHumanoid()

while True:
    pos, vel, torso_angle = env.get_state()

    torque = 200 * torso_angle + 40 * vel[0]

    env.apply_torque(env.hip_joints, [torque, torque])

    env.step()