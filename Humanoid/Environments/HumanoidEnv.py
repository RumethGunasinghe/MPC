import pybullet as p
import pybullet_data
import time

class HumanoidEnv:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0, 0, -9.81)

        self.plane = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("humanoid/humanoid.urdf", [0,0,1.2])

        # straighten legs
        p.resetJointState(self.robot, 10, 0.2)   # right knee
        p.resetJointState(self.robot, 13, 0.2)   # left knee

        p.resetJointState(self.robot, 11, -0.1)  # right ankle
        p.resetJointState(self.robot, 14, -0.1)  # left ankle

        self.dt = 1/240

        self.num_joints = p.getNumJoints(self.robot)

        # disable default motors
        for i in range(self.num_joints):
            p.setJointMotorControl2(self.robot, i,
                controlMode=p.VELOCITY_CONTROL,
                force=0)

        self.hip_joints = [9, 12]
        self.knee_joints = [10, 13]
        self.ankle_joints = [11, 14]    

    def get_state(self):
        joint_indices = self.hip_joints

        states = p.getJointStates(self.robot, joint_indices)

        pos = [s[0] for s in states]
        vel = [s[1] for s in states]

        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot)
        torso_angle = p.getEulerFromQuaternion(base_orn)[1]

        return pos, vel, torso_angle

    def apply_torque(self, joints, torques):
        for i, joint in enumerate(joints):
            p.setJointMotorControl2(
                self.robot,
                joint,
                p.TORQUE_CONTROL,
                force=torques[i]
            )

    def step(self):
        p.stepSimulation()
        time.sleep(self.dt)