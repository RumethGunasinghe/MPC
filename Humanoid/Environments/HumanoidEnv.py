import pybullet as p
import pybullet_data
import time

class HumanoidEnv:
    def __init__(self):

        # 🔥 Clean connection
        if p.isConnected():
            p.disconnect()

        self.client = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Ground
        self.plane = p.loadURDF("plane.urdf")

        start_pos = [0, 0, 1.2]

        start_orientation = p.getQuaternionFromEuler([1.57, 0, 0])

        self.robot = p.loadURDF(
            "humanoid/humanoid.urdf",
            start_pos,
            start_orientation
        )

        # ✅ number of joints
        self.num_joints = p.getNumJoints(self.robot)

        # disable default motors
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                self.robot,
                i,
                controlMode=p.VELOCITY_CONTROL,
                force=0
            )

        # correct joints (from  output)
        self.hip_joints = [9, 12]
        self.knee_joints = [10, 13]
        self.ankle_joints = [11, 14]

        # initial pose (VERY IMPORTANT)
        p.resetJointState(self.robot, 10, 0.2)
        p.resetJointState(self.robot, 13, 0.2)

        p.resetJointState(self.robot, 11, -0.1)
        p.resetJointState(self.robot, 14, -0.1)

        self.dt = 1/240

    # CORRECT state (torso, not joints)
    def get_state(self):
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot)
        angle = p.getEulerFromQuaternion(base_orn)[1]

        base_vel, base_ang_vel = p.getBaseVelocity(self.robot)

        angular_vel = base_ang_vel[1]
        linear_vel = base_vel[0]   # 🔥 forward/back motion

        return angle, angular_vel, linear_vel

    def apply_torque(self, joints, torques):
        for i, joint in enumerate(joints):
            p.setJointMotorControl2(
                self.robot,
                joint,
                controlMode=p.TORQUE_CONTROL,
                force=torques[i]
            )

    def step(self):
        p.stepSimulation()

        # damping (stability fix)
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot)

        p.resetBaseVelocity(
            self.robot,
            linearVelocity=[v * 0.98 for v in base_vel],
            angularVelocity=[v * 0.98 for v in base_ang_vel]
        )

        # p.applyExternalForce(
        #     self.robot,
        #     -1,
        #     forceObj=[0, 0, -5],
        #     posObj=[0, 0, 0],
        #     flags=p.WORLD_FRAME
        # )

        time.sleep(self.dt)