import pybullet as p
import pybullet_data
import time


import pybullet as p
import pybullet_data
import time

# Connect GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane
plane = p.loadURDF("plane.urdf")

# Load humanoid HIGHER (important)
humanoid = p.loadURDF("humanoid/humanoid.urdf", [0, 0, 2.0])

# Physics
p.setGravity(0, 0, -9.8)
p.setTimeStep(1/240)

# Damping (strong)
for i in range(p.getNumJoints(humanoid)):
    p.changeDynamics(humanoid, i, linearDamping=0.2, angularDamping=0.2)

# 🔒 DISABLE ALL MOTORS 
for i in range(p.getNumJoints(humanoid)):
    p.setJointMotorControl2(
        humanoid,
        i,
        p.VELOCITY_CONTROL,
        force=0
    )

# Run simulation
while True:
    p.stepSimulation()
    
    # Control RIGHT KNEE ONLY (joint 10)
    p.setJointMotorControl2(humanoid, 10, p.POSITION_CONTROL, targetPosition=-0.2, force=20)
    # LEFT knee (13)
    p.setJointMotorControl2(humanoid, 13, p.POSITION_CONTROL, -0.2, force=20)   

    time.sleep(1/240)