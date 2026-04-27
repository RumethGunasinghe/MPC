import matplotlib.pyplot as plt

class GraphLogger:
    def __init__(self):
        self.time = []

        self.hip_angle = []
        self.knee_angle = []
        self.ankle_angle = []

        self.hip_vel = []
        self.knee_vel = []
        self.ankle_vel = []

        self.hip_torque = []
        self.knee_torque = []
        self.ankle_torque = []

    def log(self, t, hip, knee, ankle, torque):
        # joint states
        self.time.append(t)

        self.hip_angle.append(hip[0])
        self.knee_angle.append(knee[0])
        self.ankle_angle.append(ankle[0])

        self.hip_vel.append(hip[1])
        self.knee_vel.append(knee[1])
        self.ankle_vel.append(ankle[1])

        # torques
        self.hip_torque.append(torque * 0.7)
        self.knee_torque.append(-torque * 0.5)
        self.ankle_torque.append(torque)

    def plot(self):
        plt.figure(figsize=(14,10))

        # Angles
        plt.subplot(3,1,1)
        plt.plot(self.time, self.hip_angle, label="Hip")
        plt.plot(self.time, self.knee_angle, label="Knee")
        plt.plot(self.time, self.ankle_angle, label="Ankle")
        plt.title("Joint Angles")
        plt.ylabel("rad")
        plt.legend()

        # Velocities
        plt.subplot(3,1,2)
        plt.plot(self.time, self.hip_vel, label="Hip")
        plt.plot(self.time, self.knee_vel, label="Knee")
        plt.plot(self.time, self.ankle_vel, label="Ankle")
        plt.title("Joint Velocities")
        plt.ylabel("rad/s")
        plt.legend()

        # Torques
        plt.subplot(3,1,3)
        plt.plot(self.time, self.hip_torque, label="Hip")
        plt.plot(self.time, self.knee_torque, label="Knee")
        plt.plot(self.time, self.ankle_torque, label="Ankle")
        plt.title("Applied Torques")
        plt.ylabel("Nm")
        plt.xlabel("Time (s)")
        plt.legend()

        plt.tight_layout()
        plt.show()