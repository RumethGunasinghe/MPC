import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.02              # time step
time = 10              # total simulation time
steps = int(time / dt)

# Pendulum parameters
g = 9.81               # gravity
length = 1.0           # length of pendulum

# PID Controller gains
Kp = 20
Kd = 5

# Initial state
theta = 0.2            # initial angle (radians)
theta_dot = 0.0        # angular velocity

# Store results
theta_list = []


# Simulation loop
for i in range(steps):

    #PID control 
    force = -Kp * theta - Kd * theta_dot

    #  Dynamics (simplified) 
    theta_ddot = (g / length) * theta + force

    #  Update state 
    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    # Store data
    theta_list.append(theta)

# Plot results
time_axis = np.linspace(0, time, steps)

plt.plot(time_axis, theta_list)
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Inverted Pendulum with PID Control")
plt.grid()
plt.show()