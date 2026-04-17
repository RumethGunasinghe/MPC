import numpy as np
import matplotlib.pyplot as plt

# Parameters

dt = 0.02
time = 10
steps = int(time / dt)

g = 9.81
length = 1.0

# MPC settings
horizon = 10
force_candidates = np.linspace(-10, 10, 21)

# Initial state
theta = 0.2
theta_dot = 0.0

theta_list = []

# Simulation loop
for step in range(steps):

    best_force = 0
    best_cost = float("inf")

    # Try different forces
    for force in force_candidates:

        temp_theta = theta
        temp_theta_dot = theta_dot
        cost = 0

        # Predict future
        for t in range(horizon):

            theta_ddot = (g / length) * temp_theta + force

            temp_theta_dot += theta_ddot * dt
            temp_theta += temp_theta_dot * dt

            # Cost: want theta close to 0
            cost += temp_theta**2 + 0.1 * temp_theta_dot**2

        # Choose best force
        if cost < best_cost:
            best_cost = cost
            best_force = force

    # Apply best force
    theta_ddot = (g / length) * theta + best_force
    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    theta_list.append(theta)

# Plot
time_axis = np.linspace(0, time, steps)

plt.plot(time_axis, theta_list)
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Inverted Pendulum with MPC")
plt.grid()
plt.show()