import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -------------------------------
# Parameters
# -------------------------------
dt = 0.02
g = 9.81
length = 1.0

# PID gains
Kp = 20
Kd = 5

# Initial state
theta = 1
theta_dot = 0.0

# -------------------------------
# Setup plot
# -------------------------------
fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-1.5, 1.5)

cart_width = 0.4
cart_height = 0.2

cart = plt.Rectangle((-cart_width/2, -0.1), cart_width, cart_height, color='black')
ax.add_patch(cart)

line, = ax.plot([], [], lw=3)  # pendulum
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

# -------------------------------
# Animation update
# -------------------------------
def update(frame):
    global theta, theta_dot

    # PID control
    force = -Kp * theta - Kd * theta_dot

    # Dynamics
    theta_ddot = (g / length) * theta + force

    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    # Pendulum position
    x = length * np.sin(theta)
    y = length * np.cos(theta)

    line.set_data([0, x], [0, y])
    time_text.set_text(f'theta={theta:.2f}')

    return line, time_text

# -------------------------------
# Run animation
# -------------------------------
ani = FuncAnimation(fig, update, frames=500, interval=20)
plt.title("Inverted Pendulum (PID Control)")
plt.show()