import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from InversePendulum import CartPole
from PID import PIDController
from MPC import MPCController

# SETTINGS
USE_MPC = True   # Toggle here
dt = 0.01

# SETUP
env = CartPole(theta=0.2)

if USE_MPC:
    controller = MPCController()
    title = "MPC Control"
else:
    controller = PIDController()
    title = "PID Control"

# DATA STORAGE
theta_list = []
x_list = []
force_list = []
time_list = []

# PLOT SETUP (ANIMATION)
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-1.5, 1.5)

cart_w, cart_h = 0.4, 0.2
cart = plt.Rectangle((0, -0.1), cart_w, cart_h, color='black')
ax.add_patch(cart)

pole, = ax.plot([], [], lw=3)
text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

# UPDATE FUNCTION
def update(frame):
    global theta_list, x_list, force_list, time_list

    x, x_dot, theta, theta_dot = env.get_state()

    # Compute control
    force = controller.compute(x, x_dot, theta, theta_dot)

    # Store data BEFORE stepping
    theta_list.append(theta)
    x_list.append(x)
    force_list.append(force)
    time_list.append(frame * dt)

    # Step simulation
    env.step(force)

    # Draw cart
    cart.set_xy((x - cart_w/2, -0.1))

    # Draw pole
    px = x + env.l * np.sin(theta)
    py = env.l * np.cos(theta)
    pole.set_data([x, px], [0, py])

    text.set_text(f"x={x:.2f}, theta={theta:.2f}")

    return pole, cart, text

# 
# RUN ANIMATION
ani = FuncAnimation(fig, update, frames=1000, interval=10)

plt.title(title)
plt.grid()
plt.show()

# AFTER ANIMATION → PLOTS
plt.figure(figsize=(10, 6))

# Angle
plt.subplot(3, 1, 1)
plt.plot(time_list, theta_list)
plt.ylabel("Theta (rad)")
plt.title(f"{title} - System Response")

# Position
plt.subplot(3, 1, 2)
plt.plot(time_list, x_list)
plt.ylabel("Cart Position (m)")

# Force
plt.subplot(3, 1, 3)
plt.plot(time_list, force_list)
plt.ylabel("Force")
plt.xlabel("Time (s)")

plt.tight_layout()
plt.show()