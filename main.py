import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from InversePendulum import CartPole
from PID import PIDController
from MPC import MPCController

# Toggle controller
USE_MPC = False

env = CartPole(theta=0.2)

if USE_MPC:
    controller = MPCController()
    title = "MPC Control"
else:
    controller = PIDController()
    title = "PID Control"

# Plot
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-1.5, 1.5)

cart_w, cart_h = 0.4, 0.2
cart = plt.Rectangle((0, -0.1), cart_w, cart_h, color='black')
ax.add_patch(cart)

pole, = ax.plot([], [], lw=3)
text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

# Update loop
def update(frame):
    x, x_dot, theta, theta_dot = env.get_state()
    force = controller.compute(x, x_dot, theta, theta_dot)

    env.step(force)

    # Draw cart
    cart.set_xy((x - cart_w/2, -0.1))

    # Draw pole
    px = x + env.l * np.sin(theta)
    py = env.l * np.cos(theta)

    pole.set_data([x, px], [0, py])
    text.set_text(f"x={x:.2f}, theta={theta:.2f}")

    return pole, cart, text

ani = FuncAnimation(fig, update, frames=1000, interval=10)

plt.title(title)
plt.grid()
plt.show()