import numpy as np
import matplotlib.pyplot as plt

from config import L1, L2
from kinematics import inverse_kinematics, robot_position
from animation import move_robot


# ==========================================
# Starting Joint Angles
# ==========================================

current_theta1 = np.radians(0)
current_theta2 = np.radians(0)


# ==========================================
# Create Graph
# ==========================================

fig, ax = plt.subplots()


# ==========================================
# Mouse Click Function
# ==========================================

def on_click(event):

    global current_theta1
    global current_theta2

    # --------------------------------------
    # Ignore clicks outside graph
    # --------------------------------------

    if event.inaxes != ax:
        return

    # --------------------------------------
    # Get clicked coordinates
    # --------------------------------------

    target_x = event.xdata
    target_y = event.ydata

    print("\n-------------------------")
    print("New target selected:")
    print(f"X = {target_x:.2f}")
    print(f"Y = {target_y:.2f}")

    # --------------------------------------
    # Calculate IK
    # --------------------------------------

    result = inverse_kinematics(
        L1,
        L2,
        target_x,
        target_y
    )

    # --------------------------------------
    # Target invalid / unreachable
    # --------------------------------------

    if result is None:
        return

    (
        target_theta1,
        target_theta2,
        target_theta2_servo
    ) = result

    # --------------------------------------
    # Display calculated angles
    # --------------------------------------

    print("\nCalculated joint angles:")

    print(
        f"Theta 1 = "
        f"{np.degrees(target_theta1):.2f} degrees"
    )

    print(
        f"Theta 2 = "
        f"{np.degrees(target_theta2):.2f} degrees "
        f"(natural)"
    )

    print(
        f"Theta 2 servo = "
        f"{target_theta2_servo:.2f} degrees"
    )

    # --------------------------------------
    # Move robot
    # --------------------------------------

    current_theta1, current_theta2 = move_robot(
        ax,
        L1,
        L2,
        current_theta1,
        current_theta2,
        target_theta1,
        target_theta2,
        target_x,
        target_y
    )

    print("Robot reached target.")


# ==========================================
# Connect Mouse
# ==========================================

fig.canvas.mpl_connect(
    "button_press_event",
    on_click
)


# ==========================================
# Draw Initial Robot
# ==========================================

x0, y0, x1, y1, x2, y2 = robot_position(
    L1,
    L2,
    current_theta1,
    current_theta2
)

ax.plot(
    [x0, x1, x2],
    [y0, y1, y2],
    marker="o",
    linewidth=3
)

ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)

ax.set_aspect("equal")
ax.grid()

ax.set_xlabel("X")
ax.set_ylabel("Y")

ax.set_title(
    "2-Link Robot - Click to Move"
)

plt.show()