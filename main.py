import numpy as np
import matplotlib.pyplot as plt

from config import L1, L2
from kinematics import inverse_kinematics
from animation import draw_robot, move_robot, setup_robot_scene


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

setup_robot_scene(ax)
draw_robot(
    ax,
    L1,
    L2,
    current_theta1,
    current_theta2,
    0,
    -(L1 + L2)
)

plt.show()