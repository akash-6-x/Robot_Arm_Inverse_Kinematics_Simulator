import numpy as np
import matplotlib.pyplot as plt

from kinematics import robot_position


# ==========================================
# Move Robot
# ==========================================

def move_robot(
        ax,
        L1,
        L2,
        start_theta1,
        start_theta2,
        target_theta1,
        target_theta2,
        target_x,
        target_y
):

    frames = 100

    for i in range(frames + 1):

        t = i / frames

        # ----------------------------------
        # Gradually move theta1
        # ----------------------------------

        theta1 = (
            start_theta1
            + t * (target_theta1 - start_theta1)
        )

        # ----------------------------------
        # Gradually move theta2
        # ----------------------------------

        theta2 = (
            start_theta2
            + t * (target_theta2 - start_theta2)
        )

        # ----------------------------------
        # Forward Kinematics
        # ----------------------------------

        x0, y0, x1, y1, x2, y2 = robot_position(
            L1,
            L2,
            theta1,
            theta2
        )

        # ----------------------------------
        # Clear previous frame
        # ----------------------------------

        ax.clear()

        # ----------------------------------
        # Draw robot
        # ----------------------------------

        ax.plot(
            [x0, x1, x2],
            [y0, y1, y2],
            marker="o",
            linewidth=3
        )

        # ----------------------------------
        # Draw target
        # ----------------------------------

        ax.plot(
            target_x,
            target_y,
            marker="x",
            markersize=12,
            markeredgewidth=3
        )

        # ----------------------------------
        # Convert theta2 to degrees
        # ----------------------------------

        theta2_deg = np.degrees(theta2)

        # ----------------------------------
        # Servo representation
        # ----------------------------------

        if theta2_deg < 0:
            theta2_servo = 360 + theta2_deg
        else:
            theta2_servo = theta2_deg

        # ----------------------------------
        # Information
        # ----------------------------------

        ax.text(
            0.02,
            0.97,
            f"Target: ({target_x:.2f}, {target_y:.2f})\n"
            f"Current: ({x2:.2f}, {y2:.2f})\n"
            f"Theta 1: {np.degrees(theta1):.2f}°\n"
            f"Theta 2: {theta2_deg:.2f}° (natural)\n"
            f"Theta 2 servo: {theta2_servo:.2f}°",
            transform=ax.transAxes,
            verticalalignment="top"
        )

        # ----------------------------------
        # Graph settings
        # ----------------------------------

        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)

        ax.set_aspect("equal")
        ax.grid()

        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        ax.set_title(
            "2-Link Robot - Click to Move"
        )

        plt.pause(0.03)

    return target_theta1, target_theta2