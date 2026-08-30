# import numpy as np
# import matplotlib.pyplot as plt
#
# # Robot link lengths
# L1 = 5
# L2 = 4
#
# # Joint angles
# theta1 = np.radians(30)
# theta2 = np.radians(45)
#
# # Base position
# x0 = 0
# y0 = 0
#
# # Position of joint 2
# x1 = L1 * np.cos(theta1)
# y1 = L1 * np.sin(theta1)
#
# # Position of end effector
# x2 = x1 + L2 * np.cos(theta1 + theta2)
# y2 = y1 + L2 * np.sin(theta1 + theta2)
#
# # Draw the robot
# plt.plot([x0, x1, x2], [y0, y1, y2], marker='o')
#
# plt.xlim(-10, 10)
# plt.ylim(-10, 10)
# plt.grid()
#
# plt.xlabel("X")
# plt.ylabel("Y")
# plt.title("2-Link Robot")
#
# plt.show()

import numpy as np
import matplotlib.pyplot as plt

plt.ion()


def inverse_kinematics(L1, L2, target_x, target_y):

    # -------------------------
    # Distance from base to target
    # -------------------------

    r_squared = target_x ** 2 + target_y ** 2
    distance = np.sqrt(r_squared)

    # -------------------------
    # Check whether target is reachable
    # -------------------------

    if distance > L1 + L2:
        print("Target is unreachable: too far away.\n")
        return None

    if distance < abs(L1 - L2):
        print("Target is unreachable: too close to the base.\n")
        return None

    # -------------------------
    # Calculate theta2
    # -------------------------

    cos_theta2 = (
        r_squared - L1 ** 2 - L2 ** 2
    ) / (2 * L1 * L2)

    cos_theta2 = np.clip(cos_theta2, -1, 1)

    theta2 = np.arccos(cos_theta2)

    # -------------------------
    # Calculate theta1
    # -------------------------

    theta1 = (
        np.arctan2(target_y, target_x)
        -
        np.arctan2(
            L2 * np.sin(theta2),
            L1 + L2 * np.cos(theta2)
        )
    )

    # -------------------------
    # Display result
    # -------------------------

    print("Target position:")
    print(f"X = {target_x}")
    print(f"Y = {target_y}")

    print("\nCalculated joint angles:")
    print(f"Theta 1 = {np.degrees(theta1):.2f} degrees")
    print(f"Theta 2 = {np.degrees(theta2):.2f} degrees")

    return theta1, theta2


def animate_arm(
        L1,
        L2,
        start_theta1,
        start_theta2,
        target_theta1,
        target_theta2,
        target_x,
        target_y
):

    # -------------------------
    # Create the figure
    # -------------------------

    fig, ax = plt.subplots()

    frames = 100

    # -------------------------
    # Animate
    # -------------------------

    for i in range(frames + 1):

        t = i / frames

        # Gradually change the angles
        theta1 = (
            start_theta1
            + t * (target_theta1 - start_theta1)
        )

        theta2 = (
            start_theta2
            + t * (target_theta2 - start_theta2)
        )

        # -------------------------
        # Forward Kinematics
        # -------------------------

        x0 = 0
        y0 = 0

        x1 = L1 * np.cos(theta1)
        y1 = L1 * np.sin(theta1)

        x2 = (
            x1
            + L2 * np.cos(theta1 + theta2)
        )

        y2 = (
            y1
            + L2 * np.sin(theta1 + theta2)
        )

        # -------------------------
        # Convert angles to degrees
        # -------------------------

        theta1_deg = np.degrees(theta1)
        theta2_deg = np.degrees(theta2)

        # -------------------------
        # Clear previous frame
        # -------------------------

        ax.clear()

        # -------------------------
        # Draw robot
        # -------------------------

        ax.plot(
            [x0, x1, x2],
            [y0, y1, y2],
            marker="o",
            linewidth=3
        )

        # -------------------------
        # Draw target
        # -------------------------

        ax.plot(
            target_x,
            target_y,
            marker="x",
            markersize=12,
            markeredgewidth=3
        )

        # -------------------------
        # Display information
        # -------------------------

        ax.text(
            0.02,
            0.97,
            f"Target: ({target_x:.2f}, {target_y:.2f})\n"
            f"Current: ({x2:.2f}, {y2:.2f})\n"
            f"Theta 1: {theta1_deg:.2f}°\n"
            f"Theta 2: {theta2_deg:.2f}°",
            transform=ax.transAxes,
            verticalalignment="top",
            fontsize=11
        )

        # -------------------------
        # Graph settings
        # -------------------------

        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)

        ax.set_aspect("equal")

        ax.grid()

        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        ax.set_title(
            "2-Link Robot - Inverse Kinematics"
        )

        # -------------------------
        # Display frame
        # -------------------------

        plt.pause(0.03)

    plt.show()


# =========================================
# Robot dimensions
# =========================================

L1 = 5
L2 = 4


# =========================================
# Desired target position
# =========================================

target_x = 6
target_y = 5


# =========================================
# Starting robot position
# =========================================

start_theta1 = np.radians(0)
start_theta2 = np.radians(0)


# =========================================
# Inverse Kinematics
# =========================================

result = inverse_kinematics(
    L1,
    L2,
    target_x,
    target_y
)


# =========================================
# If target is reachable
# =========================================

if result is not None:

    target_theta1, target_theta2 = result

    # -------------------------
    # Start animation
    # -------------------------

    animate_arm(
        L1,
        L2,
        start_theta1,
        start_theta2,
        target_theta1,
        target_theta2,
        target_x,
        target_y
    )