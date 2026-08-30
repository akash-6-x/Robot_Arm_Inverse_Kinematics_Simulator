import numpy as np
import matplotlib.pyplot as plt

plt.ion()


# ==========================================
# Inverse Kinematics
# ==========================================

def inverse_kinematics(L1, L2, target_x, target_y):

    # Distance from base to target
    r_squared = target_x ** 2 + target_y ** 2
    distance = np.sqrt(r_squared)

    # Check whether target is reachable
    if distance > L1 + L2:
        print("Target is unreachable: too far away.\n")
        return None

    if distance < abs(L1 - L2):
        print("Target is unreachable: too close to the base.\n")
        return None

    # Calculate theta 2
    cos_theta2 = (
        r_squared - L1 ** 2 - L2 ** 2
    ) / (2 * L1 * L2)

    # Protect against tiny floating-point errors
    cos_theta2 = np.clip(cos_theta2, -1, 1)

    theta2 = np.arccos(cos_theta2)

    # Calculate theta 1
    theta1 = (
        np.arctan2(target_y, target_x)
        - np.arctan2(
            L2 * np.sin(theta2),
            L1 + L2 * np.cos(theta2)
        )
    )

    return theta1, theta2


# ==========================================
# Animation
# ==========================================

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

    fig, ax = plt.subplots()

    frames = 100

    for i in range(frames + 1):

        # Movement progress
        t = i / frames

        # Gradually move the joint angles
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
        # Clear previous frame
        # -------------------------

        ax.clear()

        # -------------------------
        # Draw robot
        # -------------------------

        ax.plot(
            [x0, x1, x2],
            [y0, y1, y2],
            marker="o"
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
        # Information
        # -------------------------

        ax.text(
            -9.5,
            6,
            f"Target: ({target_x:.2f}, {target_y:.2f})\n"
            f"Current: ({x2:.2f}, {y2:.2f})\n"
            f"Theta 1: {np.degrees(theta1):.2f}°\n"
            f"Theta 2: {np.degrees(theta2):.2f}°"
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

        # Show frame
        plt.pause(0.03)

    plt.ioff()
    plt.show()


# ==========================================
# Robot
# ==========================================

L1 = 5
L2 = 4


# ==========================================
# Target
# ==========================================

target_x = 6
target_y = 5


# ==========================================
# Current robot position
# ==========================================

start_theta1 = np.radians(0)
start_theta2 = np.radians(0)


# ==========================================
# Calculate target angles using IK
# ==========================================

result = inverse_kinematics(
    L1,
    L2,
    target_x,
    target_y
)


# ==========================================
# If target is reachable
# ==========================================

if result is not None:

    target_theta1, target_theta2 = result

    print("Target position:")
    print(f"X = {target_x}")
    print(f"Y = {target_y}")

    print("\nCalculated joint angles:")
    print(
        f"Theta 1 = "
        f"{np.degrees(target_theta1):.2f} degrees"
    )

    print(
        f"Theta 2 = "
        f"{np.degrees(target_theta2):.2f} degrees"
    )

    # Start animation
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