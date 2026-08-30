import numpy as np
import matplotlib.pyplot as plt



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

    # Check joint limits
    if not (THETA1_MIN <= theta1 <= THETA1_MAX):
        print(
            "Target rejected: Theta 1 is outside "
            "the joint limits.\n"
        )
        return None

    if not (THETA2_MIN <= theta2 <= THETA2_MAX):
        print(
            "Target rejected: Theta 2 is outside "
            "the joint limits.\n"
        )
        return None

    return theta1, theta2


# ==========================================
# Animation
# ==========================================
# ==========================================
# Interactive Robot
# ==========================================

def robot_position(L1, L2, theta1, theta2):

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

    return x0, y0, x1, y1, x2, y2


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

        # Gradually move theta1
        theta1 = (
            start_theta1
            + t * (target_theta1 - start_theta1)
        )

        # Gradually move theta2
        theta2 = (
            start_theta2
            + t * (target_theta2 - start_theta2)
        )

        # Forward kinematics
        x0, y0, x1, y1, x2, y2 = robot_position(
            L1,
            L2,
            theta1,
            theta2
        )

        # Clear previous frame
        ax.clear()

        # Draw robot
        ax.plot(
            [x0, x1, x2],
            [y0, y1, y2],
            marker="o",
            linewidth=3
        )

        # Draw target
        ax.plot(
            target_x,
            target_y,
            marker="x",
            markersize=12,
            markeredgewidth=3
        )

        # Information
        ax.text(
            0.02,
            0.97,
            f"Target: ({target_x:.2f}, {target_y:.2f})\n"
            f"Current: ({x2:.2f}, {y2:.2f})\n"
            f"Theta 1: {np.degrees(theta1):.2f}°\n"
            f"Theta 2: {np.degrees(theta2):.2f}°",
            transform=ax.transAxes,
            verticalalignment="top"
        )

        # Graph settings
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_aspect("equal")
        ax.grid()

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("2-Link Robot - Click to Move")

        plt.pause(0.03)

    return target_theta1, target_theta2


# ==========================================
# Robot
# ==========================================

L1 = 5
L2 = 4

# ==========================================
# Joint Limits
# ==========================================

THETA1_MIN = np.radians(-90)
THETA1_MAX = np.radians(90)

THETA2_MIN = np.radians(0)
THETA2_MAX = np.radians(180)
# ==========================================
# Starting joint angles
# ==========================================

current_theta1 = np.radians(0)
current_theta2 = np.radians(0)


# ==========================================
# Create graph
# ==========================================

fig, ax = plt.subplots()


# ==========================================
# Mouse click function
# ==========================================

def on_click(event):

    global current_theta1
    global current_theta2

    # Ignore clicks outside the graph
    if event.inaxes != ax:
        return

    # Get clicked X and Y
    target_x = event.xdata
    target_y = event.ydata

    print("\n-------------------------")
    print("New target selected:")
    print(f"X = {target_x:.2f}")
    print(f"Y = {target_y:.2f}")

    # --------------------------------------
    # Calculate angles using IK
    # --------------------------------------

    result = inverse_kinematics(
        L1,
        L2,
        target_x,
        target_y
    )

    # Target unreachable
    if result is None:
        return

    target_theta1, target_theta2 = result

    print("\nCalculated joint angles:")
    print(
        f"Theta 1 = "
        f"{np.degrees(target_theta1):.2f} degrees"
    )

    print(
        f"Theta 2 = "
        f"{np.degrees(target_theta2):.2f} degrees"
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
# Connect mouse click to function
# ==========================================

fig.canvas.mpl_connect(
    "button_press_event",
    on_click
)


# ==========================================
# Draw initial robot
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
ax.set_title("2-Link Robot - Click to Move")

plt.show()