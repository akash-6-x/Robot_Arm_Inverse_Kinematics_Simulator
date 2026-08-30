import numpy as np
import matplotlib.pyplot as plt


# ==========================================
# Inverse Kinematics
# ==========================================
def inverse_kinematics(L1, L2, target_x, target_y):

    # --------------------------------------
    # Distance from base to target
    # --------------------------------------

    r_squared = target_x ** 2 + target_y ** 2
    distance = np.sqrt(r_squared)

    # --------------------------------------
    # Check whether target is reachable
    # --------------------------------------

    if distance > L1 + L2:
        print("Target is unreachable: too far away.\n")
        return None

    if distance < abs(L1 - L2):
        print("Target is unreachable: too close to the base.\n")
        return None

    # --------------------------------------
    # Calculate the elbow angle magnitude.
    #
    # This gives the two possible elbow
    # configurations:
    #
    # +elbow_angle = anticlockwise elbow bend
    # -elbow_angle = clockwise elbow bend
    # --------------------------------------

    cos_elbow = (
        r_squared - L1 ** 2 - L2 ** 2
    ) / (2 * L1 * L2)

    cos_elbow = np.clip(cos_elbow, -1, 1)

    elbow_angle = np.arccos(cos_elbow)

    # --------------------------------------
    # Target direction
    # --------------------------------------

    target_angle = np.arctan2(
        target_y,
        target_x
    )

    # --------------------------------------
    # Generate BOTH mathematical solutions
    # --------------------------------------

    possible_solutions = []

    # ----------------------------------
    # Only use the natural elbow bend.
    #
    # theta2 must NEVER be negative.
    # Negative theta2 would make the elbow
    # bend backward / in the opposite direction.
    #
    # Therefore we intentionally reject the
    # negative elbow configuration.
    # ----------------------------------

    for signed_theta2 in [
        elbow_angle
    ]:

        # ----------------------------------
        # Mathematical angle of upper arm
        # ----------------------------------

        alpha1 = (
            target_angle
            - np.arctan2(
                L2 * np.sin(signed_theta2),
                L1 + L2 * np.cos(signed_theta2)
            )
        )

        # ----------------------------------
        # Convert upper-arm direction into
        # our robot theta1 convention:
        #
        # 0°   = DOWN
        # +     = ANTICLOCKWISE
        # -     = CLOCKWISE
        # ----------------------------------

        theta1 = alpha1 + np.pi / 2

        # ----------------------------------
        # Normalize theta1 into a useful
        # range around the robot's limits.
        # ----------------------------------

        theta1 = (theta1 + np.pi) % (2 * np.pi) - np.pi

        # ----------------------------------
        # Check shoulder limit
        # ----------------------------------

        if not (
            THETA1_MIN <= theta1 <= THETA1_MAX
        ):
            continue

        # ----------------------------------
        # theta2 is RELATIVE to the upper arm.
        #
        # + = elbow rotates ANTICLOCKWISE
        # - = elbow rotates CLOCKWISE
        #
        # This is the signed mathematical
        # representation used by FK.
        # ----------------------------------

        theta2 = signed_theta2
        theta2_degrees = np.degrees(theta2)

        # ----------------------------------
        # HARD RULE:
        #
        # theta2 can NEVER be negative.
        #
        # Negative theta2 = backward/
        # unnatural elbow bend.
        # ----------------------------------

        if not (
            THETA2_SIGNED_MIN_DEG
            <= theta2_degrees
            <= THETA2_SIGNED_MAX_DEG
        ):
            continue

        # ----------------------------------
        # Servo representation
        #
        # Since valid theta2 is always
        # non-negative, the servo angle is
        # simply the same 0°-180° value.
        # ----------------------------------

        theta2_servo = theta2_degrees

        # ----------------------------------
        # Check physical servo limit
        # ----------------------------------

        if not (
            THETA2_MIN_DEG
            <= theta2_servo
            <= THETA2_MAX_DEG
        ):
            continue

        # ----------------------------------
        # Valid configuration
        # ----------------------------------

        possible_solutions.append(
            (
                theta1,
                theta2,
                theta2_servo
            )
        )

    # --------------------------------------
    # No valid configuration
    # --------------------------------------

    if not possible_solutions:

        print(
            "Target rejected: no valid joint "
            "configuration exists within the limits.\n"
        )

        return None

    # --------------------------------------
    # Choose the first valid configuration
    # --------------------------------------

    theta1, theta2, theta2_servo = possible_solutions[0]

    return theta1, theta2, theta2_servo


# ==========================================
# Forward Kinematics
# ==========================================
def robot_position(L1, L2, theta1, theta2):

    x0 = 0
    y0 = 0

    # --------------------------------------
    # First link
    #
    # theta1 = 0° means DOWN
    # positive = ANTICLOCKWISE
    # negative = CLOCKWISE
    # --------------------------------------

    x1 = L1 * np.sin(theta1)
    y1 = -L1 * np.cos(theta1)

    # --------------------------------------
    # Second link
    #
    # theta2 is RELATIVE to the upper arm.
    #
    # theta2 = 0°  -> forearm continues
    #                straight from upper arm
    #
    # theta2 > 0°  -> elbow bends
    #                ANTICLOCKWISE
    #
    # theta2 < 0°  -> NOT ALLOWED
    #                (backward/unnatural bend)
    # --------------------------------------

    forearm_angle = theta1 + theta2

    x2 = (
        x1
        + L2 * np.sin(forearm_angle)
    )

    y2 = (
        y1
        - L2 * np.cos(forearm_angle)
    )

    return x0, y0, x1, y1, x2, y2


# ==========================================
# Animation
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

        # Convert current theta2 to its
        # physical 0°-270° representation.
        theta2_deg = np.degrees(theta2)

        if theta2_deg < 0:
            theta2_servo = 360 + theta2_deg
        else:
            theta2_servo = theta2_deg

        # Information
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

# ------------------------------------------
# Shoulder
#
# -30° = clockwise/backward
#   0° = resting/down
# +90° = anticlockwise/right
# +180° = up
# ------------------------------------------

THETA1_MIN = np.radians(-90)
THETA1_MAX = np.radians(180)


# ------------------------------------------
# Elbow
#
# theta2 is RELATIVE to the upper arm.
#
#   0°   = arm completely straight
#   +90° = natural anticlockwise bend
#   +180° = completely folded
#
# IMPORTANT:
#
#   theta2 can NEVER be negative.
#
# A negative theta2 would bend the elbow
# backward / into the unnatural configuration
# we do not want.
#
# So the valid mathematical elbow range is:
#
#   0° <= theta2 <= 180°
#
# We keep the servo range at 0°-270° because
# the physical servo itself may support that
# range, but IK will only generate the natural
# 0°-180° elbow configuration.
# ------------------------------------------

THETA2_MIN_DEG = 0
THETA2_MAX_DEG = 270

# Mathematical IK elbow limit
THETA2_SIGNED_MIN_DEG = 0
THETA2_SIGNED_MAX_DEG = 180


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

    # Target unreachable / invalid
    if result is None:
        return

    (
        target_theta1,
        target_theta2,
        target_theta2_servo
    ) = result

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
