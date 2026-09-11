import numpy as np

from config import (
    THETA1_MIN,
    THETA1_MAX,
    THETA2_SIGNED_MIN_DEG,
    THETA2_SIGNED_MAX_DEG,
    THETA2_MIN_DEG,
    THETA2_MAX_DEG
)


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
    # Calculate elbow angle magnitude
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
    # Generate valid configuration
    #
    # Only natural positive elbow bend
    # is allowed.
    # --------------------------------------

    possible_solutions = []

    for signed_theta2 in [elbow_angle]:

        # ----------------------------------
        # Calculate mathematical theta1
        # ----------------------------------

        alpha1 = (
            target_angle
            - np.arctan2(
                L2 * np.sin(signed_theta2),
                L1 + L2 * np.cos(signed_theta2)
            )
        )

        # ----------------------------------
        # Convert into our theta1 convention
        #
        # 0°   = DOWN
        # +    = ANTICLOCKWISE
        # -    = CLOCKWISE
        # ----------------------------------

        theta1 = alpha1 + np.pi / 2

        # ----------------------------------
        # Normalize theta1
        # ----------------------------------

        theta1 = (
            (theta1 + np.pi)
            % (2 * np.pi)
            - np.pi
        )

        # ----------------------------------
        # Check shoulder limit
        # ----------------------------------

        if not (
            THETA1_MIN <= theta1 <= THETA1_MAX
        ):
            continue

        # ----------------------------------
        # theta2
        # ----------------------------------

        theta2 = signed_theta2

        theta2_degrees = np.degrees(theta2)

        # ----------------------------------
        # Negative theta2 is forbidden
        # ----------------------------------

        if not (
            THETA2_SIGNED_MIN_DEG
            <= theta2_degrees
            <= THETA2_SIGNED_MAX_DEG
        ):
            continue

        # ----------------------------------
        # Servo representation
        # ----------------------------------

        theta2_servo = theta2_degrees

        # ----------------------------------
        # Physical servo limit
        # ----------------------------------

        if not (
            THETA2_MIN_DEG
            <= theta2_servo
            <= THETA2_MAX_DEG
        ):
            continue

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
    # Choose first valid configuration
    # --------------------------------------

    return possible_solutions[0]


# ==========================================
# Forward Kinematics
# ==========================================

def robot_position(L1, L2, theta1, theta2):

    x0 = 0
    y0 = 0

    # --------------------------------------
    # First link
    # --------------------------------------

    x1 = L1 * np.sin(theta1)
    y1 = -L1 * np.cos(theta1)

    # --------------------------------------
    # Second link
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