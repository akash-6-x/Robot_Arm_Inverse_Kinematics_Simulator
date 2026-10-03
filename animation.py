import numpy as np
import matplotlib.pyplot as plt

from kinematics import robot_position


def setup_robot_scene(ax):

    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect("equal")
    ax.grid()
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("2-Link Robot - Click to Move")

    trail, = ax.plot(
        [], [], linestyle="--", color="tab:orange", linewidth=1.5,
        alpha=0.8
    )
    robot, = ax.plot([], [], marker="o", linewidth=3)
    target, = ax.plot(
        [], [], marker="x", markersize=12, markeredgewidth=3
    )
    info = ax.text(
        0.02,
        0.97,
        "",
        transform=ax.transAxes,
        verticalalignment="top"
    )

    ax._air_artists = trail, robot, target, info


def draw_robot(ax, L1, L2, theta1, theta2, target_x, target_y):

    x0, y0, x1, y1, x2, y2 = robot_position(
        L1, L2, theta1, theta2
    )

    _, robot, target, info = ax._air_artists
    robot.set_data([x0, x1, x2], [y0, y1, y2])
    target.set_data([target_x], [target_y])

    theta2_deg = np.degrees(theta2)
    theta2_servo = 360 + theta2_deg if theta2_deg < 0 else theta2_deg

    info.set_text(
        f"Target: ({target_x:.2f}, {target_y:.2f})\n"
        f"Current: ({x2:.2f}, {y2:.2f})\n"
        f"Theta 1: {np.degrees(theta1):.2f}°\n"
        f"Theta 2: {theta2_deg:.2f}° (natural)\n"
        f"Theta 2 servo: {theta2_servo:.2f}°"
    )

    return x2, y2


def ease_in_out(t):

    return t * t * (3 - 2 * t)


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
    trail, _, _, _ = ax._air_artists
    trail_x = []
    trail_y = []
    trail.set_data([], [])

    for i in range(frames + 1):

        t = ease_in_out(i / frames)

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

        x2, y2 = draw_robot(
            ax, L1, L2, theta1, theta2, target_x, target_y
        )

        trail_x.append(x2)
        trail_y.append(y2)
        trail.set_data(trail_x, trail_y)

        plt.pause(0.03)

    return target_theta1, target_theta2
