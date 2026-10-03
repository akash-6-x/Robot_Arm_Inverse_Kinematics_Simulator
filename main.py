import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox

from config import L1, L2
from kinematics import inverse_kinematics
from animation import (
    draw_robot,
    move_robot,
    set_status,
    setup_robot_scene
)


# ==========================================
# Starting Joint Angles
# ==========================================

current_theta1 = np.radians(0)
current_theta2 = np.radians(0)


# ==========================================
# Create Graph
# ==========================================

fig, (ax, gauge_panel) = plt.subplots(
    1,
    2,
    figsize=(11, 7),
    gridspec_kw={"width_ratios": [4, 1.25], "wspace": 0.18}
)


# ==========================================
# Target Movement
# ==========================================

def move_to_target(target_x, target_y):

    global current_theta1
    global current_theta2

    target_x_input.set_val(f"{target_x:.2f}")
    target_y_input.set_val(f"{target_y:.2f}")

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
        set_status(
            ax,
            "Target rejected: outside the current reachable limits.",
            "tab:red"
        )
        fig.canvas.draw_idle()
        return

    (
        target_theta1,
        target_theta2,
        target_theta2_servo
    ) = result

    set_status(ax, "Target accepted: moving robot.", "tab:green")

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
    set_status(ax, "Target reached.", "tab:green")
    fig.canvas.draw_idle()


# ==========================================
# Input Controls
# ==========================================

def on_target_submit(_):

    try:
        target_x = float(target_x_input.text)
        target_y = float(target_y_input.text)
    except ValueError:
        set_status(ax, "Enter valid numeric X and Y coordinates.", "tab:red")
        fig.canvas.draw_idle()
        return

    move_to_target(target_x, target_y)


def on_click(event):

    if event.inaxes != ax:
        return

    move_to_target(event.xdata, event.ydata)


# ==========================================
# Draw Initial Robot
# ==========================================

setup_robot_scene(ax, gauge_panel, L1, L2)
draw_robot(
    ax,
    L1,
    L2,
    current_theta1,
    current_theta2,
    0,
    -(L1 + L2)
)

gauge_panel.text(
    0.5,
    0.16,
    "Target Position",
    horizontalalignment="center",
    verticalalignment="center",
    fontsize=10,
    fontweight="bold"
)

target_x_input = TextBox(
    gauge_panel.inset_axes([0.04, 0.06, 0.42, 0.06]),
    "X ",
    initial="0.00",
    textalignment="center"
)
target_y_input = TextBox(
    gauge_panel.inset_axes([0.6, 0.06, 0.42, 0.06]),
    "Y ",
    initial=f"{- (L1 + L2):.2f}",
    textalignment="center"
)
move_button = Button(
    gauge_panel.inset_axes([0.25, -0.02, 0.50, 0.06]),
    "Move",
    color="lightsteelblue",
    hovercolor="lightskyblue"
)
move_button.on_clicked(on_target_submit)

fig.canvas.mpl_connect("button_press_event", on_click)

plt.show()
