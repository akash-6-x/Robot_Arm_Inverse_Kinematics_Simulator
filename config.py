import numpy as np


# ==========================================
# Robot Configuration
# ==========================================

# ------------------------------------------
# Link lengths
# ------------------------------------------

L1 = 5
L2 = 4


# ==========================================
# Joint Limits
# ==========================================

# ------------------------------------------
# Shoulder
#
# -90° = clockwise/backward limit
#   0° = resting/down
# +180° = anticlockwise/up limit
# ------------------------------------------

THETA1_MIN = np.radians(-90)
THETA1_MAX = np.radians(180)


# ------------------------------------------
# Elbow
#
# Natural mathematical range:
#
#   0°   = completely straight
#   90°  = natural bend
#   180° = completely folded
#
# Negative theta2 is NOT allowed.
# ------------------------------------------

THETA2_SIGNED_MIN_DEG = 0
THETA2_SIGNED_MAX_DEG = 180


# ------------------------------------------
# Physical servo range
# ------------------------------------------

THETA2_MIN_DEG = 0
THETA2_MAX_DEG = 270