#!/usr/bin/env python
import numpy as np
from sympy.matrices import Matrix
from sympy import symbols, atan2, sqrt

# Conversion factors
rtd = 180.0 / np.pi
dtr = np.pi / 180.0

# Fixed Axis X-Y-Z Rotation Matrix
R_XYZ = Matrix([[ 0.353553390593274, -0.306186217847897, 0.883883476483184],
                [ 0.353553390593274,  0.918558653543692, 0.176776695296637],
                [-0.866025403784439,               0.25, 0.433012701892219]])

# Useful terms within the rotation matrix
r_11 = R_XYZ[0, 0]
r_21 = R_XYZ[1, 0]
r_31 = R_XYZ[2, 0]
r_32 = R_XYZ[2, 1]
r_33 = R_XYZ[2, 2]

# Calculate the Euler angles that produces a rotation equivalent to R (above)
# NOTE: Be sure your answer has units of DEGREES!
alpha = atan2(r_21, r_11) * rtd # rotation about Z-axis
beta  = atan2(-r_31, sqrt(r_11 * r_11 + r_21 * r_21)) * rtd # rotation about Y-axis
gamma = atan2(r_32, r_33) * rtd # rotation about X-axis
