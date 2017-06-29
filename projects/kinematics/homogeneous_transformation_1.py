#!usr/bin/env python

from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

# Conversion factors
rtd = 180.0 / pi
dtr = pi / 180.0

###############################################################
# Problem Statement:
  # Let P be a vector expressed in frame {C} with (x,y,z)
  # coordinates = (15.0, 0.0, 42.0)
  # Rotate P about the Y-axis by angle = 110 degrees. 
  # Then translate the vector 1 unit
  # in the X-axis and 30 units in the Z-axis. 
  # Print the new (x, y, z) coordinates of P after the transformation.  
###############################################################
#### Create symbols for joint variables
q1 = symbols('q1')
gamma  = symbols('gamma')

# Construct P in {A} which should be a 4x1 Matrix
P = Matrix([[15.0], [0.0], [42.0], [1]])
# Define a homogeneous transformation of T Should be a 4x4 matrix
T = Matrix([[ cos(q1), 0, sin(q1),  1],
            [       0, 1,       0,  0],
            [-sin(q1), 0, cos(q1), 30],
            [       0, 0,        0, 1]])
# Calculate new coordinates of P in {A}
P_new = simplify(T * P)
print('P_new is :', P_new)

# Evaluate numerically
print('The new coordinates of P_A are :', P_new.evalf(subs={q1: 110*dtr}))
