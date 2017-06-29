############ Rotation_Template.py
#!/usr/bin/env python
from sympy import symbols, cos, sin, pi, sqrt
from sympy.matrices import Matrix
import numpy as np

### Create symbols for joint variables
q1, q2 = symbols('q1:3')

# Create a symbolic matrix representing an extrinsic sequence of rotations 
  # about the Z and then Y axes. Let the rotation about the Y axis be described
  # by q1 and the rotation about Z by q2. 
####### TO DO ########
# Replace R_y and R_z with the appropriate (symbolic) elementary rotation matrices 
  # and then compute ZY_extrinsic. 
R_y = Matrix([[ cos(q1),        0, sin(q1)],
              [       0,        1,       0],
              [-sin(q1),        0, cos(q1)]])
R_z = Matrix([[ cos(q2), -sin(q2),       0],
              [ sin(q2),  cos(q2),       0],
              [       0,        0,       1]])
ZY_extrinsic_sym = R_y * R_z

####### TO DO ########
# Numerically evaluate ZY_extrinsic assuming:
   # q1 = 45 degrees and q2 = 60 degrees. 
   # NOTE: Trigonometric functions in Python assume the input is in radians!  

dtr = np.pi / 180.0
ZY_extrinsic_num = ZY_extrinsic_sym.evalf(subs={q1: 45*dtr, q2: 60*dtr})
