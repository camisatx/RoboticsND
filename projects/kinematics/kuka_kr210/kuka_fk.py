from sympy import symbols, pi, sin, cos, simplify
from sympy.matrices import Matrix

"""
Test file for building the Kuka 6 DoF manipulator's forward and inverse
kinematic code.

FK(thetas) -> pose
IK(pose) -> thetas
"""


def build_mod_dh_matrix(s, theta, alpha, d, a):
    """Build the modified DH transformation matrix based on the provided
    theta, alpha, d and a values.

    :param s: Dictionary of DH parameters for the manipulator
    :param theta: Sympy symbol
    :param alpha: Sympy symbol
    :param d: Sympy symbol
    :param a: Sympy symbol
    :return: Sympy Matrix object of the DH transformation matrix
    """
    # Create the transformation matrix template
    Ta_b = Matrix([[            cos(theta),           -sin(theta),           0,             a],
                   [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                     0,                     0,           0,             1]])
    # Substitute in the DH parameters into the matrix
    Ta_b = Ta_b.subs(s)
    return Ta_b

# Define DH param symbols
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')    # link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')    # link lengths

# Modified DH params for KUKA KR210
kuka_s = {alpha0:     0, d1:  0.75, a0:      0,
          alpha1: -pi/2, d2:     0, a1:   0.35, theta2: (theta2 - pi/2),
          alpha2:     0, d3:     0, a2:   1.25,
          alpha3: -pi/2, d4:  1.50, a3: -0.054,
          alpha4:  pi/2, d5:     0, a4:      0,
          alpha5: -pi/2, d6:     0, a5:      0,
          alpha6:     0, d7: 0.303, a6:      0, theta7: 0}

# Define Modified DH Transformation matrix
T0_1 = build_mod_dh_matrix(s=kuka_s, theta=theta1, alpha=alpha0, d=d1, a=a0)
T1_2 = build_mod_dh_matrix(s=kuka_s, theta=theta2, alpha=alpha1, d=d2, a=a1)
T2_3 = build_mod_dh_matrix(s=kuka_s, theta=theta3, alpha=alpha2, d=d3, a=a2)
T3_4 = build_mod_dh_matrix(s=kuka_s, theta=theta4, alpha=alpha3, d=d4, a=a3)
T4_5 = build_mod_dh_matrix(s=kuka_s, theta=theta5, alpha=alpha4, d=d5, a=a4)
T5_6 = build_mod_dh_matrix(s=kuka_s, theta=theta6, alpha=alpha5, d=d6, a=a5)
T6_G = build_mod_dh_matrix(s=kuka_s, theta=theta7, alpha=alpha6, d=d7, a=a6)

# Create individual transformation matrices
T0_2 = simplify(T0_1 * T1_2)    # base link to link 2
T0_3 = simplify(T0_2 * T2_3)    # bawe link to link 3
T0_4 = simplify(T0_3 * T3_4)    # base link to link 4
T0_5 = simplify(T0_4 * T4_5)    # base link to link 5
T0_6 = simplify(T0_5 * T5_6)    # base link to link 6
T0_G = simplify(T0_6 * T6_G)    # base link to gripper

# Correction to account for orientation difference between the gripper and
#   the arm base (rotation around Z axis by 180 deg and Y axis by -90 deg)
R_z = Matrix([[     cos(pi), -sin(pi),          0, 0],
              [     sin(pi),  cos(pi),          0, 0],
              [           0,        0,          1, 0],
              [           0,        0,          0, 1]])
R_y = Matrix([[  cos(-pi/2),        0, sin(-pi/2), 0],
              [           0,        1,          0, 0],
              [ -sin(-pi/2),        0, cos(-pi/2), 0],
              [           0,        0,          0, 1]])
R_corr = simplify(R_z * R_y)

# Total homogeneous transform between base and gripper with orientation correction applied
T_total = simplify(T0_G * R_corr)

# Numerically evaluate transforms (compare this with output of tf_echo)
print(T0_1)
print(T1_2)
print(T2_3)
print(T3_4)
print(T4_5)
print(T5_6)
print(T6_G)
#print(T0_G)
#print('T0_1 = ', T0_1.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T1_2 = ', T0_2.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T2_3 = ', T0_3.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T3_4 = ', T0_4.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T4_5 = ', T0_5.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T5_6 = ', T0_6.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T0_G = ', T0_G.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
print('T_total = ', T_total.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
