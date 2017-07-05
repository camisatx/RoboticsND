import math
import numpy as np
from sympy import symbols, cos, sin, pi, atan2, sqrt, simplify
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


def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> np.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> np.allclose(M, np.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> np.allclose(M, np.diag([1, -1, -1, 1]))
    True

    """
    _EPS = np.finfo(float).eps * 4.0
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])


def rot_x(q):
    R_x = Matrix([[       1,       0,       0],
                  [       0,  cos(q), -sin(q)],
                  [       0,  sin(q),  cos(q)]])
    return R_x


def rot_y(q):
    R_y = Matrix([[  cos(q),       0,  sin(q)],
                  [       0,       1,       0],
                  [ -sin(q),       0,  cos(q)]])
    return R_y


def rot_z(q):
    R_z = Matrix([[  cos(q), -sin(q),       0],
                  [  sin(q),  cos(q),       0],
                  [       0,       0,       1]])
    return R_z

rtd = 180 / np.pi
dtr = np.pi / 180

# Define DH param symbols
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')
alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7 = symbols('alpha1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')    # link offsets
a1, a2, a3, a4, a5, a6, a7 = symbols('a1:8')    # link lengths

# Modified DH params for KUKA KR210
kuka_s = {alpha1:     0, d1:  0.75, a1:      0,
          alpha2: -pi/2, d2:     0, a2:   0.35, theta2: (theta2 - pi/2),
          alpha3:     0, d3:     0, a3:   1.25,
          alpha4: -pi/2, d4:  1.50, a4: -0.054,
          alpha5:  pi/2, d5:     0, a5:      0,
          alpha6: -pi/2, d6:     0, a6:      0,
          alpha7:     0, d7: 0.303, a7:      0, theta7: 0}

#px = req.poses[x].position.x
#py = req.poses[x].position.y
#pz = req.poses[x].position.z
#(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
#    [req.poses[x].orientation.x, req.poses[x].orientation.y,
#     req.poses[x].orientation.z, req.poses[x].orientation.w])
#R = tf.transformations.quaternion_matrix(
#    [req.poses[x].orientation.x, req.poses[x].orientation.y,
#     req.poses[x].orientation.z, req.poses[x].orientation.w])
#a, b, g = tf.transformations.euler_from_matrix(R, axes='ryzx')

# Test pose and orientation of the end effector
#px = 2.153
#py = 0.0
#pz = 1.946
#roll = 0.0
#pitch = 0.0
#yaw = 0.0

#px = 1.5
#py = -0.3
#pz = 1.04
#roll = -0.299
#pitch = -0.322
#yaw = -0.635

px = -0.209973
py = 2.4999
pz = 1.600
roll = -0.0004
pitch = 0.000232
yaw = 0.00076

Rrpy = rot_x(roll) * rot_y(pitch) * rot_z(yaw)
#print(Rrpy)

# Determine angle of EE
#ee_rot = rot_x(roll) * rot_y(pitch) * rot_z(yaw)
#print("EE angle = ", ee_rot)

lx = Rrpy[0, 0]
ly = Rrpy[1, 0]
lz = Rrpy[2, 0]

# Calculate the wrist center
wx = px - (kuka_s[d6] + kuka_s[d7]) * lx
wy = py - (kuka_s[d6] + kuka_s[d7]) * ly
wz = pz - (kuka_s[d6] + kuka_s[d7]) * lz
#print("wx:", wx, "; wy:", wy, "; wz:", wz)

###############################################################################
# Calculate joint angles using Geometric IK method

# Joint 1 angle
theta1 = atan2(wy, wx).evalf()
theta1 = np.clip(theta1, -185 * dtr, 185 * dtr)

#D0_WC = sqrt(wx**2 + wy**2)
#X1_3 = D0_WC - kuka_s[a2]  # unsure which to use
X1_3 = wx - kuka_s[a2]
Z1_3 = wz - kuka_s[d1]

# Theta 3 Calculation (using the law of cosines)
#D = (X1_3**2 + Z1_3**2 - kuka_s[a3]**2 - kuka_s[a4]**2) / (2 * kuka_s[a3] * kuka_s[a4])    # Shoud this use a4 or d4?
D = (X1_3**2 + Z1_3**2 - kuka_s[a3]**2 - kuka_s[d4]**2) / (2 * kuka_s[a3] * kuka_s[d4])

# Check if D is greater than 1 to prevent imaginary numbers
if abs(D) > 1:
    D = 1
    raise Warning("D for angle 3 is greater than 1")

#theta3 = atan2(D, -sqrt(1 - D**2)).evalf()     # Is D first, or second?
theta3 = atan2(-sqrt(1 - D**2), D).evalf()

# Theta 2 Calculation
S1 = ((kuka_s[a3] + kuka_s[d4] * cos(theta3)) * Z1_3 - kuka_s[d4] * sin(theta3) * X1_3) / (X1_3**2 + Z1_3**2)
C1 = ((kuka_s[a3] + kuka_s[d4] * cos(theta3)) * X1_3 + kuka_s[d4] * sin(theta3) * Z1_3) / (X1_3**2 + Z1_3**2)
theta2 = atan2(S1, C1).evalf()
theta2 = np.clip(theta2, -45 * dtr, 85 * dtr)

##############################################################################

# Define Modified DH Transformation matrix
T0_1 = build_mod_dh_matrix(s=kuka_s, theta=theta1, alpha=alpha1, d=d1, a=a1)
T1_2 = build_mod_dh_matrix(s=kuka_s, theta=theta2, alpha=alpha2, d=d2, a=a2)
T2_3 = build_mod_dh_matrix(s=kuka_s, theta=theta3, alpha=alpha3, d=d3, a=a3)
T3_4 = build_mod_dh_matrix(s=kuka_s, theta=theta4, alpha=alpha4, d=d4, a=a4)
T4_5 = build_mod_dh_matrix(s=kuka_s, theta=theta5, alpha=alpha5, d=d5, a=a5)
T5_6 = build_mod_dh_matrix(s=kuka_s, theta=theta6, alpha=alpha6, d=d6, a=a6)
T6_G = build_mod_dh_matrix(s=kuka_s, theta=theta7, alpha=alpha7, d=d7, a=a7)

# Create individual transformation matrices
T0_2 = simplify(T0_1 * T1_2)    # base link to link 2
T0_3 = simplify(T0_2 * T2_3)    # base link to link 3
T0_4 = simplify(T0_3 * T3_4)    # base link to link 4
T0_5 = simplify(T0_4 * T4_5)    # base link to link 5
T0_6 = simplify(T0_5 * T5_6)    # base link to link 6
T0_G = simplify(T0_6 * T6_G)    # base link to gripper

# Correction to account for orientation difference between definition of gripper link in
#   URDF file and the DH convention.
#   (rotation around Z axis by 180 deg and X axis by -90 deg)
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
#print('T0_1 = ', T0_1.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T1_2 = ', T0_2.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T2_3 = ', T0_3.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T3_4 = ', T0_4.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T4_5 = ', T0_5.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T5_6 = ', T0_6.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T6_G = ', T0_G.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))
#print('T_total = ', T_total.evalf(subs={theta1: 0, theta2: 0, theta3: 0, theta4: 0, theta5: 0, theta6:0}))


###############################################################################
# Calculate wrist rotation
#R0_3 = T0_3.evalf(subs={theta1: theta1, theta2: theta2, theta3: theta3})[0:3,0:3]
R0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={theta1: theta1, theta2: theta2, theta3: theta3})[0:3,0:3]
R3_6 = R0_3.inv() * Rrpy

r31 = R3_6[2, 0]
r11 = R3_6[0, 0]
r21 = R3_6[1, 0]
r32 = R3_6[2, 1]
r33 = R3_6[2, 2]

# Joing 4 Calculation
theta4 = atan2(r32, r33).evalf()
theta4 = np.clip(theta4, -350 * dtr, 350 * dtr)

# Joint 5 Calculation
theta5 = atan2(-r31, sqrt(pow(r11, 2) + pow(r21, 2))).evalf()
theta5 = np.clip(theta5, -125 * dtr, 125 * dtr)

# Joint 6 Calculation
theta6 = atan2(r21, r11).evalf()
theta6 = np.clip(theta6, -350 * dtr, 350 * dtr)

print(theta1)
print(theta2)
print(theta3)
print(theta4)
print(theta5)
print(theta6)
