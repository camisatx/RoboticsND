import numpy as np
from sympy import symbols, pi, sin, cos, atan2, sqrt, simplify
from sympy.matrices import Matrix
import tf

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


def rot_x(q):
    r_x = Matrix([[ 1,      0,       0],
                  [ 0, cos(q), -sin(q)],
                  [ 0, sin(q),  cos(q)]])
    return r_x


def rot_y(q):
    r_y = Matrix([[  cos(q), 0, sin(q)],
                  [       0, 1,      0],
                  [ -sin(q), 0, cos(q)]])
    return r_y


def rot_z(q):
    r_z = Matrix([[ cos(q), -sin(q), 0],
                  [ sin(q),  cos(q), 0],
                  [      0,       0, 1]])
    return r_z

# Conversion factors between radians and degrees
rtd = 180 / pi
dtr = pi / 180

# Define DH parameter symbols
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')    # link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')    # link lengths

# Modified DH params for KUKA KR210
s = {alpha0:     0, d1:  0.75, a0:      0,
     alpha1: -pi/2, d2:     0, a1:   0.35, theta2: theta2 - pi/2,
     alpha2:     0, d3:     0, a2:   1.25,
     alpha3: -pi/2, d4:  1.50, a3: -0.054,
     alpha4:  pi/2, d5:     0, a4:      0,
     alpha5: -pi/2, d6:     0, a5:      0,
     alpha6:     0, d7: 0.303, a6:      0, theta7: 0,}

# EE location and orientation
#px = req.poses[x].position.x
#py = req.poses[x].position.y
#pz = req.poses[x].position.z
#(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
#    [req.poses[x].orientation.x, req.poses[x].orientation.y,
#     req.poses[x].orientation.z, req.poses[x].orientation.w])

# Test pose and orientation of the end effector with all angles at 0 degrees
px = 2.1529
py = 0.0
pz = 1.9465
roll = 0.0
pitch = 0.0
yaw = 0.0

##############################################################################
# Step 1: Convert pose and orientation into a transformation matrix to 
#   compute the wrist center

# Build EE roation matrix
Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
lx = Rrpy[0, 0]
ly = Rrpy[1, 0]
lz = Rrpy[2, 0]

# Calculate the wrist center (test should be (1.85, 0, 1.947)
wx = px - (s[d7] + s[d6]) * lx
wy = py - (s[d7] + s[d6]) * ly
wz = pz - (s[d7] + s[d6]) * lz
#print('WC location: (%s, %s, %s)' % (wx, wy, wz))

##############################################################################
# Step 2: Calculate thetas for joint 1, 2 and 3 (determines EE position)

# Determine the angle for joint 1
theta1 = atan2(wy, wx).evalf()
theta1 = np.clip(theta1, -185*dtr, 185*dtr)

# Find the coordinates of x2, y2 and z2 considering theta 1
x2 = s[a1] * cos(theta1)
y2 = s[a1] * sin(theta1)
z2 = s[d1]

# Find the x, y and z distances between joint 2 and the wrist center
X2_WC = wx - x2
Y2_WC = wy - y2
Z2_WC = wz - z2

# Find the distances between joint 2 and the wrist center
L2_WC = sqrt(X2_WC**2 + Y2_WC**2 + Z2_WC**2)

# Find the distance between joint 2 and the wrist center
L3_4 = 0.96     # Distance from joint 3 to joint 4
L4_5 = 0.54     # Distance from joint 4 to joint 5 (WC)
L3_4_x = sqrt(L3_4**2 + abs(s[a3])**2)      # X distance from joint 3 to joint 4
phi1 = pi - atan2(abs(s[a3]), L3_4_x)
L3_WC = sqrt(L3_4**2 + L4_5**2 - 2 * L3_4 * L4_5 * cos(phi1))

# Determine the angle for joint 3
cos_phi2 = (L2_WC**2 - L3_WC**2 - s[a2]**2) / (-2 * L3_WC * s[a2])
if abs(cos_phi2) > 1:
    cos_phi2 = 1
    print('cos_phi2 is greater than 1')
phi2 = atan2(sqrt(1 - cos_phi2**2), cos_phi2)
theta3 = (pi/2 - phi2).evalf()
theta3 = np.clip(theta3, -210*dtr, (155-90)*dtr)

# Determine the angle for joint 2
L2_WC_xy = sqrt(X2_WC**2 + Y2_WC**2)
phi3 = atan2(Z2_WC, L2_WC_xy)
cos_phi4 = (L3_WC**2 - L2_WC**2 - s[a2]**2) / (-2 * L2_WC * s[a2])
if abs(cos_phi4) > 1:
    cos_phi4 = 1
    print('cos_phi4 is greater than 1')
phi4 = atan2(sqrt(1 - cos_phi4**2), cos_phi4)
theta2 = (pi/2 - (phi3 + phi4)).evalf()
theta2 = np.clip(theta2, -45*dtr, 85*dtr)

##############################################################################
# Step 3: Determine the rotation matrix for the spherical wrist joints

# Build the transformation matrices of the first 3 joints
T0_1 = build_mod_dh_matrix(s=s, theta=theta1, alpha=alpha0, d=d1, a=a0)
T1_2 = build_mod_dh_matrix(s=s, theta=theta2, alpha=alpha1, d=d2, a=a1)
T2_3 = build_mod_dh_matrix(s=s, theta=theta3, alpha=alpha2, d=d3, a=a2)

# Rotation matrix of the first three joints
R0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={theta1: theta1,
                                        theta2: theta2,
                                        theta3: theta3})[0:3, 0:3]

# Correction to account for orientation difference between definition of
#   gripper link in the URDF file and the DH convention.
#   (rotation around Z axis by 180 deg and Y axis by -90 deg)
R_corr = simplify(rot_z(pi) * rot_y(-pi/2))

# Calculate the symbolic rotation matrix of the spherical wrist joints
R3_6 = R0_3.T * Rrpy * R_corr

##############################################################################
# Step 4: Calculate the spherical wrist joint angles by converting the
#   the rotation matrix to Euler angles

# tf requires a numpy matrix instead of a sympy matrix
R3_6_np = np.array(R3_6).astype(np.float64)

# Convert the rotation matrix to Euler angles using tf
alpha, beta, gamma = tf.transformations.euler_from_matrix(
    R3_6_np, axes='rxyz')   # xyx, yzx, xyz
theta4 = alpha
theta5 = beta
theta6 = gamma

theta4 = np.pi/2 + theta4
theta5 = np.pi/2 - theta5
#theta6 = theta6 - 2*np.pi

#r11 = R3_6[0, 0]
#r12 = R3_6[0, 1]
#r13 = R3_6[0, 2]
#r21 = R3_6[1, 0]
#r31 = R3_6[2, 0]
#r32 = R3_6[2, 1]
#r33 = R3_6[2, 2]
#
## Pitch angle; rotation around the y-axis
#theta5 = atan2(-r31, sqrt(r11**2 + r21**2)).evalf()
#theta5 = np.clip(theta5, -125*dtr, 125*dtr)
#
#if r31 == 1:
#    # Gimbal lock at pitch = -90
#    theta4 = 0                          # yaw = 0
#    theta6 = atan2(-r12, -r13).evalf()  # roll
#    print('Gimbal lock at pitch = -90')
#elif r31 == -1:
#    # Gimal lock at pitch = 90
#    theta4 = 0                          # yaw = 0
#    theta6 = atan2(r12, r13).evalf()    # roll
#    print('Gimbal lock at pitch = 90')
#else:
#    # General orientation
#
#    # Yaw angle; rotation around the z-axis
#    theta4 = (atan2(r21, r11)).evalf()
#    theta4 = np.clip(theta4, -350*dtr, 350*dtr)
#    
#    # Roll angle; rotation around the x-axis
#    theta6 = (atan2(r32, r33)).evalf()
#    theta6 = np.clip(theta6, -350*dtr, 350*dtr)

print('Theta 1: %s' % theta1)
print('Theta 2: %s' % theta2)
print('Theta 3: %s' % theta3)
print('Theta 4: %s' % theta4)
print('Theta 5: %s' % theta5)
print('Theta 6: %s' % theta6)
