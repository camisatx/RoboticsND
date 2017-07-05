import math
import numpy
from sympy import symbols, cos, sin, pi, atan2, acos, sqrt, simplify
from sympy.matrices import Matrix

"""
Test file for building the Kuka 6 DoF manipulator's forward and inverse
kinematic code.

FK(thetas) -> pose
IK(pose) -> thetas
"""

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """
    _EPS = numpy.finfo(float).eps * 4.0
    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    n = numpy.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    return numpy.array([
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
#w = 0

#px = 1.5
#py = -0.3
#pz = 1.04
#roll = -0.299
#pitch = -0.322
#yaw = -0.635
#w = 0.634

px = -0.209973
py = 2.4999
pz = 1.600
roll = -0.0004
pitch = 0.000232
yaw = 0.00076
w = 1

Rrpy = quaternion_matrix([px, py, pz, w])
print(Rrpy)

# Determine angle of EE
ee_rot = rot_x(roll) * rot_y(pitch) * rot_z(yaw)
print("EE angle = ", ee_rot)

# EE correction
ee_corr_y = Matrix([[  cos(-pi/2),        0, sin(-pi/2), 0],
                    [           0,        1,          0, 0],
                    [ -sin(-pi/2),        0, cos(-pi/2), 0],
                    [           0,        0,          0, 1]])
ee_corr_z = Matrix([[     cos(pi), -sin(pi),          0, 0],
                    [     sin(pi),  cos(pi),          0, 0],
                    [           0,        0,          1, 0],
                    [           0,        0,          0, 1]])
ee_corr = simplify(ee_corr_z * ee_corr_y)
print("EE correction = ", ee_corr)

# Get the Z axis vector for the gripper
ee_z = [Rrpy[0][0], Rrpy[1][0], Rrpy[2][0], 0]

#L_x = np.cos(roll) * np.cos(pitch)
#L_y = np.cos(roll) * np.sin(pitch) * np.sin(yaw) - np.sin(roll) * np.cos(yaw)
#L_z = np.cos(roll) * np.sin(pitch) * np.cos(yaw) + np.sin(roll) * np.sin(yaw)

# Calculate the wrist center
w_c = [px - kuka_s[d7] * ee_z[0],
       py - kuka_s[d7] * ee_z[1],
       pz - kuka_s[d7] * ee_z[2]]
print('Wrist center = ', w_c)

# Calculate joint angles using Geometric IK method
theta1 = atan2(w_c[2], w_c[0])

X1_3 = w_c[0] - kuka_s[a2]
Z1_3 = w_c[2] - kuka_s[d1]

r0 = sqrt(X1_3**2 + Z1_3**2)
print((kuka_s[a4]**2 - kuka_s[a3]**2 - r0**2) / (-2 * kuka_s[a3] * r0))
print((kuka_s[a4]**2 - kuka_s[a3]**2 - r0**2))
print((-2 * kuka_s[a3] * r0))
phi1 = acos((kuka_s[a4]**2 - kuka_s[a3]**2 - r0**2) / (-2 * kuka_s[a3] * r0))

phi2 = atan2(Z1_3, X1_3)
theta2 = phi2 - phi1

#phi3 = acos((r1**2 - kuka_s[a3]**2 - kuka_s[a4]**2) / (-2 * kuka_s[a3] * kuka_s[a4]))
#theta3 = pi - phi3

print(X1_3)
print(Z1_3)
print(r0)
print(phi1)
print(phi2)
print('____________')

###################################
r1 = sqrt(kuka_s[a4]**2 + kuka_s[a5]**2)
r2 = sqrt(w_c[0]**2 + w_c[1]**2)

#cos_phi2 = (r1**2 - kuka_s[a2]**2 - r2**2) / (-2 * kuka_s[a2] * r2)

#phi1 = atan2(kuka_s[a4], kuka_s[d4])
#phi2 = atan2(sqrt(1 - cos_phi2**2), cos_phi2)
#phi3 = atan2(py, px)
#phi4 = (r2**2 - kuka_s[a2]**2 - r1**2) / (-2 * kuka_s[a2] * r1)

#theta2 = phi2 + phi3
#theta3 = phi4 - phi1 - pi/2

#####################################
#cos_theta3 = (X1_3**2 + Z1_3**2 - kuka_s[a3]**2 - kuka_s[a4]**2) / (2 * kuka_s[a3] * kuka_s[a4])
#theta3 = atan2(cos_theta3, sqrt(1 - cos_theta3**2))
theta3 = acos((X1_3**2 + Z1_3**2 - kuka_s[a3]**2 - kuka_s[a4]**2) / (2 * kuka_s[a3] * kuka_s[a4]))

theta4, theta5, theta6 = 0, 0, 0

print(theta1)
print(theta2)
print(theta3)
