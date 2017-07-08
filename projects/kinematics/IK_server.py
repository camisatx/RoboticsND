#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Import modules
import numpy as np
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []	# Stores joint angle values calculated
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

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
                 alpha1: -pi/2, d2:     0, a1:   0.35, theta2: (theta2 - pi/2),
                 alpha2:     0, d3:     0, a2:   1.25,
                 alpha3: -pi/2, d4:  1.50, a3: -0.054,
                 alpha4:  pi/2, d5:     0, a4:      0,
                 alpha5: -pi/2, d6:     0, a5:      0,
                 alpha6:     0, d7: 0.303, a6:      0, theta7: 0}
            
            # EE location and orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])
            
            # Test pose & orientation of end effector w/ all angles at 0 degrees
            #px = 2.1529
            #py = 0.0
            #pz = 1.9465
            #roll = 0.0
            #pitch = 0.0
            #yaw = 0.0
            
            ###################################################################
            # Step 1: Convert pose and orientation into a transformation matrix
            #   to compute the wrist center
            
            # Build EE roation matrix
            Rrpy = rot_x(roll) * rot_y(pitch) * rot_z(yaw)
            lx = Rrpy[0, 0]
            ly = Rrpy[1, 0]
            lz = Rrpy[2, 0]
            
            # Calculate the wrist center (test should be (1.85, 0, 1.947)
            wx = px - (s[d7] + s[d6]) * lx
            wy = py - (s[d7] + s[d6]) * ly
            wz = pz - (s[d7] + s[d6]) * lz
            #print(wx, wy, wz)
            
            ###################################################################
            # Step 2: Calculate thetas for joint 1, 2 and 3 (determines EE pos)
            
            theta1 = atan2(wy, wx).evalf()
            theta1 = np.clip(theta1, -185*dtr, 185*dtr)
            #print('theta1: %s' % theta1)
            
            # Calculation from graph 1
            #r1 = sqrt(s[a2]**2 + s[d4]**2)
            #r2 = sqrt(sqrt(wx**2 + wy**2)**2 + wz**2)
            #phi1 = atan2(s[a2], s[d4])
            #cos_phi4 = (r2**2 - s[a1]**2 - r1**2) / (-2 * s[a1] * r1)
            #phi4 = atan2(sqrt(1 - cos_phi4**2), cos_phi4)
            #theta3 = (phi4 - phi1 - pi/2).evalf()
            ##theta3 = (phi4 - phi1).evalf()
            #print('graph_1 theta3: %s' % theta3)
            
            X1_3 = wx - s[a2]
            Y1_3 = wy - s[d1]
            Z1_3 = wz - s[d1]
            
            r1 = sqrt(X1_3**2 + Z1_3**2)
            #r1 = sqrt(X1_3**2 + Y1_3**2)
            #r1 = sqrt(sqrt(wx**2 + wz**2) + wy**2)
            cos_phi3 = (r1**2 - s[a2]**2 - s[d4]**2) / (-2 * s[a2] * s[d4])
            phi3 = atan2(sqrt(1 - cos_phi3**2), cos_phi3)
            theta3 = (pi - phi3).evalf()
            #print('robogrok theta3: %s' % theta3)
            
            cos_phi1 = (s[d4]**2 - s[a2]**2 - r1**2) / (-2 * s[a2] * r1)
            phi1 = atan2(sqrt(1 - cos_phi1**2), cos_phi1)
            phi2 = atan2(Z1_3, X1_3)
            theta2 = phi2 - phi1
            #print('robogrok theta2: %s' % theta2)
            
            ###################################################################
            # Step 3: Determine the rotation matrix for the spherical wrist joints
            
            # Build the transformation matrices of the first 3 joints
            T0_1 = build_mod_dh_matrix(s=s, theta=theta1, alpha=alpha0, d=d1, a=a0)
            T1_2 = build_mod_dh_matrix(s=s, theta=theta2, alpha=alpha1, d=d2, a=a1)
            T2_3 = build_mod_dh_matrix(s=s, theta=theta3, alpha=alpha2, d=d3, a=a2)
            
            # Rotation matrix of the first three joints
            R0_3 = (T0_1 * T1_2 * T2_3).evalf(subs={theta1: theta1,
                                                    theta2: theta2,
                                                    theta3: theta3})[0:3, 0:3]
            
            # Calculate the symbolic rotation matrix of the spherical wrist joints
            R3_6 = R0_3.inv() * Rrpy
            
            ###################################################################
            # Step 4: Calculate the spherical wrist joint angles
            
            r31 = R3_6[2, 0]
            r11 = R3_6[0, 0]
            r21 = R3_6[1, 0]
            r32 = R3_6[2, 1]
            r33 = R3_6[2, 2]
            
            theta4 = atan2(r32, r33).evalf()
            
            theta5 = atan2(-r31, sqrt(r11**2 + r21**2)).evalf()
            
            theta6 = atan2(r21, r11).evalf()

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
