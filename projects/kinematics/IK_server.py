#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Import modules
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

            # Define DH param symbols

            # Define joint angle symbols
            theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1:7')
            alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7 = symbols('alpha1:8')
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:d8')    # link offsets
            a1, a2, a3, a4, a5, a6, a7 = symbols('a1:a8')    # link lengths

            # Modified DH params
            s = {theta1: theta1, alpha1:   0, d1: d1, a1: a1,
                 theta2: theta2, alpha2: -90, d2:  0, a2: a2,
	         theta3: theta3, alpha3:   0, d3:  0, a3: a3,
	         theta4: theta4, alpha4: -90, d4: d4, a4:  0,
		 theta5: theta5, alpha5:  90, d5:  0, a5:  0,
		 theta6: theta6, alpha6: -90, d6: d6, a6:  0}

            # Define Modified DH Transformation matrix
            T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a1],
                           [ sin(q1)*cos(alpha1), cos(q1)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d1],
                           [ sin(q1)*sin(alpha1), cos(q1)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d1],
                           [                   0,                   0,            0,               1]])
            T0_1 = T0_1.subs(s)

            T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a2],
                           [ sin(q2)*cos(alpha2), cos(q2)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d2],
                           [ sin(q2)*sin(alpha2), cos(q2)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d2],
                           [                   0,                   0,            0,               1]])
            T1_2 = T1_2.subs(s)

            T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a3],
                           [ sin(q3)*cos(alpha3), cos(q3)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d3],
                           [ sin(q3)*sin(alpha3), cos(q3)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d3],
                           [                   0,                   0,            0,               1]])
            T2_3 = T2_3.subs(s)

            T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a4],
                           [ sin(q4)*cos(alpha4), cos(q4)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d4],
                           [ sin(q4)*sin(alpha4), cos(q4)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d4],
                           [                   0,                   0,            0,               1]])
            T3_4 = T3_4.subs(s)

            T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a5],
                           [ sin(q5)*cos(alpha5), cos(q5)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d5],
                           [ sin(q5)*sin(alpha5), cos(q5)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d5],
                           [                   0,                   0,            0,               1]])
            T4_5 = T4_5.subs(s)

            T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a6],
                           [ sin(q6)*cos(alpha6), cos(q6)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d6],
                           [ sin(q6)*sin(alpha6), cos(q6)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d6],
                           [                   0,                   0,            0,               1]])
            T5_6 = T5_6.subs(s)

            T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                           [ sin(q7)*cos(alpha7), cos(q7)*cos(alpha7), -sin(alpha7), -sin(alpha7)*d7],
                           [ sin(q7)*sin(alpha7), cos(q7)*sin(alpha7),  cos(alpha7),  cos(alpha7)*d7],
                           [                   0,                   0,            0,               1]])
            T6_G = T6_G.subs(s)

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
            print('T0_1 = ', T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
            print('T1_2 = ', T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
            print('T2_3 = ', T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
            print('T3_4 = ', T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
            print('T4_5 = ', T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
            print('T5_6 = ', T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))
            print('T6_G = ', T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6:0}))

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
            t = arctan()

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
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
