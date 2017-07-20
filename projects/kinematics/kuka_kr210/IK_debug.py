import numpy as np
from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generate test cases can be added to the test_cases dictionary
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[2.1529,0,1.9465],
                  [0,0,0,1]],
                  [1.8499,0,1.9465],
                  [0,0,0,0,0,0]],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ###########################################################################
    ## Insert IK code here starting at: Define DH parameter symbols
    ## YOUR CODE HERE!
    ## Ending at: Populate response for the IK request

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
         alpha6:     0, d7: 0.303, a6:      0, theta7: 0}
    
    # EE location and orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
         req.poses[x].orientation.z, req.poses[x].orientation.w])
    
    ###################################################################
    # Step 1: Convert pose and orientation into a transformation matrix
    #   to compute the wrist center
    
    # Build EE roation matrix
    Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
    lx = Rrpy[0, 0]
    ly = Rrpy[1, 0]
    lz = Rrpy[2, 0]
    
    # Calculate the wrist center (test should be (1.85, 0, 1.947)
    wc_x = px - (s[d7] + s[d6]) * lx
    wc_y = py - (s[d7] + s[d6]) * ly
    wc_z = pz - (s[d7] + s[d6]) * lz
    #print('WC location: (%s, %s, %s)' % (wc_x, wc_y, wc_z))

    ###################################################################
    # Step 2: Calculate thetas for joint 1, 2 and 3 (determines EE pos)
    
    # Determine the angle for joint 1
    theta1 = atan2(wc_y, wc_x).evalf()
    theta1 = np.clip(theta1, -185*dtr, 185*dtr)
    
    # Find the coordinates of x2, y2, and z2 considering theta 1
    x2 = s[a1] * cos(theta1)
    y2 = s[a1] * sin(theta1)
    z2 = s[d1]

    # Find the x, y, and z distances between joint 2 and wrist center
    X2_WC = wc_x - x2
    Y2_WC = wc_y - y2
    Z2_WC = wc_z - z2

    # Find the distances between joint 2 and the wrist center
    L2_WC = sqrt(X2_WC**2 + Y2_WC**2 + Z2_WC**2)

    # Find the distance between joint 3 and the wrist center
    L3_4 = 0.96     # Distance from joint 3 to joint 4
    L4_5 = 0.54     # Distance from joint 4 to joint 5 (WC)
    L3_4_x = sqrt(L3_4**2 + abs(s[a3])**2)  # X distance from joint 3 to joint 4
    phi1 = pi - atan2(abs(s[a3]), L3_4_x)
    L3_WC = sqrt(L3_4**2 + L4_5**2 - 2 * L3_4 * L4_5 * cos(phi1))

    # Determine the angle for joint 3
    cos_phi2 = (L2_WC**2 - L3_WC**2 - s[a2]**2) / (-2 * L3_WC * s[a2])
    if abs(cos_phi2) > 1:
        cos_phi2 = 1
        #print('cos_phi2 is greater than 1.')
    phi2 = atan2(sqrt(1 - cos_phi2**2), cos_phi2)
    theta3 = (pi/2 - phi2).evalf()
    theta3 = np.clip(theta3, -210*dtr, (155-90)*dtr)

    # Determine the angle for joint 2
    L2_WC_xy = sqrt(X2_WC**2 + Y2_WC**2)
    phi3 = atan2(Z2_WC, L2_WC_xy)
    cos_phi4 = (L3_WC**2 - L2_WC**2 - s[a2]**2) / (-2 * L2_WC * s[a2])
    if abs(cos_phi4) > 1:
        cos_phi4 = 1
    phi4 = atan2(sqrt(1 - cos_phi4**2), cos_phi4)
    theta2 = (pi/2 - (phi3 + phi4)).evalf()
    theta2 = np.clip(theta2, -45*dtr, 85*dtr)
    
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

    # Correction to account for orientation difference between
    #   definition of gripper link in URDF file and the DH convention.
    #   (rotation around Z axis by 180 deg and X axis by -90 deg)
    R_corr = simplify(rot_z(pi) * rot_y(-pi/2))
    
    # Calculate the symbolic rotation matrix of the spherical wrist joints
    R3_6 = R0_3.T * Rrpy * R_corr
    
    ###################################################################
    # Step 4: Calculate the spherical wrist joint angles via Euler angles

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

    ## Pitch angle; rotation around the y-axis
    #theta5 = atan2(-r31, sqrt(r11**2 + r21**2)).evalf()
    #theta5 = np.clip(theta5, -125*dtr, 125*dtr)

    #if r31 == 1:
    #    # Gimbal lock at pitch = -90
    #    theta4 = 0                          # yaw = 0
    #    theta6 = atan(-r12, -r13).evalf()   # roll
    #elif r31 == -1:
    #    # Gimbal lock at pitch = 90
    #    theta4 = 0                          # yaw = 0
    #    theta6 = atan2(r12, r13).evalf()    # roll
    #else:
    #    # General orientation

    #    # Yaw angle; rotation around the z-axis
    #    theta4 = (atan2(r21, r11)).evalf()
    #    theta4 = np.clip(theta4, -350*dtr, 350*dtr)
    #                
    #    # Roll angle; rotation around the x-axis
    #    theta6 = (atan2(r32, r33)).evalf()
    #    theta6 = np.clip(theta6, -350*dtr, 350*dtr)

    ###########################################################################
    ###########################################################################
    ## For additional debugging add your forward kinematics here. Use your
    ## previously calculated thetas as the input and output the position of
    ## your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    T3_4 = build_mod_dh_matrix(s=s, theta=theta4, alpha=alpha3, d=d4, a=a3)
    T4_5 = build_mod_dh_matrix(s=s, theta=theta5, alpha=alpha4, d=d5, a=a4)
    T5_6 = build_mod_dh_matrix(s=s, theta=theta6, alpha=alpha5, d=d6, a=a5)
    T6_G = build_mod_dh_matrix(s=s, theta=theta7, alpha=alpha6, d=d7, a=a6)

    T0_G = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G).evalf(subs={
        theta1: theta1, theta2: theta2, theta3: theta3, theta4: theta4,
        theta5: theta5, theta6: theta6, theta7: theta7})

    fk_ee_x = T0_G[0, 3]
    fk_ee_y = T0_G[1, 3]
    fk_ee_z = T0_G[2, 3]

    ## End your code input for forward kinematics here!
    ###########################################################################

    ## For error analysis please set the following variables of your WC
    ## location and EE location in the format of [x,y,z]
    
    # Load your calculated WC values in this array
    your_wc = [wc_x, wc_y, wc_z] 
    # Load your calculated EE value from your forward kinematics
    your_ee = [fk_ee_x, fk_ee_y, fk_ee_z] 

    ###########################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    #print("%0.4f - %0.4f = %0.4f" % (test_case[2][3], theta4, t_4_e))
    print ("Theta 5 error is: %04.8f" % t_5_e)
    #print("%0.4f - %0.4f = %0.4f" % (test_case[2][4], theta5, t_5_e))
    print ("Theta 6 error is: %04.8f" % t_6_e)
    #print("%0.4f - %0.4f = %0.4f" % (test_case[2][5], theta6, t_6_e))
    #print ("\n**These theta errors may not be a correct representation of \
    #       \nyour code, due to the fact that the arm can have muliple positions. \
    #       \nIt is best to add your forward kinematics to confirm whether your \
    #       \ncode is working or not**")

    # Find FK EE error
    if not(sum(your_ee)==3):
        print('')
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        #print('test: %0.4f; mine: %0.4f' % (test_case[0][0][0], your_ee[0]))
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        #print('test: %0.4f; mine: %0.4f' % (test_case[0][0][1], your_ee[1]))
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        #print('test: %0.4f; mine: %0.4f' % (test_case[0][0][2], your_ee[2]))
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("End effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
