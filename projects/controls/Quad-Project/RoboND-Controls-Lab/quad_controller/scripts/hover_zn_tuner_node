#!/usr/bin/env python
import dynamic_reconfigure.client
import matplotlib.pyplot as plt
import numpy as np
import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from quad_controller.srv import SetPose
from quad_controller.srv import SetInt
from std_srvs.srv import SetBool

def plotRun(samples,  title, axis_labels):
    t_init = samples[0][1].to_sec()
    sample_times = [ sample[1].to_sec() - t_init for sample in samples ]
    sample_values = [ sample[0] for sample in samples ]

    fig = plt.figure(figsize=(12,3))
    plt.subplot(111)
    plt.title(title)
    plt.xlabel(axis_labels[0])
    plt.ylabel(axis_labels[1])
    plt.plot(sample_times, sample_values)
    plt.plot((sample_times[0], sample_times[-1]),
                (10, 10), 'k--')
    plt.show()

def reset_pose_and_dynamics():
    try:
        # Reset the forces and velocities on the quad
        reset_force_vel = rospy.ServiceProxy('/quad_rotor/reset_orientation', SetBool)
        reset_force_vel(True)
        
        # Call service to set position
        set_position = rospy.ServiceProxy('/quad_rotor/set_pose', SetPose)
        initial_pose = Pose()
        initial_pose.position.z = 0
        response = set_position(initial_pose)
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: {}'.format(e))


def set_kp(kp):
    dr_client.update_configuration({'kp': kp})

def set_target(target):
    dr_client.update_configuration({'target': target})

def dr_callback(config):
    kp = config.kp
    ki = config.ki
    kd = config.kd
    target = config.target
    rospy.loginfo('config set: target:{}, kp:{}, ki:{}, kd:{}'.format(target, kp, ki, kd))

def pose_callback(pose):
    global current_altitude
    global current_time
    global samples
    current_altitude = pose.pose.position.z
    current_time = pose.header.stamp
    samples.append([pose.pose.position.z, pose.header.stamp])


def setup_experiment():
    rospy.wait_for_service('/quad_rotor/reset_orientation')
    rospy.wait_for_service('/quad_rotor/set_pose')
    rospy.wait_for_service('/quad_rotor/x_force_constrained')
    rospy.wait_for_service('/quad_rotor/y_force_constrained')
    rospy.wait_for_service('/quad_rotor/x_torque_constrained')
    rospy.wait_for_service('/quad_rotor/y_torque_constrained')
    rospy.wait_for_service('/quad_rotor/z_torque_constrained')
    rospy.wait_for_service('/quad_rotor/camera_pose_type')

    try:
        # Reset the forces and velocities on the quad
        reset_force_vel = rospy.ServiceProxy('/quad_rotor/reset_orientation', SetBool)
        reset_force_vel(True)
        
        # Call service to set position
        set_position = rospy.ServiceProxy('/quad_rotor/set_pose', SetPose)
        initial_pose = Pose()
        initial_pose.position.z = 0
        response = set_position(initial_pose)

        # Call service to constrain translations. We only want to be able to translate in Z
        x_force_constrained = rospy.ServiceProxy('/quad_rotor/x_force_constrained', SetBool)
        y_force_constrained = rospy.ServiceProxy('/quad_rotor/y_force_constrained', SetBool)
        x_force_constrained(True)
        y_force_constrained(True)

        # Call service to constrian rotations, we don't want to rotate at All
        x_torque_constrained = rospy.ServiceProxy('/quad_rotor/x_torque_constrained', SetBool)
        y_torque_constrained = rospy.ServiceProxy('/quad_rotor/y_torque_constrained', SetBool)
        z_torque_constrained = rospy.ServiceProxy('/quad_rotor/z_torque_constrained', SetBool)
        x_torque_constrained(True) 
        y_torque_constrained(True) 
        z_torque_constrained(True) 

        #Set camera pose to be aligned to X-axis (1)
        camera_pose_type = rospy.ServiceProxy('/quad_rotor/camera_pose_type', SetInt)
        camera_pose_type(1)
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: {}'.format(e))

def zn_tune(max_duration, kmin, kmax, kstep, rule='classic'):
    global samples
    tune_rate_hz = 10.0
    tune_rate = rospy.Rate(tune_rate_hz)
    
    for kp in np.linspace(kmin, kmax, (kmax-kmin)/kstep):
        set_kp(kp)
        reset_pose_and_dynamics()

        #Clear samples for each test run
        samples = []

        oscillation_count = 0
        oscillation_times = []
        initial_error = 0
        
        for i in range(0, int(max_duration*tune_rate_hz)):
            tune_rate.sleep()
            if i is 0:
                initial_error = target - current_altitude
            new_error = target - current_altitude

            #Check for oscillsation
            if (cmp(initial_error, 0) != cmp(new_error, 0)) & (cmp(new_error, 0) != 0) &  (cmp(initial_error, 0) != 0):
                rospy.loginfo('Oscillation Detected! ({})'.format(oscillation_count))
                oscillation_count += 1
                oscillation_times.append(current_time)
                initial_error = new_error

                # One period is 3 oscillations
                if oscillation_count is 3:
                    # Calculate tu
                    tu = oscillation_times[2] - oscillation_times[0]
                    rospy.loginfo('Oscillation Period (Tu): {}'.format(tu.to_sec()))

                    # Check to make sure oscillation is significant
                    if tu.to_sec() > 3.0 * 1.0/tune_rate_hz:

                        if rule == 'classic':
                            kp_zn = 0.6*kp
                            ki_zn = 1.2*kp/tu.to_sec()
                            kd_zn = 0.075*kp*tu.to_sec()

                        if rule == 'no-overshoot':
                            kp_zn = 0.2*kp
                            ki_zn = 0.4*kp/tu.to_sec()
                            kd_zn = 0.2*kp*tu.to_sec()/3.0
                        
                        if rule == 'some-overshoot':
                            kp_zn = 0.33*kp
                            ki_zn = 0.66*kp/tu.to_sec()
                            kd_zn = 0.1089*kp*tu.to_sec()
                        
                        rospy.loginfo('Computed Params! kp:{}, ki:{}, kd:{}'.format(kp_zn, ki_zn, kd_zn))
                        return (kp_zn, ki_zn, kd_zn)
                    else:
                        #Try a larger Kp
                        break
    rospy.loginfo('Found no oscillations in search range range ({}-{})'.format(kmin,kmax))
    return (0, 0, 0)

if __name__ == '__main__':
    kp = 0
    ki = 0
    kd = 0
    target = 10
    current_altitude = 0
    current_time = 0
    samples = []

    rospy.init_node('hover_zn_tuner')
    dr_client = dynamic_reconfigure.client.Client('/hover_controller', config_callback=dr_callback)
    pose_sub_ = rospy.Subscriber("/quad_rotor/pose", PoseStamped, pose_callback)

    setup_experiment()
    set_target(target)
    (kp_zn, ki_zn, kd_zn) = zn_tune(10.0, 5, 10 , 1, rule='no-overshoot')
    dr_client.update_configuration({'kp': kp_zn, 'ki': ki_zn, 'kd':kd_zn})
    plotRun(samples,'ZN Tuning Run: no-overshoot', ['Time','Altitude'])

    reset_pose_and_dynamics()
    samples = []
    set_target(10)
    rospy.sleep(10)
    plotRun(samples,'ZN Test Run: no-overshoot ',['Time','Altitude'])

    rospy.signal_shutdown('finished')