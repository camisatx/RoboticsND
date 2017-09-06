#!/bin/bash

rosservice call /quad_rotor/camera_pose_type '{data: 1}'
rosservice call /quad_rotor/gravity '{data: true}'
rosservice call /quad_rotor/x_force_constrained '{data: true}'
rosservice call /quad_rotor/y_force_constrained '{data: true}'
rosservice call /quad_rotor/x_torque_constrained '{data: true}'
rosservice call /quad_rotor/y_torque_constrained '{data: true}'
rosservice call /quad_rotor/z_torque_constrained '{data: true}'
