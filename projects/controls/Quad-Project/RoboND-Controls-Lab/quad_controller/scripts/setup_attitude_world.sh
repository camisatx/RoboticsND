#!/bin/bash

rosservice call /quad_rotor/gravity '{data: false}'
rosservice call /quad_rotor/x_force_constrained '{data: true}'
rosservice call /quad_rotor/y_force_constrained '{data: true}'
rosservice call /quad_rotor/z_force_constrained '{data: true}'
rosservice call /quad_rotor/x_torque_constrained '{data: false}'
rosservice call /quad_rotor/y_torque_constrained '{data: false}'
rosservice call /quad_rotor/z_torque_constrained '{data: false}'
rosservice call /quad_rotor/set_pose '{pose:{ position:{x: 0.0, y: 0.0, z: 10.0}, orientation:{x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'
