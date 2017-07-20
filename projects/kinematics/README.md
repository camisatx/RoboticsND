[//]: # (Image Reference)
[kuka_pick_and_place]: ./kuka_kr210/misc/kuka_pick_and_place.png

# Kinematics

Kinematics is a branch of classical mechanics that describes the motion of points, bodies (objects), and systems of bodies (groups of objects) without considering the mass of each or the forces that caused the motion ([Wikipedia](https://en.wikipedia.org/wiki/Kinematics)). In robotics, kinematics is mostly used to describe the motion of robotic arms.

This section focuses on kinematic basics, including the use of rotation matrices, displacement vectors and homogeneous transformations. [Denavit-Hartenberg](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters) parameters are also covered, which are useful when determining the kinematic aspects of robotic manipulators.

## Topics

### Sympy

[Sympy](http://www.sympy.org/en/index.html) is a Python library for symbolic mathematics.

[Example 1](sympy_test.py)

### Rotation Matrices

[Example 1](rotation_1.py)

[Example 2](rotation_2.py)

### Euler Angles

[Euler angles from a matrix](euler_angles_from_matrix.py)

### Homogeneous Transformations

[Example 1](homogeneous_transformation_1.py)

[Example 2](homogeneous_transformation_2.py)

### Forward Kinematics

[Example 1](forward_kinematics.py)

[2 DoF Arm](dof_2_arm_config.py)

## Kuka Pick and Place Project

![Kuka KR210 Pick and Place Project][kuka_pick_and_place]

The Kuka Pick and Place project used kinematics to control a simulated Kuka KR210 6 DoF manipulator to pick up a can from a shelf and drop it into a bin next to the arm.

This task involved calculating the forward and inverse kinematics of the arm, so that the gripper could be placed in the right position and orientation for grabbing the can.

View the write-up [here](./kuka_kr210/README.md).
