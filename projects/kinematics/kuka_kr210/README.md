[//]: # (Image References)
[dh_diagram]: ./misc/dh_diagram.jpg
[trigonometric_diagram]: ./misc/trigonometric_diagram.jpg

# Kuka KR210 Pick and Place Project
---

This project uses a simulated [Kuka KR210](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/kr-210-2-f-exclusive) 6 degree of freedom manipulator to pick up a can from a shelf and drop it into a bin next to the manipulator.

The can is location at a random location on the shelf, thus inverse kinematics must be used to determine what angles each joint must use to successfully put the gripper around the can, and to move the arm to the drop off bin afterwards.

### Contents

- [Denavit-Hartenberg Diagram](#denavit-hartenberg-diagram)
- [Denavit-Hartenberg Parameters](#denavit-hartenberg-parameters)
- [Joint Based Transformation Matrices](#joint-based-transformation-matrices)
- [Base to Gripper Homogeneous Transformation Matrix](#base-to-gripper-homogeneous-transformation-matrix)
- [Inverse Kinematic Calculations](#inverse-kinematic-calculations)
    - [Inverse Kinematic Position](#inverse-kinematic-position)
    - [Inverse Kinematic Orientation](#inverse-kinematic-orientation)
- [Test Code](#test-code)

## Denavit-Hartenberg Diagram

Here is a Denavit-Hartenberg (DH) diagram of the arm:

![Denavit-Hartenberg diagram of the Kuka KR210 6 DoF arm][dh_diagram]

The arm contains six revolute joints connected to each other in a linear fashion.

## Denavit-Hartenberg Parameters

Based on the arm's specifications, I was able to derive the following parameters that were also used within the DH diagram. Note that these values relate to modified DH parameters.

| n |  θ |   d   |    a   |  α  |
|:-:|:--:|:-----:|:------:|:---:|
| 0 |  - |   -   |    0   |  0  |
| 1 | θ1 |  0.75 |  0.35  | -90 |
| 2 | θ2 |   0   |  1.25  |  0  |
| 3 | θ3 |   0   | -0.054 | -90 |
| 4 | θ4 |  1.5  |    0   |  90 |
| 5 | θ5 |   0   |    0   | -90 |
| 6 | θ6 | 0.303 |    0   |  0  |

The a and α parameters do not change from orientation to orientation because they are specific to the arm. However, the θ and d parameters can change depending on the orientation. For this arm, only the θ parameters will change since all the joints are revolute.

## Joint Based Transformation Matrices

All of the joints have their own transformation matrix that describes their position and orientation relative to prior joints.

The transformation matrix can be calculated by substituting the DH parameters from the table above into this matrix:

```
TM = [[        cos(θ),       -sin(θ),       0,         a],
      [ sin(θ)*cos(α), cos(θ)*cos(α), -sin(α), -sin(α)*d],
      [ sin(θ)*sin(α), cos(θ)*sin(α),  cos(α),  cos(α)*d],
      [             0,             0,       0,         1]]
```

Using the transformation matrix formula above, here are the joint transformation matrices for the arm:

```
Joint 1: [[ cos(θ1), -sin(θ1),  0,     0],
          [ sin(θ1),  cos(θ1),  0,     0],
          [       0,        0,  1,  0.75],
          [       0,        0,  0,     1]]
```

```
Joint 2: [[ sin(θ2),  cos(θ2),  0,  0.35],
          [       0,        0,  1,     0],
          [ cos(θ2), -sin(θ2),  0,     0],
          [       0,        0,  0,     1]]
```

```
Joint 3: [[ cos(θ3), -sin(θ3),  0,  1.25],
          [ sin(θ3),  cos(θ3),  0,     0],
          [       0,        0,  1,     0],
          [       0,        0,  0,     1]]
```

```
Joint 4: [[ cos(θ4), -sin(θ4),  0, -0.054],
          [       0,        0,  1,    1.5],
          [-sin(θ4), -cos(θ4),  0,      0],
          [       0,        0,  0,      1]]
```

```
Joint 5: [[ cos(θ5), -sin(θ5),  0,      0],
          [       0,        0, -1,      0],
          [ sin(θ5),  cos(θ5),  0,      0],
          [       0,        0,  0,      1]]
```

```
Joint 6: [[ cos(θ6), -sin(θ6),  0,      0],
          [       0,        0,  1,      0],
          [-sin(θ6), -cos(θ6),  0,      0],
          [       0,        0,  0,      1]]
```

Finally, here is the transformation matrix for the gripper. It does not have a rotation, but it does have an offset in the Z direction (out forward).

```
Gripper: [[       1,        0,  0,      0],
          [       0,        1,  0,      0],
          [       0,        0,  1,  0.303],
          [       0,        0,  0,      1]]
```

## Base to Gripper Homogeneous Transformation Matrix

The individual joint transformation matrices can be combined into a single homogeneous transformation matrix that describes the position and orientation to get from the base of the arm to the arm's gripper.

```
Base to Gripper:

[[((sin(theta1)*sin(theta4) + sin(theta2 + theta3)*cos(theta1)*cos(theta4))*cos(theta5) + sin(theta5)*cos(theta1)*cos(theta2 + theta3))*cos(theta6) + (sin(theta1)*cos(theta4) - sin(theta4)*sin(theta2 + theta3)*cos(theta1))*sin(theta6), -((sin(theta1)*sin(theta4) + sin(theta2 + theta3)*cos(theta1)*cos(theta4))*cos(theta5) + sin(theta5)*cos(theta1)*cos(theta2 + theta3))*sin(theta6) + (sin(theta1)*cos(theta4) - sin(theta4)*sin(theta2 + theta3)*cos(theta1))*cos(theta6), -(sin(theta1)*sin(theta4) + sin(theta2 + theta3)*cos(theta1)*cos(theta4))*sin(theta5) + cos(theta1)*cos(theta5)*cos(theta2 + theta3), -0.303*(sin(theta1)*sin(theta4) + sin(theta2 + theta3)*cos(theta1)*cos(theta4))*sin(theta5) + (1.25*sin(theta2) - 0.054*sin(theta2 + theta3) + 1.5*cos(theta2 + theta3) + 0.35)*cos(theta1) + 0.303*cos(theta1)*cos(theta5)*cos(theta2 + theta3)],
 [((sin(theta1)*sin(theta2 + theta3)*cos(theta4) - sin(theta4)*cos(theta1))*cos(theta5) + sin(theta1)*sin(theta5)*cos(theta2 + theta3))*cos(theta6) - (sin(theta1)*sin(theta4)*sin(theta2 + theta3) + cos(theta1)*cos(theta4))*sin(theta6), -((sin(theta1)*sin(theta2 + theta3)*cos(theta4) - sin(theta4)*cos(theta1))*cos(theta5) + sin(theta1)*sin(theta5)*cos(theta2 + theta3))*sin(theta6) - (sin(theta1)*sin(theta4)*sin(theta2 + theta3) + cos(theta1)*cos(theta4))*cos(theta6), -(sin(theta1)*sin(theta2 + theta3)*cos(theta4) - sin(theta4)*cos(theta1))*sin(theta5) + sin(theta1)*cos(theta5)*cos(theta2 + theta3), -0.303*(sin(theta1)*sin(theta2 + theta3)*cos(theta4) - sin(theta4)*cos(theta1))*sin(theta5) + (1.25*sin(theta2) - 0.054*sin(theta2 + theta3) + 1.5*cos(theta2 + theta3) + 0.35)*sin(theta1) + 0.303*sin(theta1)*cos(theta5)*cos(theta2 + theta3)],
 [                                                                                           -(sin(theta5)*sin(theta2 + theta3) - cos(theta4)*cos(theta5)*cos(theta2 + theta3))*cos(theta6) - sin(theta4)*sin(theta6)*cos(theta2 + theta3),                                                                                              (sin(theta5)*sin(theta2 + theta3) - cos(theta4)*cos(theta5)*cos(theta2 + theta3))*sin(theta6) - sin(theta4)*cos(theta6)*cos(theta2 + theta3),                                                     -sin(theta5)*cos(theta4)*cos(theta2 + theta3) - sin(theta2 + theta3)*cos(theta5),                                                                   -0.303*sin(theta5)*cos(theta4)*cos(theta2 + theta3) - 0.303*sin(theta2 + theta3)*cos(theta5) - 1.5*sin(theta2 + theta3) + 1.25*cos(theta2) - 0.054*cos(theta2 + theta3) + 0.75],
 [                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                         0,                                                                                                                                    0,                                                                                                                                                                                                                                                1]]
```

Substituting thetas of zero into this equation yields:

```
T_total = [[ 1.0,   0,   0, 2.153],
           [   0, 1.0,   0,     0],
           [   0,   0, 1.0, 1.946],
           [   0,   0,   0,     1]]
```

Which indicates that the with neutral joint angles, the gripper is at location 2.153 in the x-axis, 0 in the y-axis, and 1.946 in the z-axis.

This matrix includes a correction to account for the orientation difference between the gripper and the arm's base. This correction required rotating the gripper's coordinate frame around the Z axis by 180 degrees and around the Y axis by -90 degrees.

## Inverse Kinematic Calculations

Inverse kinematics were used to calculate the joint angles required to move the arm's gripper to the exact location required. This problem was broken into two parts: the position calculation and the orientation calculation.

The final three joints are known as a `wrist center`, which is able to rotate the end effector to the exact **orientation** required for the task at hand. The first three joints move the wrist center to a **position** where it has enough mobility to orient itself to the angle required.

### Inverse Kinematic Position

The inverse kinematic position calculations are only concerned with the first three joints. Here are the 2-d diagrams and trigonometric calculations I used to derive these joint angles.

![Inverse kinematic position calculation diagram][trigonometric_diagram]

### Inverse Kinematic Orientation

To derive the inverse kinematic orientation, I calculate the rotation matrix of joints 4, 5 and 6 (the spherical wrist), then calculate the Euler angles from this rotation matrix.

I calculate the spherical wrist's rotation matrix by using this formula:

```python
R3_6 = R0_3.T * Rrpy * R_corr
```

Where:
- `R0_3.T` is the transposition of the rotation matrix from joints 1, 2 and 3, requiring the angles derived in the inverse kinematic position calculations
- `Rrpy` is the rotation matrix of the gripper's current roll, pitch and yaw
- `R_corr` is the rotation matrix of the gripper correction matrix that rotates the gripper around the Z axis by 180 degrees and around the Y axis by -90 degrees

With this rotation matrix, it is possible to derive the Euler angles.

#### Euler Angles

I used a [tf transformations](http://www.lfd.uci.edu/~gohlke/code/transformations.py.html) function called `euler_from_matrix` that takes in a numpy rotation matrix and Euler axis sequence, and returns the three Euler angles (alpha, beta and gamma).

The rotation matrix I provided used the Euler definition of `XYZ`, which is a Tait-Bryan angle combination. With the alpha, beta and gamma angles, I mapped them to theta 4, theta 5 and theta 6, respectively.

However, theta 4 and theta 5 required these additional calculations:

```python
theta4 = np.pi/2 + theta4
theta5 = np.pi/2 - theta5
```

## Test Code

The inverse kinematic code can be tested with the [IK_debug.py](IK_debug.py) file. It has the ability to test the code against multiple positions and orientations, printing out the error rates for the joint angles and the wrist center location.

Finally, the [IK_server.py](IK_server.py) file contains the code that links into the ROS/Gazebo/Rviz simulator.

### Running the Simulator

To run the [IK_server.py](IK_server.py) code, install this [Udacity Kinematics Project](https://github.com/udacity/RoboND-Kinematics-Project) code. Then change the inverse kinematics flag to `false`.

To launch the simulator, run:

```bash
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
./safe_spawner.sh
```

You should see both Gazebo and Rviz launch. Gazebo should have a living room environment with the arm, bookshelf, bin and blue can all visible. Rviz should have the arm, bookshelf and bin visible, along with text above the scene indicating its current status.

To run the inverse kinematic code, run the following code in a new terminal:

```bash
cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
rosrun kuka_arm IK_server.py
```

You must now press `Next` in the Rviz window to have each step proceed. If everything worked, the arm should eventually move to pick up the can, grab it, then move so the arm will drop the can into the bin on the ground.
