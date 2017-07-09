# Kuka KR210 Pick and Place Project

This project uses a simulated [Kuka KR210](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/kr-210-2-f-exclusive) 6 degree of freedom manipulator to pick up a can from a shelf and drop it into a bin next to the manipulator.

The can is location at a random location on the shelf, thus inverse kinematics must be used to determine what angles each joint must be in to successfully put the gripper around the can, along with moving to the drop off bin afterwards.

A forward kinematic test file is located [here](kuka_fk.py) that builds transformation matrices for each joint. This is important for ensuring that these matrices are correct.

To test the inverse kinematic code, this [file](kuka_ik.py) uses predetermined trigonometry formulas to determine the correct joint angles from the test end-effector location.

Finally, this [file](IK_server.py) contains the code that links into the ROS/Gazebo/Rviz simulator.

### Running the Simulator

First install this [Udacity Kinematics Project](https://github.com/udacity/RoboND-Kinematics-Project) code. Then change the inverse kinematics flag to `false`.

To launch the simulator, run:

`cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts`

`./safe_spawner.sh`

You should see both Gazebo and Rviz launch. Gazebo should have a living room environment with the arm, bookshelf, bin and blue can all visible. Rviz should have the arm, bookshelf and bin visible, along with text above the scene indicating its current status.

To run the inverse kinematic code, run the following code in a new terminal:

`cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts`

`rosrun kuka_arm IK_server.py`

You must now press `Next` in the Rviz window to have each step proceed.
