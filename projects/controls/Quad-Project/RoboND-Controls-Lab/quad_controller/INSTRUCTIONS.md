
# Hover Controller #
** Step 1: **
Add the PID class you wrote to quad_controller/src/quad_controller/pid_controller.py

** Step 2: **
$catkin_make
$ source deve/setup.bash
$ roslaunch quad_controller hover_controller.launch

** Step 3: **
Launch Unity
	-Verify you can see from within vm (rostopic echo /quad_controller/pose
Tune parameters in rqt_reconfigure until you are happy
Use Twiddle and Zn scripts if you want, or use some other method.

Bonus points: Why is it so hard to get a good tune when the quad is heading downwards?
		Can you fix this? (i,e, add mg term)

Write down the parameters you came up with, you will need them later!

# Attitude Controller #
** Step 1: **
$roslaunch quad_controller attitude_controller.launch

** Step 2: **
Tune roll and pitch PID parmaeters until things look good.
Write down the parameters you came up with, once again, you will need them later.


# Position Controller #
** Step 1: **
$roslaunch quad_controller position_controller.launch


** Step 2: ** 
Tune parameters until the controller is well-behaved

** Step 3: **
Run the Trajectory manager, feel free to use it to test your controller.

** Step 4: ** 
When you think it looks good, run the plotting tool, and share your score!
