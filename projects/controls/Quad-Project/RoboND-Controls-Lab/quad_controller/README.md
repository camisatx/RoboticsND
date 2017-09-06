# Quad Controller Project #

### Launching the Simulator ###
In order to launch the simulator, simply run the simulator binary.

**Note:**
You must first run `roscore`. If you don't run `roscore`, the
simulator will still run, but you will not be able to communicate
with it via ROS.

### The hover controller node ###
Perhaps the simplest controller in the `quad_controller` package is the hover
controller node. It is nothing more than a simple PID controller that sets
applies a vertical thrust vector to the quad rotor body.

#### Running the hover controller node ####

```sh
$ rosrun quad_controller hover_controller_node

```
**Optional Step:**
You can set up camera angles and motion constrants to make it
make visualization and debugging of the hover conteroller
simple. To do, simply run `setup_2d_hover_world.sh`.

```sh
$ rosrun quad_controller setup_2d_hover_world.sh
```

**Hover controller parameter tuning:**
The easiest way to tune the hover controller's PID parmaeters
or set the target position is to use ROS' dynamic reconfigure.

To run dynamic reconfigure:
```sh
$ rqt_reconfigure
```

If `hover_controller_node` is running you should now be able
to interactively explore the hover controller.

**Plotting Altitude**
One simple way to plot the altitude is to use `rqt_plot`. 

```sh
$ rqt_plot /quad_rotor/pose/pose/position/z
```
