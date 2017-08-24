# Robot Grasps

Robot grasps is the field of understanding how robots are able to efficiently grasps objects. This includes using unique gripper mechanisms, planning gripper grasps with deep learning, and more.

## Jammer Based Gripper

Using multi-fingered robot grippers require complicated hardware and software designs, including controlling multiple joints, force sensing, and computing how to orient and press each finger against an object. A different approach uses a single mass of granular material that, when pressed onto a target object, flows around it and conforms to its shape. Upon application of a vacuum, the granular material contracts and hardens quickly to pinch and hold the object without requiring sensor feedback.

Jamming grippers rely on friction, suction and interlocking to contribute to the mechanical strength of the jammed state.

## Using Deep Learning to Determine Grasp

It is impossible to train a traditional system to plan grasps for every object in existence. This difficulty is magnified because objects of even the same dimension can have different pose, material properties, and mass. However, deep learning can be used to train a model that learns how to best grasp an object, including those it has never seen before.

Some approaches used to train the deep learning models include using physical robots to test multiple grasps, and using simulations to test grasps.

A rubber bag was filled with smooth soda-lime glass spheres 100 um in diameter to about 80% of the bag's volume. A Venturi aspirator was used to generate pressures around 75 kPa, making the bag around 1/4 atm.

### Physical Training

*Learning Hand-Eye Coordination* used a CNN to train a model that learns how to servo a robotic gripper to poses that are likely to produce successful grasps. It continuously recomputes the most promising motor commands, integrating new sensory cues from the environment, allowing it to react to perturbations and adjust the grasp to maximize the probability of success.

This system issues motor commands in the frame of the robot, meaning that the model does not require the camera to be precisely calibrated with respect to the end-effector. Instead, it uses visual cues to determine the spatial relationship between the gripper and graspable objects in the scene.

The method consists of a grasp success predictor, which uses a deep convolutional neural network (CNN) to determine how likely a given motion is to produce a successful grasp, and a continuous servoing mechanism that uses the CNN to continuously update the robot's motor commands. By continuously choosing the best predicted path to a successful grasp, the servoing mechanism provides the robot with fast feedback to perturbations and object motion, as well as robustness to inaccurate actuation.

### Dex-Net 2.0

The *Dex-Net* (Dexterity Network) system predicts grasp success directly from depth images by training a deep Convolutional Neural Net (CNN) on a massive dataset of parallel-jaw grasps, grasp metrics, and rendered point clouds generated using analytic models of robust grasping and image formation.

The Dex-Net 2.0 research focused on planning a robust planar parallel-jaw grasp for a singulated rigid object resting on a table, based on point clouds from a depth camera.

The pipeline for training dataset generation starts with a database of 1,500 3D object mesh models. For each model, hundreds of parallel-jaw grasps are sampled across the surface, evaluating the analytic grasp metrics. For each stable pose of the object, a set of grasps that are perpendicular to the table and are collision free are flagged.

Point clouds of each object are rendered for each stable pose, with the planar object pose and camera pose sampled uniformly at random. Every grasp for a given stable pose is associated with a pixel location and orientation in the rendered image.

In total, there are over 6.7 million grasp images.

The performance of the Grasp Quality CNN (GQ-CNN) was much better than a random forest or a SVM with RBF kernel.

## Grasp Slip Detection and Correction Strategy

Determining the right grasp force on objects is difficult because the mass and strength of the object can change, along with the potential for environmental changes to alter the friction of the object (water). This method uses a force regulation strategy that is well suited for precision grasps and objects that are non-rigid, fragile, or with a changing weight.

The strategy applies a slip detection algorithm that is object and hand independent to detect slippage and then compute the required grasping forces that will eliminate this slippage.

### Slip Detection

Object slippage can be detected based on vibration and object motion. Slip detectors can use either specially designed sensors or general force/torque contact sensors. Slip sensors can also detect either gross or incipient slip.

The slip detection algorithm proposed is based on sensing the object in the context of the current grasp, rather than about the object on its own. The slip signals are sourced from multiple tactile sensors in contact with the object. The algorithms is calibrated autonomously, on-line in real time, during a grasp in order to account for object specific parameters. The slip detection is independent of the characteristics of the object being grasped, the gripper design, and the tactile sensor layout.

A DIY implementation of a slip detection algorithm would require data from tactile sensors and the ability to control the force applied to the gripper. The system would measure the slippage past the tactile sensor. If the measure exceeded a threshold, the gripper would increase it's pressure either by a fixed amount or by a proportion to how much slippage was detected. The system should calibrate itself to identify the tactile sensor's baseline when no slippage is detected. Any measure exceeding this should be flagged as slippage. However, movement and vibration in the robot arm itself might cause a false reading in the tactile sensor measurement.

### Sources

[Robotic Grasping and Contact: A Review](./papers/Robotic_Grasping_and_Contact.pdf)
[Universal robotic gripper based on the jamming of granular material](./papers/Universal_robotic_gripper_based_on_the_jamming_of_granular_material.pdf)
[Dex-Net 2.0: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics](./papers/Dex-Neet_2_0_Deep_Learning_to_Plan_Robust_Grasps.pdf)
[Learning Hand-Eye Coordination for Robotic Grasping with Large-Scale Data Collection](./papers/Learning_Hand_Eye_Coordination_for_Robotic_Grasping.pdf)
[A slip detection and correction strategy for precision robot grasping](./papers/A_slip_detection_and_correction_strategy_for_precision_robot_grasping.pdf)
