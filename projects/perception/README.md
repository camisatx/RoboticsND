# Perception

Perception involves collecting information about the immediate environment around the robot. This can be achieved using active or passive sensors.

## Sensor Types

#### Active Sensors

Active sensors send an energy source into the environment, and measure it's response to develop a map of the environment. The energy source can include light and sound. Different sensor types use different light frequencies.

**Laser range finders (Lidar)** uses lasers pulses to detect and range objects around it (by rotating the laser and receiver).

**Time of flight cameras** user infrared light to illuminate the scene before calculating the time it took for the light to be detected by the receiver.

**Phase shift continuous wave sensors** emit a continuous stream of modulated light waves, where the distance can be determined by measuring the phase shift of the reflected light.

**Ultrasonic sensors** use high frequency sound waves to measure distance by calculating the time it takes for the sound wave to be deflected back to the sensor's receiver. These sensors can be impacted by humidity and temperature of the air the sound waves are traveling through.

#### Passive Sensors

Passive sensors receive light from the environment around them (i.e. sunlight). Depth data can be determined by capturing multiple angles of a scene. This is done by either moving the camera to a new location, or having a stereo camera which has two cameras separated by a known distance.

#### Hybrid Sensors

Hybrid sensors utilize a combination of the active and passive sensor designs. An example of this is a RGB-D sensor.

An RGB-D sensors collects RGB data from a normal camera, in addition to depth data from an infrared emitter and receiver. These sensors can save a lot of computational processing by having knowing pixel depth values directly, whereas a stereo camera would have to infer the depth from raw images.

## Camera Calibration

Camera calibration is essential for removing distortions and artifacts produced by the camera. This allows the scene to be captured accurately.

This [Udacity camera calibration notebook](https://github.com/udacity/RoboND-Camera-Calibration/blob/master/camera_calibration.ipynb) provides an overview for using OpenCV for calibrating a camera.

## Perception Pipeline

The goal of perception is to convert sensor input into a point cloud image where specific objects can be identified. The perception processing pipeline filters the point cloud to only keep essential data, removing adversarial data points and compressing the data cloud.

View the final perception pipeline [here](Exercise-1/RANSAC.py).

#### Voxel Grid Filter

The raw point cloud will often have more details than required, causing excess processing power use when analyzing it.

A voxel grid filter downsamples the data by taking a spatial average of the points in the cloud confined by each voxel. The set of points which lie within the bounds of a voxel are assigned to that voxel and are statistically combined into one output point.

#### Passthrough Filter

Allows a 3D point cloud to be cropped by specifying an axis with cut-off values along that axis. The region allowed to *pass through* is often called the *region of interest*.

#### RANSAC Plan Segmentation

Random Sample Consensus (RANSAC) is used to identify points in the dataset that belong to a particular model. It assumes that all of the data in a dataset is composed of both inliers and outliers, where inliers can be defined by a particular model with a specific set of parameters, and outliers don't thus they can be discarded.

#### Outlier Removal Filter

Outlier filters can remove outliers from the point cloud. These outliers are often caused by external factors like dust in the environment, humidity in the air, or presence of various light sources. One such filter is PCL's StatisticalOutlierRemoval filter that computes the distance to each point's neighbors, then calculates a mean distance. Any points whose mean distances are outside a defined interval are removed.

## Clustering

With a cleaned point cloud, it is possible to identify individual objects within it. One straightforward approach is to use Euclidean clustering to perform this calculation.

The two main algorithms possible include:

- K-means

- DBSCAN

#### K-means

K-means clustering algorithm is able to group data points into n groups based on their distance to randomly chosen centroids.

Here is some test [code](Exercise-2/k_means.py) for k-means.

#### DBSCAN

Density-based spatial cluster of applications with noise (DBSCAN) (sometimes called *Euclidean clustering*) is a clustering algorithm that creates clusters by grouping data points that are within some threshold distance from their nearest neighbor.

DBSCAN is unique over k-means because you don't need to know how many clusters to expect in the data. However, you do need to know something about the density of the data points being clustered.

Here is some test [code](Exercise-2/dbscan.py) covering DBSCAN.

## Perception Exercises

This exercise processes a raw RGB-D camera feed, builds a cleaned point cloud, and then segments all objects on the tabletop. 

The code [here](Exercise-2/segmentation.py) handles are steps outlined above. It interfaces with ROS, Gazebo and Rviz to run a simulation to test the performance of the code.

The original Udacity perception exercise code can be found [here](https://github.com/udacity/RoboND-Perception-Exercises). To install the exercise data on your own computer, view the installation guide [here](Install_Guide.md).
