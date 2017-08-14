# Sensor Types

## Active Sensors

Active sensors send an energy source into the environment, and measure it's response to develop a map of the environment. The energy source can include light and sound. Different sensor types use different light frequencies.

**Laser range finders (Lidar)** uses lasers pulses to detect and range objects around it (by rotating the laser and receiver).

**Time of flight cameras** user infrared light to illuminate the scene before calculating the time it took for the light to be detected by the receiver.

**Phase shift continuous wave sensors** emit a continuous stream of modulated light waves, where the distance can be determined by measuring the phase shift of the reflected light.

**Ultrasonic sensors** use high frequency sound waves to measure distance by calculating the time it takes for the sound wave to be deflected back to the sensor's receiver. These sensors can be impacted by humidity and temperature of the air the sound waves are traveling through.

## Passive Sensors

Passive sensors receive light from the environment around them (i.e. sunlight). Depth data can be determined by capturing multiple angles of a scene. This is done by either moving the camera to a new location, or having a stereo camera which has two cameras separated by a known distance.

## Hybrid Sensors

Hybrid sensors utilize a combination of the active and passive sensor designs. An example of this is a RGB-D sensor.

An RGB-D sensors collects RGB data from a normal camera, in addition to depth data from an infrared emitter and receiver. These sensors can save a lot of computational processing by having knowing pixel depth values directly, whereas a stereo camera would have to infer the depth from raw images.

# Camera Calibration

Camera calibration is essential for removing distortions and artifacts produced by the camera. This allows the scene to be captured accurately.

This [Udacity camera calibration notebook](https://github.com/udacity/RoboND-Camera-Calibration/blob/master/camera_calibration.ipynb) provides an overview for using OpenCV for calibrating a camera.
