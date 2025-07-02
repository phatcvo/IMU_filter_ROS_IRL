IMU EKF Filter with Butterworth Noise Filtering



Overview

This ROS package implements an Extended Kalman Filter (EKF) for estimating the orientation of an Inertial Measurement Unit (IMU) using accelerometer, gyroscope, and optional magnetometer data. The orientation is represented as a quaternion, with compensation for magnetometer hard iron biases and gravity effects on accelerometer measurements. A second-order Butterworth low-pass filter can be applied to reduce noise in the IMU's linear acceleration and angular velocity data.

The Butterworth filter can be enabled or disabled via a ROS parameter, offering flexibility for different use cases.
