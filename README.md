IMU EKF Filter with Butterworth Noise Filtering

Overview
This ROS package implements an Extended Kalman Filter (EKF) for estimating the orientation of an Inertial Measurement Unit (IMU) using accelerometer, gyroscope, and optional magnetometer data. The orientation is represented as a quaternion, with compensation for magnetometer hard iron biases and gravity effects on accelerometer measurements. A second-order Butterworth low-pass filter can be applied to reduce noise in the IMU's linear acceleration and angular velocity data, with configurable cutoff (fc) and sampling (fs) frequencies.
Designed for applications like agricultural autonomous vehicles, this package ensures robust orientation estimation in noisy environments. The Butterworth filter can be enabled or disabled via a ROS parameter, offering flexibility for different use cases.
Features

EKF Orientation Estimation: Fuses accelerometer, gyroscope, and optional magnetometer data to compute quaternion-based orientation.
Gravity Compensation: Publishes gravity-compensated acceleration and estimated gravity vector.
Magnetometer Support: Optional integration with hard iron bias compensation.
Butterworth Filter: Applies a second-order low-pass filter to IMU data when enabled, with adjustable fc and fs.
ROS Integration: Subscribes to IMU and magnetometer topics, publishes filtered orientation, compensated acceleration, and gravity estimates.
Configurable Parameters: Loads noise covariances, magnetic references, and filter settings from a YAML file.

Project Structure
imu_ekf_filter/
├── include/
│   ├── ekf_imu.h          # EKF class and function declarations
│   └── butterworth.h      # Butterworth filter class declaration
├── src/
│   ├── ekf_imu_node.cpp   # EKF node implementation
│   └── butterworth.cpp    # Butterworth filter implementation
├── config/
│   └── param.yaml         # Parameter configuration file
├── launch/
│   └── ekf_imu.launch     # Example launch file
├── CMakeLists.txt         # Build configuration
├── package.xml            # Package manifest
└── README.md              # This file

Prerequisites

ROS: Compatible with ROS Noetic or Melodic (adjust for your distribution).
Dependencies:
roscpp
sensor_msgs
message_filters
eigen (install with sudo apt-get install libeigen3-dev on Ubuntu)


Catkin Workspace: A configured ROS catkin workspace.
Git: For cloning the repository.

Installation

Clone the Repository:
cd ~/catkin_ws/src
git clone https://github.com/your_username/imu_ekf_filter.git


Key Parameters

noise_filter (bool): Enable (true) or disable (false) the Butterworth filter.
fc (float): Cutoff frequency for the Butterworth filter (Hz, default: 12.0).
fs (float): Sampling frequency for the Butterworth filter (Hz, default: 100.0, ensure fs > 2*fc).
acc_noise (float): Accelerometer noise covariance (m²/s⁴, default: 0.25).
mag_noise (float): Magnetometer noise covariance (µT², default: 0.01).
gyro_noise (float): Gyroscope noise covariance (rad²/s², default: 0.09).
coordinate_frame (string): "NED" or "ENU" for gravity and magnetic field references.
imu_topic, mag_topic (string): Input topics for IMU and magnetometer data.

Edit config/param.yaml to customize these values.
Running the Node

Create a Launch File:Create launch/ekf_imu.launch in your package:
<launch>
    <node pkg="imu_ekf_filter" type="ekf_imu_node" name="ekf_imu_node" output="screen">
        <rosparam file="$(find imu_ekf_filter)/config/param.yaml" command="load" ns="ekf_imu_node"/>
    </node>
</launch>


Run the Node:Ensure IMU and magnetometer data are published on the configured topics (/imu/data_newIMU and /mag_newIMU by default).
roscore
roslaunch imu_ekf_filter ekf_imu.launch


Verify Output:The node publishes:

/imu/data_ekf: Filtered orientation (quaternion) and IMU data.
/imu/accel_compensated: Gravity-compensated acceleration.
/imu/gravity_ekf: Estimated gravity vector.Check topics with:

rostopic echo /imu/data_ekf


Debugging:

Parameter Issues: If warnings like [WARN] Failed to load noise_filter appear, verify param.yaml is loaded correctly and parameters exist in the /ekf_imu_node namespace:rosparam list | grep /ekf_imu_node
rosparam get /ekf_imu_node/noise_filter


IMU Data: Ensure accelerometer norm is 9.81 m/s² and magnetometer norm matches the reference (49.47 µT for defaults).
Filter Tuning: Adjust fc and fs in param.yaml to match your IMU data rate and noise characteristics.



Troubleshooting

Parameter Loading Failures:
Check param.yaml path and namespace in the launch file.
Use rosparam set /ekf_imu_node/noise_filter true to test parameter loading manually.


Invalid IMU Data:
If the node skips updates due to invalid accelerometer or magnetometer norms, verify sensor calibration and data topics.


Butterworth Filter:
Ensure fs > 2*fc to avoid aliasing.
Set noise_filter: false to bypass filtering for debugging.


Magnetometer Issues:
If magnetometer data is unreliable, set use_magnetometer: false in param.yaml.



Contributing
Contributions are welcome! To contribute:

Fork the repository.
Create a feature branch (git checkout -b feature/your-feature).
Commit changes (git commit -m "Add your feature").
Push to the branch (git push origin feature/your-feature).
Open a pull request.

Please report issues or feature requests via the GitHub Issues page.
License
This project is licensed under the MIT License. See the LICENSE file for details.
Contact
For questions or support, contact Hoang Quoc Hung via GitHub or open an issue in the repository.
Acknowledgments

Developed as part of the IRL Agricultural Autonomous Vehicle project.
Butterworth filter implementation inspired by tttapa.github.io.
