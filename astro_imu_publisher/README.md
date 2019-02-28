# ASTRO 2 IMU Publisher

This package was created due to limitations in the Arduino Nano in publishing all the necessary IMU data to the ROS network. It simply takes raw data in a primitive (non-standard) form from a BNO055 IMU device connected to an Arduino node, and publishes it in a standard (REPL) form with added covariance matrices.

## Subscribed Topics
`imu/raw` ([std_msgs/Float32MultiArray](http://docs.ros.org/jade/api/std_msgs/html/msg/Float32MultiArray.html))  
    Raw data stream from Arduino Nano `imu` node. Arranged as follows:
```
    [ quat_x, quat_y, quat_z, quat_w, 
  	  euler_x, euler_y, euler_z, 
 	  accel_x, accel_y, accel_z, 
  	  gyro_x, gyro_y, gyro_z, 
  	  sys_cal, accel_cal, gyro_cal, mag_cal, 
  	  temperature ]
```

## Published Topics
*Note: frame_ids are currently hard-coded to 'imu_link'*
`imu/data` ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))  
	Contains quaternion, linear acceleration, and angular velocity data along with covariance matrices.  

`imu/pose` ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))  
	Contains Euler angle representation (and quaternion). Published for human-readability of IMU orientation.  

`imu/mag` ([sensor_msgs/MagneticField](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/MagneticField.html))  
	Magnetometer output from the IMU sensor.

`imu/temp` ([sensor_msgs/Temperature](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Temperature.html))  
	Temperature output from the IMU sensor.

`imu/status` ([diagnostic_msgs/DiagnosticArray](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html))  
	Array of 4 [DiagnosticStatus](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html) messages. Each one shows the calibration status for a particular component (system, accelerometer, gyroscope, and magnetometer). 3 indicates fully calibrated and 0 indicates not calibrated. Note that some IMU sensors are supposed to auto-calibrate as they run (like the BNO055).

## Parameters

`~rate` (`float`)  
	Spin rate. Defaults to 10.

`~orientation_covariance` (`float[9]`)  
	Covariance matrix to accompany `sensor_msgs/Imu` message. Defaults to zeroes.

`~linear_acceleration_covariance` (`float[9]`)  
	Covariance matrix to accompany `sensor_msgs/Imu` message. Defaults to zeroes.

`~angular_velocity_covariance` (`float[9]`)  
	Covariance matrix to accompany `sensor_msgs/Imu` message. Defaults to zeroes.

`~magnetic_field_covariance` (`float[9]`)  
	Covariance matrix to accompany `sensor_msgs/MagneticField` message. Defaults to zeroes.

`~temperature_covariance` (`float`)  
	Covariance matrix to accompany `sensor_msgs/Temperature` message. Defaults to zero.


## Covariance Matrices

Here are some details on how I obtained covariance matrices for the BNO055. **Please note that I am no expert, and this might be totally wrong. There isn't much info about it, so I did what I thought made sense**.

All covariance matrices used are diagonal matrices.

### Orientation Covariance

Calculated with the following procedure:
1. Place the the imu in a specific pose and read the Euler angles.
2. Move it around, then place it back exactly in its original place
3. Read the values again.
4. Repeat steps 1-3 nine more times for the same pose.
5. Repeat 1-4 two more times with very different positions.
6. There are now 3 data sets. Calculate the variance in the yaw, pitch, and roll angles. Since for a single data set they are supposed to represent the same position, the difference in measurements is error.
7. As a conservative estimate, take the max variance of each.
8. We approximate the angles as independent, so we fill our covariance matrix with yaw -> xx, pitch -> yy, roll -> zz.

`diagonal values: [0.020354665, 5.72348e-05, 0.000119784]`

### Linear Acceleration Covariance

Calculated from "output noise density" (from data sheet) at 100Hz, converted to m/s, and converted to variance (square it).

`diagonal values: [0.000385, 0.000385, 0.000385]`


### Angular Velocity Covariance

Calculated from "output noise" (0.3 deg/s), converted to rad/s, and finally converted to variance (square it).

`diagonal values: [0.00002742, 0.00002742, 0.00002742]`

### Magnetic Field Covariance

Calculated from "device resolution" (0.3 uT), converted to T, and finally converted to variance (square it).

`diagonal values: [9.0e-8, 9.0e-8, 9.0e-8]`
