# ASTRO 2 - Autonomous Delivery Robot
## Prototype 0.4.0
#### Stable teleop with localization

### Current Features
* Runs on ROS Kinetic
* Depedency installation scripts for `amd64` and `armhf`
* Open-loop differential drive
* Self-contained system
* Manual Xbox 360 wireless remote control
* Full URDF robot description
* Robot state publisher with limit checking
* Stable EKF robot localization
	- Fuses encoders, visual odometry, and IMU
* Basic RTABMap mapping

### Current Hardware
* MacBook Pro (i5 quad core, 16GB ram, Ubuntu 16.04)
* 2 Arduino UNOs
* 1 Arduino Nano
* 1 Intel RealSense D415
* 1 Adafruit BNO055 9-axis IMU sensor
* 2 Andy Mark PG71 motors (with built-in encoders)
* 1 Cytron 10A Dual-Channel Motor Driver
* 2 LS7366R encoder counters
* 2 40MHz crystals for the encoder counters
* 1 12V 18Ah lead-acid battery
* 1 12-24V to 5V power regulator

Stable teleop system with localization, next step is navigation.
