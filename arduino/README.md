# ASTRO 2 Arduino Sketches
### Prototype v0.3 (Large Robot)

These Arduino programs implement details that are specific to our hardware.

## Hardware

* Two Arduino Unos
* Two PG71 Andy Mark DC motors
* A MDD10A Rev2.0 Dual Channel Motor Driver
* Two LM7366 encoder counters
* Two 40MHz quartz crystals (for the LM7366 counters)

One Arduino is responsible for `driver` (*writing* to motors) and the other is responsible for `encoder` (*reading* from motors).

## Code

`driver` - Write to motors

* Arduino UNO
* `driver.ino` subscribes to `cmd_vel/left` and `cmd_vel/right` (from `astro_diff_drive`) and converts the `sensor_msgs::Float32` commands into PWM signals using an experimentally-determined equation. These PWM signals are sent to the motor driver to actuate the motors.
* `Motor.h` defines the `Motor` class that abstracts some of the motor communications.
* `driver.h` simply refactors all the pin defs.


`encoders` - Read from motors

* Arduino UNO
* `encoders.ino` implements open-source code for the LM7366 encoder counters and modifies it to publish the joint states of each wheel to be collected by `astro_state_publisher`.

`encoders_alternate` - Read from motors

* Arduino UNO
* Does the same as `encoders` but also publishes encoder ticks to `lwheel` and `rwheel`. This is no longer used since the `diff_tf` node is no longer used by `astro_diff_drive`  (in favor of a simpler method requiring only joint states).


`imu` - Publish raw IMU data

* Arduino Nano
* We cannot publish all the IMU data in its standardized form (`sensor_msgs/Imu`, `sensor_msgs/MagneticField`, covariance, etc) without massively exceeding the memory capacity of the Nano (151% of memory).
* This node simply collects all the raw data into a single `Float32MultiArray` message to be re-published in its proper form (with covariance matrices) by `astro_imu_publisher`.
* With these optimizations, it was possible to bring the memory usage down to 89% stable.
* Publishes to `imu/raw`

`imu_minified` - Publish raw IMU data

* Arduino Nano
* Publishes only what is necessary for `robot_localization` to do its magic (i.e. one complete `Imu` message)
* Published temperature... just to have
* Although it takes the same amount of dynamic memory, it runs much faster.
* Publishes to `imu/raw`


`test`

* Manually read and write to the motors without depending on ROS. This is how we can run velocity/pwm tests, ensure electronics are working, etc.
* Generate calibration data for IMU
* Test IMU functionality (also used to measure covariances)