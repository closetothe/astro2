# Astro 2 Arduino Sketches
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

`driver` - READ

* `driver.ino` subscribes to `cmd_vel/left` and `cmd_vel/right` (from `astro_diff_drive`) and converts the `sensor_msgs::Float32` commands into PWM signals using an experimentally-determined equation. These PWM signals are sent to the motor driver to actuate the motors.
* `Motor.h` defines the `Motor` class that abstracts some of the motor communications.
* `driver.h` simply refactors all the pin defs.


`encoders` - WRITE

* `encoders.ino` implements open-source code for the LM7366 encoder counters and modifies it to publish encoder ticks to `lwheel` and `rwheel`.


`test` - MANUAL R/W

* These programs simply allow us to manually read and write to the motors without depending on ROS. This is how we can run velocity/pwm test, ensure electronics are working, etc.