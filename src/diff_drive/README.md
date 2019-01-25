### Astro Differential Drive

This is a simple package that implements an open-loop diff drive. 

The diff_drive node takes Twist messages under the topic "/cmd_vel" and applies diff drive kinematics equations to split the velocities into "/cmd_vel/left" and "/cmd_vel/right". It is meant to be general-purpose, so it doesn't implement anything hardware-specific, except for the "driver" launch file.

The `differential_drive` package was used originally, but it converts velocities directly into PWM signals. The relationship between velocity and PWM depends on each robot, and needs to be experimentally determined. It also uses a PID which I had trouble tuning, and didn't seem necessary for this use case. Thus the motivation for writing my own diff_drive node in C++. 

However, the "diff_tf.py" remained useful for publishing odometry and tf from encoder ticks, so this package is a bit of a messy combination. But it works.

## Dependencies

* [differential_drive](http://wiki.ros.org/differential_drive) (must be installed manually)
* [twist_teleop_joy](http://wiki.ros.org/teleop_twist_joy)
* rosserial_python (optional by commenting out inclusion of driver.launch)
* geometry_msgs

The launch file runs:

* The diff_drive node 
* The diff_tf.py node from the existing "differential_drive" package
* The twist_teleop_joy launch file for manual remote control
* Two instances of "rosserial_python" (one Arduino for writing, the other for reading)

## diff_drive node

# Published topics
`cmd_vel/left` - (std_msgs/Float32)
    Left velocity in m/s

`cmd_vel/right` - (std_msgs/Float32)
    Right velocity in m/s

# Subscribed topics
`cmd_vel` - (geometry_msgs/Twist)
    Standard velocity command. Only linear.x and angular.z are used.

See documentation of `differential_drive` for publishers/subscribers provided by `diff_tf.py`.