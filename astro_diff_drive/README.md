# ASTRO 2 Differential Drive

This is a simple package that implements an open-loop diff drive. 

The diff_drive node takes Twist messages under the `/cmd_vel` topic and applies diff drive kinematics equations to split the velocities into `/cmd_vel/left` and `/cmd_vel/right`. It is meant to be general-purpose, so it doesn't implement anything hardware-specific, except for the `driver` launch file.

The `differential_drive` package was used originally, but the conversion from velocity to PWM does not work great with this hardware. The relationship between velocity and PWM depends on each robot, and needs to be experimentally determined. It also uses a PID which I had trouble tuning, and didn't seem necessary for this use case. Thus the motivation for writing my own diff_drive node in C++. 

However, the `diff_tf.py` node remained useful for publishing odometry from encoder ticks, so a modified version was included.

## Dependencies

* A modified version of the `diff_tf` node from [differential_drive](http://wiki.ros.org/differential_drive). My version makes tf broadcasting optional and adds covariances (for compatibility with robot_localization). The package does not need to be installed, as `diff_tf.py` is included in the `scripts` folder.
* [twist_teleop_joy](http://wiki.ros.org/teleop_twist_joy)
* rosserial_python (optional by commenting out inclusion of driver.launch)
* geometry_msgs

The launch file runs:

* The diff_drive node 
* The modified diff_tf.py node originating from the "differential_drive" package
* The twist_teleop_joy launch file for manual remote control
* Two instances of "rosserial_python" (one Arduino for writing, the other for reading)

## diff_drive node

### Subscribed topics
`cmd_vel` - ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))  
    Standard velocity command. Only linear.x and angular.z are used.

### Published topics
`cmd_vel/left` - ([std_msgs/Float32](http://docs.ros.org/jade/api/std_msgs/html/msg/Float32.html))  
    Left velocity in m/s

`cmd_vel/right` - ([std_msgs/Float32](http://docs.ros.org/jade/api/std_msgs/html/msg/Float32.html))  
    Right velocity in m/s

See documentation of `differential_drive` for publishers/subscribers provided by `diff_tf.py`.
