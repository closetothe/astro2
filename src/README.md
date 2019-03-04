# ASTRO 2 Packages
## Prototype 0.4

These are all the packages used to run the ASTRO 2 robot. 


### `astro_diff_drive`  
Basic differential drive control with encoder feedback.

### `astro_state_publisher`  
A custom implementation of a `joint_state_publisher`-type package that collects all joint states for `robot_state_publisher`.

### `astro_nav`  
The `teleop` launch file runs `astro_diff_drive`, `astro_state_publisher`, and `teleop_twist_joy` for basic teleop control.

### `astro_imu_publisher`  
Republishes raw IMU data in a REPL-compliant form along with covariances.

### `astro_vision`
Responsible for all vision-related tasks, including SLAM and visual odometry.

### `astro_localization`
Runs `astro_imu_publisher`, along with visual odometry from the `astro_vision` package, and finally runs the `robot_localization` package for a complete combined localization system.