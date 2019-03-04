## Encoder Publisher (Alternate)

This version of the encoder counter/publisher is no longer used in the ASTRO 2 robot. 

The only difference is that it publishes to `lwheel` and `rwheel` the encoder ticks, so that the `diff_tf` node can calculate wheel odometry. However, I discovered that `diff_tf` was no longer giving reliable results, and furthermore is not useful when the wheel odom is being fused by `robot_localization` anyway.

Because `r_l` handles integration for you, it's best to simply publish only the reliable readings: *vx* and *vyaw*.