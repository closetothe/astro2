# ASTRO 2 State Publisher

This is a general-purpose joint state publisher similar to the official `joint_state_publisher` node, but written in C++. It was created partly as an exercise and partly to add joint limit checking for the joint states. If enabled, the joint state publisher will warn you if it detects a joint beyond the limits defined in the URDF.

It collects `sensor_msgs/JointState` messages from a list of sources (topics) and combines them into one under the standard topic `joint_states`.

By default, it keeps joint state arrays empty until the first message of that type.

## Subscribed Topics
*User-defined. All subscribed topics must use JointState messages.*

## Published Topics
`joint_states` ([sensor_msgs/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html))  
	Combined joint states from all sources defined in the `sources` parameter.

## Parameters

`rate` (`float`, default: `10.0`)  
	Publishing rate. 

`robot_description` (`urdf`)  
	Robot URDF. *Required*.

`check_limits` (`bool`, default: `false`)  
	Check if joint states are beyond limits defined in URDF.

`sources` (`string[]`)  
	List of topics to read `JointState` messages from. *At least one is required*.

`zero_positions` (`bool`, default: `false`)  
	Initialize all positions to 0 (instead of an empty array).

`zero_velocities` (`bool`, default: `false`)  
	Initialize all velocities to 0 (instead of an empty array).

`zero_efforts` (`bool`, default: `false`)  
	Initialize all efforts to 0 (instead of an empty array).