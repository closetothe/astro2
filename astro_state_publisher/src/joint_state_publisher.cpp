/* Jamiel Rahi
 * Astro 2.0 Capstone Team 14
 * 2018-2019
 * Concordia University
 * GNU License
 *
 * This program collects all joint states from the hardware nodes
 * and publishes them together as a standard JointState message.
 *
 * It also optionally 'enforces' joint limits as defined in the robot urdf.
 *
 * The launch file uses this, a URDF, and the robot_state_publisher 
 * package to publish the entire robot state to tf.
 */


#include <ros/ros.h>
#include "JointStateCollector.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "joint_state_publisher");

	JointStateCollector jointStateCollector;

	ROS_INFO("Started joint state publisher...");

	ros::Rate loop_rate(jointStateCollector.pub_rate);
    
	while(ros::ok()){
		jointStateCollector.publish();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}