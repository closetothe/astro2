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


#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <joint_state_publisher.hpp>

int main(int argc, char** argv){
	ros::init(argc, argv, "joint_state_publisher");

	JointStateCollector jointStateCollector;

	tf::TransformBroadcaster broadcaster;
	const double degree = M_PI/180;
	double angle=0;
	ROS_INFO("Started joint state publisher...");

	ros::Rate loop_rate(jointStateCollector.pub_rate);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";
    
	while(ros::ok()){
		odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
        broadcaster.sendTransform(odom_trans);

		jointStateCollector.publish();
		ros::spinOnce();

		angle += degree/4; //

		loop_rate.sleep();
	}
	return 0;
}