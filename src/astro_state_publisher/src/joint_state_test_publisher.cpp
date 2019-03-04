/* Jamiel Rahi
 * Astro 2.0 Capstone Team 14
 * 2018-2019
 * Concordia University
 * GNU License
 *
 * This program publishes fake joint states to test
 * my custom joint_state_publisher package. Specifically,
 * to test joint limit checking and publishing rates.
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "joint_state_test_publisher");
	ROS_INFO("Started publishing joint states...");

	ros::NodeHandle nh;
	sensor_msgs::JointState js;
	sensor_msgs::JointState js2;
	ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joints/lwheel", 100);
	ros::Publisher pub2 = nh.advertise<sensor_msgs::JointState>("joints/camera", 100);

	js.name.push_back("left_wheel_joint");
	js.velocity.push_back(0.1);

	js2.name.push_back("camera_servo_joint");
	js2.position.push_back(2.11);

	ros::Rate loop_rate(100);
	while(ros::ok()){
		pub.publish(js);
		pub2.publish(js2);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}