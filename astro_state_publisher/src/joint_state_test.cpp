/* Jamiel Rahi
 * Astro 2.0 Capstone Team 14
 * 2018-2019
 * Concordia University
 * GNU License
 *
 * This program is the same as joint_state_publisher.cpp
 * but includes a fake odom frame so that the robot state
 * can be visualized without actually running the robot.
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include <tf/transform_broadcaster.h>

class JointStateCollector
{
private:
	ros::NodeHandle nh_;
	ros::Subscriber lwheel_sub_;
	ros::Subscriber rwheel_sub_;
	ros::Subscriber camera_sub_;
	ros::Publisher joint_state_pub_;
	sensor_msgs::JointState js_;

public:
	JointStateCollector()
	{
		// Initialize wheel joints
		js_.name.push_back("left_wheel_joint");
		js_.name.push_back("right_wheel_joint");
		js_.velocity.push_back(0); // left
		js_.velocity.push_back(0); // right
		// Fill first two positions of the pos[] array
		// since the array lengths have to be the same size
		// (or empty)
		js_.position.push_back(0);
		js_.position.push_back(0);

		// Initialize camera joint
		js_.name.push_back("camera_joint");
		js_.position.push_back(0); // camera
		js_.velocity.push_back(0); // Empty filler

		// Initialize subscribers
		lwheel_sub_ = nh_.subscribe("joints/lwheel/vel", 100, &JointStateCollector::lwheelCallback, this);
		rwheel_sub_ = nh_.subscribe("joints/rwheel/vel", 100, &JointStateCollector::rwheelCallback, this);		
		rwheel_sub_ = nh_.subscribe("joints/camera/pos", 100, &JointStateCollector::cameraCallback, this);		
		
		// Initialize publisher
		joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
	}
	void lwheelCallback(const std_msgs::Float32::ConstPtr& vel){
		js_.velocity[0] = vel->data;
	}
	void rwheelCallback(const std_msgs::Float32::ConstPtr& vel){
		js_.velocity[1] = vel->data;
	}
	void cameraCallback(const std_msgs::Float32::ConstPtr& pos){
		js_.position[2] = pos->data;
	}
	void publish(){
		js_.header.stamp = ros::Time::now();
		joint_state_pub_.publish(js_);
	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "joint_state_test");
	JointStateCollector jointStateCollector;

	tf::TransformBroadcaster broadcaster;
	const double degree = M_PI/180;
	double angle=0;
	ROS_INFO("Started joint state publisher...");
	ros::Rate loop_rate(10);

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