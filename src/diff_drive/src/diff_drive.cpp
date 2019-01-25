/* Jamiel Rahi
 * Astro 2.0 Capstone Team 14
 * 2018-2019
 * Concordia University
 * GNU License
 *
 * This program converts Twist (cmd_vel) messages into diff drive commands.
 * We use the linear.x and angular.z components to calculate wheel speeds.
 * The rest are ignored.
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

// Note that Twist messages are in Float64, so we will 
// lose some data in converting them to Float32. It should
// be fine, since we don't need that much precision.

class TwistToDiffDrive
{
public:
  TwistToDiffDrive()
  {
    if (nh_.getParam("base_width", w_)) ROS_INFO("Robot wheel width: %f", w_);
    else ROS_ERROR("Failed to get base_width parameter (required).");

    left_pub_ = nh_.advertise<std_msgs::Float32>("/cmd_vel/left", 100);
    right_pub_ = nh_.advertise<std_msgs::Float32>("/cmd_vel/right", 100);
    twist_sub_ = nh_.subscribe("cmd_vel", 100, &TwistToDiffDrive::twistCallback, this);
  }

  void twistCallback(const geometry_msgs::Twist& twist)
  {
  	std_msgs::Float32 r, l;
    r.data = twist.linear.x + (w_/2)*twist.angular.z; 
    l.data = twist.linear.x - (w_/2)*twist.angular.z;
  	right_pub_.publish(r);
    left_pub_.publish(l);
  }

private:
  ros::NodeHandle nh_; 
  ros::Subscriber twist_sub_;
  ros::Publisher left_pub_;
  ros::Publisher right_pub_;
  float w_;
};


int main(int argc, char** argv){
	ros::init(argc, argv, "diff_drive");
	TwistToDiffDrive twistToDiffDrive;
	ros::spin();
	return 0;
}