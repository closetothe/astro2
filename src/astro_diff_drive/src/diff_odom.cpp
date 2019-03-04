/* Jamiel Rahi
 * Astro 2.0 Capstone Team 14
 * 2018-2019
 * Concordia University
 * GNU License
 *
 * This program listens for wheel joint states of a diff drive robot
 * and publishes vx and vyaw as an Odometry message.
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

class OdomPublisher
{
public:
  int frequency;
  OdomPublisher()
  {
  	ros::NodeHandle nh_private("~"); 
  	ROS_INFO("Starting Diff Drive Odom Publisher");

    if (nh_private.getParam("base_width", w_)) ROS_INFO("Robot wheel separation width: %f", w_);
    else ROS_ERROR("Failed to get base_width parameter (required).");

    if (nh_private.getParam("wheel_radius", r_)) ROS_INFO("Robot wheel radius: %f", r_);
    else ROS_ERROR("Failed to get wheel_radius parameter (required).");

    nh_private.param<int>("frequency", frequency, 10);

    nh_private.param<std::string>("left_wheel_joint", left_joint_, "left_wheel_joint");
    nh_private.param<std::string>("right_wheel_joint", right_joint_, "right_wheel_joint");

    std::string odom_topic;
    nh_private.param<std::string>("odom_topic", odom_topic, "/wheels/odom");
    std::string js_topic;
    nh_private.param<std::string>("joint_state_topic", js_topic, "/joint_states");

    int queue_size;
    nh_private.param<int>("queue_size", queue_size, 10);
    nh_private.param<std::string>("frame_id", frame_id_, "odom");
    nh_private.param<std::string>("child_frame_id", child_frame_id_, "base_link");

    odom_msg_.header.frame_id = frame_id_;
    odom_msg_.child_frame_id = child_frame_id_;
    odom_msg_.twist.covariance.fill(0);

    std::vector<double> covar_diagonals;

    if(nh_private.getParam("twist_covariance_diagonals", covar_diagonals))
    {
    	if(covar_diagonals.size() == 6)
    	{
    		int j = 0;
    		for(int i = 0; i < 36; i+= 7)
    		{
    			odom_msg_.twist.covariance[i] = covar_diagonals[j];
    			j++;
    		}
    	}
    	else
    	{
    		ROS_ERROR("Covariance diagonal vector must have length 6.");
    	}
    }


    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, queue_size);
    js_sub_ = nh_.subscribe(js_topic, queue_size, &OdomPublisher::jointStateCallback, this);

    ROS_INFO( "Listening for joint states on topic %s...", js_topic.c_str() );
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& js)
  {
  	float v_left;
  	float v_right;
    bool found_left = false;
    bool found_right = false;
  	if(js->velocity.size() == js->name.size()){

	  	for(int i = 0; i < js->name.size(); i++)
	  	{
	  		if(js->name[i] == left_joint_)
	  		{
          // Convert to m/s
	  			v_left = (r_) * ( js->velocity[i] );
          found_left = true;
	  		}
	  		else if (js->name[i] == right_joint_)
	  		{
          // Convert to m/s
	  			v_right = (r_) * ( js->velocity[i] );
          found_right = true;
	  		}
	  		if (found_left && found_right)
	  		{

	  			float vx = (v_left + v_right)/2;
	  			float vyaw = (v_left - v_right)/w_;

	  			odom_msg_.twist.twist.linear.x = vx;
	  			odom_msg_.twist.twist.angular.z = vyaw;
	  			odom_msg_.header.stamp = ros::Time::now();
	  			odom_pub_.publish(odom_msg_);

	  			break;
	  		}
	  	}
	  	if (!found_left)
	  	{
	  		ROS_WARN("Failed to get left wheel joint state. Is it being published?");
	  	}
	  	if (!found_right)
	  	{
	  		ROS_WARN("Failed to get right wheel joint state. Is it being published?");
	  	}
  	}
  }

private:
  ros::NodeHandle nh_; 
  ros::Publisher odom_pub_;
  ros::Subscriber js_sub_;
  std::string left_joint_;
  std::string right_joint_;
  std::string frame_id_;
  std::string child_frame_id_;
  nav_msgs::Odometry odom_msg_;
  float w_; // Wheel separation width
  float r_; // Wheel radius
};


int main(int argc, char** argv){
	ros::init(argc, argv, "diff_drive_odom_publisher");
	OdomPublisher odomPublisher;
  ros::Rate loop_rate(odomPublisher.frequency);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
	
	return 0;
}
