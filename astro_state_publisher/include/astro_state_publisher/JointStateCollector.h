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


#include "sensor_msgs/JointState.h"
#include <urdf/model.h>

class JointStateCollector
{
public:
	int pub_rate;
	JointStateCollector()
	{
		if(!getParams()){
			ROS_ERROR("Initialization failed. Shutting down node.");
			ros::shutdown();
		}

		parseJoints();

		// Initialize subscribers
		for(auto topic : sources_){
			auto sub = nh_.subscribe(topic, 100, &JointStateCollector::subCallback, this);	
			subs_.push_back(sub);	
			ROS_INFO("Subscribed to topic %s", topic.c_str());
		}

		// Initialize publisher
		joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
	}

	void publish(){
		js_.header.stamp = ros::Time::now();
		joint_state_pub_.publish(js_);
	}

private:
	ros::NodeHandle nh_;
	std::vector<ros::Subscriber> subs_;
	std::vector<std::string> sources_;
	std::vector<boost::shared_ptr<urdf::Joint> > joints_;
	ros::Publisher joint_state_pub_;
	sensor_msgs::JointState js_;
	bool check_limits_;
	bool zero_positions_;
	bool zero_velocities_;
	bool zero_efforts_;
	std::string urdf_;
	urdf::Model model_;

	bool getParams(){
		// Get params
		nh_.param("rate", pub_rate, 10);
		nh_.param("zero_positions", zero_positions_, false);
		nh_.param("zero_velocities", zero_velocities_, false);
		nh_.param("zero_efforts", zero_efforts_, false);

		if(nh_.getParam("sources", sources_)){
			if(sources_.empty()){
				ROS_ERROR("Empty list of sources");
				return false;
			}
		}
		else{
			ROS_ERROR("Sources param not found");
			return false;
		}

		nh_.param("check_limits", check_limits_,false);
		bool found_urdf = nh_.getParam("robot_description", urdf_);

		if(found_urdf){
			ROS_INFO("Found robot robot_description");
		}
		else{
			ROS_ERROR("Robot description not found.");
			return false;
		}

		// Parse URDF
		if(!model_.initString(urdf_)){
			ROS_ERROR("Failed to parse URDF");
			return false;
		}
		ROS_INFO("Successfully parsed URDF");

		return true;
	}

	void subCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){

		// A JointState message can have information for multiple joints,
		// so we loop through each 
		for(auto i = 0; i < joint_msg->name.size(); i++)
		{		
			auto it = std::find(js_.name.begin(), js_.name.end(), joint_msg->name[i]);

			if (it == js_.name.end()) { 
				// Joint not found 
				ROS_ERROR("Joint \"%s\" not found in robot description", joint_msg->name[i].c_str());
			} else {
				// Joint found

			  	auto j = std::distance(js_.name.begin(), it);

			  	// Enforce limits if requested
			  	double pos, vel, eff;
			  	if(check_limits_){

			  		// Check if joint has limits
			  		if(jointHasLimits(joints_[j])){
				  		if(!joint_msg->position.empty()){
				  			if(joint_msg->position[i] > joints_[j]->limits->upper){
				  				ROS_WARN("Joint \"%s\" position is past upper limit", joints_[j]->name.c_str());
				  			}
				  			else if(joint_msg->position[i] < joints_[j]->limits->lower){
				  				ROS_WARN("Joint \"%s\" position is past lower limit", joints_[j]->name.c_str());
				  			}
				  		}
				  		if(!joint_msg->velocity.empty()){
				  			if(std::abs(joint_msg->velocity[i]) > std::abs(joints_[j]->limits->velocity)){
				  				ROS_WARN("Joint \"%s\" velocity is past limit", joints_[j]->name.c_str());
				  			}		  			
				  		}
				  		if(!joint_msg->effort.empty()){
				  			if(std::abs(joint_msg->effort[i]) > std::abs(joints_[j]->limits->effort)){
				  				ROS_WARN("Joint \"%s\" effort is past limit", joints_[j]->name.c_str());
				  			}		  			
				  		}

			  		}
			  	}

			  	// Update complete joint state with new values

			  	// The complete joint state will keep arrays
			  	// as empty until it sees that type (for example, effort)
			  	// for the first time. When it does, it'll resize and fill the
			  	// rest of the array with zeros. This is to preserve the order of
			  	// the indices.

		    	if(!joint_msg->position.empty()){
		    		if(js_.position.empty())
		    			js_.position.resize(js_.name.size());
		    		js_.position[j] = joint_msg->position[i];
		    	}
		    	if(!joint_msg->velocity.empty()){
		    		if(js_.velocity.empty())
		    			js_.velocity.resize(js_.name.size());

		    		js_.velocity[j] = joint_msg->velocity[i];
		    	}
		    	if(!joint_msg->effort.empty()){
		    		if(js_.effort.empty())
		    			js_.effort.resize(js_.name.size());
		    		js_.effort[j] = joint_msg->effort[i];
		    	}
			}


		}

	}

	void parseJoints(){
		for (const auto& jnt : model_.joints_)
		{
			// From urdf::Joint::type
			// enum {UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED}
			int t = jnt.second->type;

			// Save non-fixed joints
			if(t != 6 && t != 0){
				js_.name.push_back(jnt.second->name);
				joints_.push_back(jnt.second);
				if(zero_positions_)
					js_.position.push_back(0);
				if(zero_velocities_)
					js_.velocity.push_back(0);
				if(zero_efforts_)
					js_.effort.push_back(0);
			}
		}

		if(js_.name.empty()){
			ROS_WARN("No non-fixed joints in robot description");
		}
	}

	bool jointHasLimits(boost::shared_ptr<urdf::Joint> jnt){
		if (!jnt->limits) return false;
		return (jnt->limits->lower != 0 || 
				jnt->limits->upper != 0 || 
				jnt->limits->lower != 0 || 
				jnt->limits->upper != 0 || 
				jnt->limits->effort !=0);
	}

};
