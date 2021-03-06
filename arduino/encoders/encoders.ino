/*  Jamiel Rahi
 *  Astro 2.0 Capstone Team 14
 *  2018-2019
 *  --------------------------
 *  ---- READ FROM MOTORS ----
 *  --------------------------
 *  This ROS node gets encoder ticks
 *  from two LS7366R devices using a
 *  driver written by Jason Traud.
 *
 *  It calculates the angular velocites of the wheels
 *  and publishes them as joint states to /joints/wheels
 *  to be collected by the astro_state_publisher and diff_twist nodes.
 *
 *  ALL HARDWARE-SPECIFIC PARAMETERS ARE IN:
 *  "encoders.h"
 */


#include <ros.h>
#include <sensor_msgs/JointState.h>

#include "encoders.h"
#include "ls7366_driver.h"

unsigned int t, t_prev;
long c_left, c_left_prev;
long c_right, c_right_prev;
float vel_left_prev;
float vel_right_prev;

ros::NodeHandle nh;

// Joint velocity publisher
// (publishes left and right wheel velocities as one msg)
sensor_msgs::JointState wheel_joints;
char* joint_names[2] = {"left_wheel_joint", "right_wheel_joint"};
float vel[2] = {0, 0};
ros::Publisher wheel_joint_pub("joints/wheels", &wheel_joints);

// These hold the current encoder count.
signed int encoderLcount = 0;
signed int encoderRcount = 0;

// // // //
// BEGIN //
// // // //

void setup() {
 wheel_joints.name = joint_names;
 wheel_joints.name_length = 2;
 wheel_joints.velocity = vel;
 wheel_joints.velocity_length = 2;

 nh.getHardware()->setBaud(115200);
 nh.initNode();
 delay(500);

 // Joint states
 nh.advertise(wheel_joint_pub);
 delay(100);

 initEncoders(); 
 clearEncoderCount();
}

void loop() {
 // Retrieve current encoder counters
 encoderLcount = readEncoder(1); 
 encoderRcount = readEncoder(2);

 // // // // // // // // // // // // //
 //   CALCULATE ANGULAR VELOCITIES   //
 // // // // // // // // // // // // //

 c_left = encoderLcount;
 c_right = encoderRcount;
 t = millis();

 float dt = (t - t_prev)/1000.0;
 float dc_right = c_right - c_right_prev;
 float dc_left = c_left - c_left_prev;
 float vel_right = 0;
 float vel_left = 0;

 if (dt > 0.001 && abs(dc_left) > 0)
  vel_left = (2*PI*dc_left)/(LCPR*dt);

 if (dt > 0.001 && abs(dc_right) > 0)
  vel_right = (2*PI*dc_right)/(RCPR*dt);

 // Ignore sudden changes in velocity
 // when encoder ticks overflow
 if (abs(vel_left-vel_left_prev) > 100)
  vel_left = vel_left_prev;

 if (abs(vel_right-vel_right_prev) > 100)
  vel_right = vel_right_prev;

 //float rpm_left = (60*vel_left)/(2*PI);
 //float rpm_right = (60*vel_right)/(2*PI);

 // Fill joint vel messages
 wheel_joints.velocity[0] = vel_left;
 wheel_joints.velocity[1] = vel_right;

 // Publish joint states
 wheel_joint_pub.publish(&wheel_joints); 
 nh.spinOnce();

 // Update
 t_prev = t;
 c_left_prev = c_left;
 c_right_prev = c_right;
 vel_left_prev = vel_left;
 vel_right_prev = vel_right;

 delay(50);
}
