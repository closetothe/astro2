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
 *  It publishes encoder ticks to /lwheel and /rwheel.
 *
 *  It also calculates the angular velocites of the wheels
 *  and publishes them to /joints/lwheel/vel and /joints/rwheel/vel
 *  to be collected by the astro_state_publisher
 *
 *  ALL HARDWARE-SPECIFIC PARAMETERS ARE IN:
 *  "encoders.h"
 */


#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

#include "encoders.h"
#include "ls7366_driver.h"

unsigned int t, t_prev;
long c_left, c_left_prev;
long c_right, c_right_prev;
float vel_left_prev;
float vel_right_prev;


ros::NodeHandle nh;

// Encoder count publishers
std_msgs::Int16 left_ticks;
std_msgs::Int16 right_ticks;
ros::Publisher left_count_pub("lwheel", &left_ticks);
ros::Publisher right_count_pub("rwheel", &right_ticks);

// Joint velocity publishers
std_msgs::Float32 lvel;
std_msgs::Float32 rvel;
ros::Publisher left_vel_pub("joints/lwheel/vel", &lvel);
ros::Publisher right_vel_pub("joints/rwheel/vel", &rvel);

// These hold the current encoder count.
signed int encoderLcount = 0;
signed int encoderRcount = 0;


void clearEncoderCB(std_srvs::Empty::Request, std_srvs::Empty::Response)
{
  clearEncoderCount();
}

ros::ServiceServer<std_srvs::Empty::Request, 
                   std_srvs::Empty::Response> 
                   reset_server("reset_encoders",&clearEncoderCB);

// // // //
// BEGIN //
// // // //

void setup() {
 nh.initNode();
 // Counters
 nh.advertise(left_count_pub);
 nh.advertise(right_count_pub);
 // Joint states
 nh.advertise(left_vel_pub);
 nh.advertise(right_vel_pub);

 nh.advertiseService(reset_server);
 
 initEncoders(); 
 clearEncoderCount();
}

void loop() {
 // Retrieve current encoder counters
 encoderLcount = readEncoder(1); 
 encoderRcount = readEncoder(2);

 // Fill counter messages
 left_ticks.data = encoderLcount;
 right_ticks.data = encoderRcount;

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
 lvel.data = vel_left;
 rvel.data = vel_right;

 // Publish
 left_count_pub.publish(&left_ticks);
 right_count_pub.publish(&right_ticks);
 left_vel_pub.publish(&lvel); 
 right_vel_pub.publish(&rvel);
 nh.spinOnce();

 // Update
 t_prev = t;
 c_left_prev = c_left;
 c_right_prev = c_right;
 vel_left_prev = vel_left;
 vel_right_prev = vel_right;

 delay(50);
}
