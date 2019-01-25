/*  Jamiel Rahi
 *  Astro 2.0 Capstone Team 14
 *  2018-2019
 *  This is a driver that converts velocity
 *  commands (for each wheel) into PWM signals.
 *  The actual differential drive calculations
 *  are done elsewhere, i.e. decoupled from here. 
 */



#include "defs.h"
#include "controls.h"

#include <ros.h>  
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

// Instantiate motors
Motor right = Motor(ENA, IN1, IN2, 0, LED_RIGHT);
Motor left = Motor(ENB, IN3, IN4, 0, LED_LEFT);

void left_cb(const std_msgs::Float32& cmd){
  // cmd is in m/s
  
  if(abs(cmd.data) >= VMIN){
    
    // Experimental relationship between velocity and pwm
    int pwm;
    if (abs(cmd.data) > VMAX) pwm = 255;
    else pwm = (int)(830.87*abs(cmd.data) - 25.617);
    
    if (cmd.data > 0) left.forward();
    else left.reverse();

    left.setSpeed(pwm);    
  }
  else{
    left.stop();
  }
}

void right_cb(const std_msgs::Float32& cmd){
  // cmd is in m/s
  
  if(abs(cmd.data) >= VMIN){
    
    // Experimental relationship between velocity and pwm
    // Right motor moves a little faster
    int pwm;
    if (abs(cmd.data) > VMAX) pwm = 250;
    else pwm = (int)(830.87*abs(cmd.data) - 20.617);
    
    if (cmd.data > 0) right.forward();
    else right.reverse();

    right.setSpeed(pwm);    
  }
  else{
    right.stop();
  }
} 

ros::Subscriber<std_msgs::Float32> left_motor_sub("/cmd_vel/left", left_cb);
ros::Subscriber<std_msgs::Float32> right_motor_sub("/cmd_vel/right", right_cb);

void setup()
{
  right.init();
  left.init();
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  nh.initNode();
  nh.subscribe(left_motor_sub);
  nh.subscribe(right_motor_sub);
}

void loop()
{
  nh.spinOnce();
  delay(30);
}
