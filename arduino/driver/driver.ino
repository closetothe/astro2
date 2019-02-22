/*  Jamiel Rahi
 *  Astro 2.0 Capstone Team 14
 *  2018-2019
 *  -------------------------
 *  ---- WRITE TO MOTORS ----
 *  -------------------------
 *  This is a driver that converts velocity
 *  commands (for each wheel) into PWM signals.
 *
 *  It subscribes to left and right velocity commands
 *  from astro_diff_drive and uses an experimentally-determined
 *  linear relationship to calculate the desired PWM.
 *
 *  ALL HARDWARE SPECIFIC PARAMETERS ARE IN:
 *  "driver.h"
 */



#include "driver.h"
#include "Motor.h"

#include <ros.h>  
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

// Instantiate motors
Motor right = Motor(ENA, IN1, IN2, RIGHTDB, LED_RIGHT);
Motor left = Motor(ENB, IN3, IN4, LEFTDB, LED_LEFT);

void left_cb(const std_msgs::Float32& cmd){
  // cmd is in m/s
  
  if(abs(cmd.data) >= VMIN){
    
    // Experimental relationship between velocity and pwm
    int pwm;

    if (abs(cmd.data) < VMAX) pwm = left.velocityToPWM((float)cmd.data);
    else pwm = left.velocityToPWM(VMAX);
    
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
    if (abs(cmd.data) < VMAX) pwm = right.velocityToPWM((float)cmd.data);
    else pwm = right.velocityToPWM(VMAX);

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
  right.setVelocityParams(RIGHTA, RIGHTB, RDIA);
  right.setMaxPWM(RLIMIT);

  left.setVelocityParams(LEFTA, LEFTB, LDIA);
  left.setMaxPWM(LLIMIT);

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
