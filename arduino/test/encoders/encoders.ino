/*  Jamiel Rahi
 *  Astro 2.0 Capstone Team 14
 *  2018-2019
 *
 *  This program is for testing the encoders without ROS.
 */

#include "encoders.h"
#include "ls7366_driver.h"

// These hold the current encoder count.
signed long encoderRcount = 0;
signed long encoderLcount = 0;

unsigned int t, t_prev;
long c_left, c_left_prev;
long c_right, c_right_prev;
float vel_left_prev;
float vel_right_prev;

void setup() {
 Serial.begin(9600);
 initEncoders();       Serial.println("Encoders Initialized...");  
 clearEncoderCount();  Serial.println("Encoders Cleared...");

 // Initialize 
 t = t_prev = millis();
 c_left = c_left_prev = 0;
 c_right = c_right_prev = 0;
 vel_left_prev = vel_right_prev = 0;
}

void loop() {
 // Retrieve current encoder counters
 encoderLcount = readEncoder(1); 
 encoderRcount = readEncoder(2);
 //Serial.print("LEFT TICKS: "); Serial.print(encoderLcount); 
 //Serial.print("RIGHT TICKS: "); Serial.println(encoderRcount); 

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


 float rpm_left = (60*vel_left)/(2*PI);
 float rpm_right = (60*vel_right)/(2*PI);
 Serial.print("LEFT RPM: "); Serial.print(rpm_left); 
 Serial.print("  RIGHT RPM: "); Serial.println(rpm_right);

 t_prev = t;
 c_left_prev = c_left;
 c_right_prev = c_right;
 vel_left_prev = vel_left;
 vel_right_prev = vel_right;
 delay(200);
}
