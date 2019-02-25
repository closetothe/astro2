/*  Jamiel Rahi
 *  Astro 2.0 Capstone Team 14
 *  2018-2019
 *
 *  This program is for testing the motors without ROS.
 */

#include "driver.h"
#include "Motor.h"

// Instantiate motors
Motor right = Motor(ENA, IN1, IN2, 0);
Motor left = Motor(ENB, IN3, IN4, 0);

unsigned char lpwm = 125;
unsigned char rpwm = 125;

void setup()
{
  right.init();
  left.init();
  left.forward();
  right.forward();
  left.setSpeed(lpwm); 
  right.setSpeed(rpwm); 
}

void loop()
{
  delay(30);
}
