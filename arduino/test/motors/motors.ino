/*  Jamiel Rahi
 *  Astro 2.0 Capstone Team 14
 *  2018-2019
 *  This is a driver that converts velocity
 *  commands (for each wheel) into PWM signals.
 *  The actual differential drive calculations
 *  are done elsewhere, i.e. decoupled from here. 
 */



#include "driver.h"
#include "Motor.h"

// Instantiate motors
Motor right = Motor(ENA, IN1, IN2, 0);
Motor left = Motor(ENB, IN3, IN4, 0);

void left_cmd(float cmd){
  // cmd is in m/s
  
  if(abs(cmd) >= VMIN){
    
    // Experimental relationship between velocity and pwm
    int pwm;
    if (abs(cmd) > VMAX) pwm = 255;
    // TODO: UPDATE EQUATION
    else pwm = (int)(830.87*abs(cmd) - 25.617);
    
    if (cmd > 0) left.forward();
    else left.reverse();

    left.setSpeed(pwm);    
  }
  else{
    left.stop();
  }
}

void right_cmd(float cmd){
  // cmd is in m/s
  
  if(abs(cmd) >= VMIN){
    
    // Experimental relationship between velocity and pwm
    int pwm;
    if (abs(cmd) > VMAX) pwm = 250;
    else pwm = (int)(830.87*abs(cmd) - 20.617);
    
    if (cmd > 0) right.forward();
    else right.reverse();

    right.setSpeed(pwm);    
  }
  else{
    right.stop();
  }
} 

unsigned char lpwm = 255;
unsigned char rpwm = 255;

void setup()
{
  right.init();
  left.init();
  left.reverse();
  right.reverse();
  left.setSpeed(lpwm); 
  right.setSpeed(rpwm); 
}

void loop()
{
  delay(30);
}
