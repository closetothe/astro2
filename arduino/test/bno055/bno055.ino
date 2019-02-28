/*  Jamiel Rahi
 *  Astro 2.0 Capstone Team 14
 *  2018-2019
 *  License: GPL
 *  ----------------------
 *  ---- GET IMU DATA ----
 *  ----------------------
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

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "calibration.h"

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  setCal();       // Set Calibration Values - Comment out to read calibration values
  delay(1000);
    
  /* Display some basic information on this sensor */
  displaySensorDetails();

  bno.setExtCrystalUse(true);  

}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  //getCalStat();
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Quaternion quat = bno.getQuat();
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x(), 4);
  Serial.print("\tY: ");
  Serial.print(euler.y(), 4);
  Serial.print("\tZ: ");
  Serial.print(euler.z(), 4);
  Serial.println("");

//  Serial.print("X: ");
//  Serial.print(quat.x(), 4);
//  Serial.print("\tY: ");
//  Serial.print(quat.y(), 4);
//  Serial.print("\tZ: ");
//  Serial.print(quat.z(), 4);
//  Serial.print("\tW: ");
//  Serial.print(quat.w(), 4);
//  Serial.println("");
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
}
