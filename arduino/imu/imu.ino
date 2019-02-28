/*  Jamiel Rahi
 *  Astro 2.0 Capstone Team 14
 *  2018-2019
 *  License: GPL
 *  ----------------------
 *  ---- GET IMU DATA ----
 *  ----------------------
 *  This is a driver for transferring data from the
 *  Adafruit BNO055 to ROS (and ultimately robot_localization).
 *  
 *  It uses a modified version of the Adafruit BNO055 library
 *  with calibration functions, both created by "davegun"
 *  at Adafruit forums.
 *  
 *  offsets.h lists calibration data obtained beforehand using
 *  Dave's functions. See the "test" folder for more.
 *
 *  It publishes quaternion, accel, and gyro to imu/data.
 *  It also publishes temperature to imu/temp.
 *  Finally it also publishes the calibration statuses of each
 *  component (sys, accel, gyro, mag) to imu/status 
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/ByteMultiArray.h>

#include "calibration.h"

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)

ros::NodeHandle nh;

// To do hardware limitations, all the IMU measurement
// data will be crammed into a single float32 array.
// [ quat_x, quat_y, quat_z, quat_w, 
//   euler_x, euler_y, euler_z, 
//   accel_x, accel_y, accel_z, 
//   gyro_x, gyro_y, gyro_z, 
//   sys_cal, accel_cal, gyro_cal, mag_cal, 
//   temperature ]
std_msgs::Float32MultiArray float_msg;

ros::Publisher float_pub("imu/raw", &float_msg);

void setup(void) 
{  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  delay(100);
  nh.advertise(float_pub); 
  delay(100);
  

  float_msg.data_length = 21;

  // Initialize sensor
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    nh.logerror("BNO055 not found.");
    while(1);
  }
  
  setCal(); // Set Calibration Values
  delay(500);
  bno.setExtCrystalUse(true); 
}

byte i = 0;

void loop(void) 
{
  // First bunch of data is 0s
  if (i > 100){
  // Stores calibration and temperature data
  byte byte_array[5] = {0,0,0,0, 25};
  getCalStatus(byte_array);
  byte_array[4] = bno.getTemp();

  // Get main IMU data
  float float_array[21];

  imu::Quaternion quat = bno.getQuat();

  float_array[0] = quat.w();
  float_array[1] = quat.x();
  float_array[2] = quat.y();
  float_array[3] = quat.z();

  imu::Vector<3> vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float_array[4] = vec.x();
  float_array[5] = vec.y();
  float_array[6] = vec.z();

  vec = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float_array[7] = vec.x();
  float_array[8] = vec.y();
  float_array[9] = vec.z();

  vec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float_array[10] = vec.x();
  float_array[11] = vec.y();
  float_array[12] = vec.z();

  vec = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  float_array[13] = vec.x();
  float_array[14] = vec.y();
  float_array[15] = vec.z();

  float_array[16] = (float) byte_array[0];
  float_array[17] = (float) byte_array[1];
  float_array[18] = (float) byte_array[2];
  float_array[19] = (float) byte_array[3];
  float_array[20] = (float) byte_array[4];

  // Update msg pointer
  float_msg.data = float_array;

  // Publish
  float_pub.publish(&float_msg);

  nh.spinOnce();  
  }
  else i++;
  delay(BNO055_SAMPLERATE_DELAY_MS);
}