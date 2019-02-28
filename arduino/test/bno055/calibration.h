/* Calibration functions created by davegun at Adafruit forums
 * https://www.forums.adafruit.com/viewtopic.php?f=19&t=73014&sid=7cd41f381e6dd45e21de95bff0f476d7
 * 
 * Allows offsets to be retrieved from Serial once a full calibration status is reached (3 3 3 3)
 * Then, at startup, these can be used right away to set calibration.
 * 
 * These use a version of the Arduino BNO055 library modified by Dave.
 */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "offsets.h"

extern Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setCal(){
  // DAVES MOD - Writes calibration data to sensor//
  byte calData;
  bno.setMode( bno.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode
  delay(25);
  
  calData = bno.setCalvalARL(ARL);
  
  calData = bno.setCalvalARM(ARM);
  
  calData = bno.setCalvalMRL(MRL);
  
  calData = bno.setCalvalMRM(MRM);
  
  calData = bno.setCalvalAOXL(AOXL);
  
  calData = bno.setCalvalAOXM(AOXM);
  
  calData = bno.setCalvalAOYL(AOYL);
  
  calData = bno.setCalvalAOYM(AOYM);
  
  calData = bno.setCalvalAOZL(AOZL);
 
  calData = bno.setCalvalAOZM(AOZM);
  
  calData = bno.setCalvalMOXL(MOXL);
  
  calData = bno.setCalvalMOXM(MOXM);
  
  calData = bno.setCalvalMOYL(MOYL);
 
  calData = bno.setCalvalMOYM(MOYM);
 
  calData = bno.setCalvalMOZL(MOZL);
  
  calData = bno.setCalvalMOZM(MOZM);
  
  calData = bno.setCalvalGOXL(GOXL);
  
  calData = bno.setCalvalGOXM(GOXM);
  
  calData = bno.setCalvalGOYL(GOYL);
  
  calData = bno.setCalvalGOYM(GOYM);
  
  calData = bno.setCalvalGOZL(GOZL);
  
  calData = bno.setCalvalGOZM(GOZM);
  
  bno.setMode( bno.OPERATION_MODE_NDOF );    // Put into NDOF Mode
  delay(25);
}

void getCal(){
  // Dave's Mod - Reads Calibration Data when sensors are calibrated
  byte calData;
  bno.setMode( bno.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode
  
  calData = bno.getCalvalARL();
  Serial.println(calData);
  
  calData = bno.getCalvalARM();
  Serial.println(calData);
  
  calData = bno.getCalvalMRL();
  Serial.println(calData);
  
  calData = bno.getCalvalMRM();
  Serial.println(calData);
  
  calData = bno.getCalvalAOXL();
  Serial.println(calData);
  
  calData = bno.getCalvalAOXM();
  Serial.println(calData);
  
  calData = bno.getCalvalAOYL();
  Serial.println(calData);
  
  calData = bno.getCalvalAOYM();
  Serial.println(calData);
  
  calData = bno.getCalvalAOZL();
  Serial.println(calData);
 
  calData = bno.getCalvalAOZM();
  Serial.println(calData);
  
  calData = bno.getCalvalMOXL();
  Serial.println(calData);
  
  calData = bno.getCalvalMOXM();
  Serial.println(calData);
  
  calData = bno.getCalvalMOYL();
  Serial.println(calData);
 
  calData = bno.getCalvalMOYM();
  Serial.println(calData);
 
  calData = bno.getCalvalMOZL();
  Serial.println(calData);
  
  calData = bno.getCalvalMOZM();
  Serial.println(calData);
  
  calData = bno.getCalvalGOXL();
  Serial.println(calData);
  
  calData = bno.getCalvalGOXM();
  Serial.println(calData);
  
  calData = bno.getCalvalGOYL();
  Serial.println(calData);
  
  calData = bno.getCalvalGOYM();
  Serial.println(calData);
  
  calData = bno.getCalvalGOZL();
  Serial.println(calData);
  
  calData = bno.getCalvalGOZM();
  Serial.println(calData);
  
  while(1){                              // Stop
    delay(1000);
  }
  
  
}
void getCalStat(){
  // Dave's Mod - Move sensor to calibrate, when status shows calibration, read values
  byte cal = bno.getCalib();
  byte calSys = (0xC0 & cal) >> 6;
  byte calGyro = (0x30 & cal) >> 4;
  byte calAccel = (0x0C & cal) >> 2;
  byte calMag = (0x03 & cal) >> 0;
  
  Serial.println(cal);
  Serial.print("System calibration status "); Serial.println(calSys);
  Serial.print("Gyro   calibration status "); Serial.println(calGyro);
  Serial.print("Accel  calibration status "); Serial.println(calAccel);
  Serial.print("Mag    calibration status "); Serial.println(calMag);
  
  delay(1000);
  if (cal==255){
    getCal();
  }
}
