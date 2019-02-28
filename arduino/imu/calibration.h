/* Calibration functions created by davegun at Adafruit forums
 * (and modified slightly for this use case)
 * https://www.forums.adafruit.com/viewtopic.php?f=19&t=73014&sid=7cd41f381e6dd45e21de95bff0f476d7
 * 
 * Allows offsets to be retrieved from Serial once a full calibration status is reached (3 3 3 3)
 * Then, at startup, these can be used right away to set calibration.
 * 
 * These use a version of the Arduino BNO055 library modified by Dave.
 */
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

void getCalStatus(byte status[]){
  // Dave's Mod - Read calibration registers
  byte cal = bno.getCalib();
  status[0] = (0xC0 & cal) >> 6; //sys
  status[1] = (0x0C & cal) >> 2; //accel
  status[2] = (0x30 & cal) >> 4; //gyro
  status[3] = (0x03 & cal) >> 0; // mag
}
