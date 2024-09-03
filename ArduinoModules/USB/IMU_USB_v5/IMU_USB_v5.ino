/*      
  * USB IMU code for For AgOpenGPS
  * 4 Feb 2021, Brian Tischler
  * 2 Sept 2024 
  * Like all Arduino code - copied from somewhere else :)
  * So don't claim it as your own
  */

#include "BNO_RVC.h"
#include <elapsedMillis.h>
#include <SoftwareSerial.h>

// BNO08x definitions
#define REPORT_INTERVAL 50  //Report interval in ms (same as the delay at the bottom)

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

//CMPS PGN - 211
uint8_t data[] = { 0x80, 0x81, 0x7D, 0xD3, 8, 0, 0, 0, 0, 0, 0, 0, 0, 15 };
int16_t dataSize = sizeof(data);

//Roomba Vac mode for BNO085 and data
BNO_rvc rvc = BNO_rvc();
BNO_rvcData bnoData;
elapsedMillis bnoTimer;
bool bnoTrigger = false;
bool useBNO08xRVC = false;

SoftwareSerial mySerial(10,11);

void imuHandler();

// IMU
int imuHeading;
int imuRoll;
int imuPitch;
int imuYawRate;

void setup() {
  Serial.begin(38400);  // Start serial port

  mySerial.begin(115200);
  
  rvc.begin(&mySerial);

  bnoTimer = 0;
  Serial.println("\r\nChecking for serial BNO08x");
  while (bnoTimer < 5000) {
    //check if new bnoData
    if (rvc.read(&bnoData)) {
      useBNO08xRVC = true;
      Serial.println("Serial BNO08x Good To Go :-)");
      imuHandler();
      break;
    }
  }
  if (!useBNO08xRVC) Serial.println("No Serial BNO08x not Connected or Found");
}

void loop() {

  //delay(70);
  //RVC BNO08x
  //Serial.println("Reading");
  if (rvc.read(&bnoData)) useBNO08xRVC = true;
  //&& bnoTimer > 70
  if (useBNO08xRVC) {
    useBNO08xRVC = false;
    //Serial.println("DataFound");
    imuHandler();  //Get IMU data ready
    //bnoTimer = 0;

    //the heading x10
    data[5] = (uint8_t)imuHeading;
    data[6] = imuHeading >> 8;

    //the roll x10
    data[7] = (uint8_t)imuRoll;
    data[8] = imuRoll >> 8;


    int16_t CK_A = 0;

    for (int16_t i = 2; i < dataSize - 1; i++) {
      CK_A = (CK_A + data[i]);
    }

    data[dataSize - 1] = CK_A;

    Serial.write(data, dataSize);
    Serial.flush();

    //10 hz
    delay(REPORT_INTERVAL);
  }
}

void imuHandler() {
  float angVel;

  // Fill rest of Panda Sentence - Heading
  imuHeading = bnoData.yawX10;

  // the pitch x100
  imuPitch = bnoData.pitchX10;

  // the roll x100
  imuRoll = bnoData.rollX10;

  // YawRate
  if (rvc.angCounter > 0) {
    angVel = ((float)bnoData.angVel) / (float)rvc.angCounter;
    angVel *= 10.0;
    rvc.angCounter = 0;
    bnoData.angVel = (int16_t)angVel;
  } else {
    bnoData.angVel = 0;
  }

  bnoData.angVel = bnoData.angVel;
}
