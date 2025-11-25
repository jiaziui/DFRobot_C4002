/*!
 * @file getAllResults.ino
 * @brief This is an example to show how to use the DFRobot_C4002 library to get all the results of the C4002 sensor.
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author JiaLi(zhixin.liu@dfrobot.com)
 * @version V1.0
 * @date 2025-11-04
 * @url https://github.com/cdjq/DFRobot_C4002
 */
#include "DFRobot_C4002.h"

/* ---------------------------------------------------------------------------------------------------------------------
  *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
  *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
  *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
  *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  | 25/D2 |     X      |  tx1  |
  *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  | 26/D3 |     X      |  rx1  |
  * ----------------------------------------------------------------------------------------------------------------------*/
/* Baud rate can be changed */

#if defined(ESP8266) || defined(ARDUINO_AVR_UNO)
SoftwareSerial mySerial(4, 5);
DFRobot_C4002  c4002(&mySerial, 115200);
#elif defined(ESP32)
DFRobot_C4002 c4002(&Serial1, 115200, /*D2*/ D2, /*D3*/ D3);
#else
DFRobot_C4002 c4002(&Serial1, 115200);
#endif

void setup()
{

  Serial.begin(115200);

  // Initialize the C4002 sensor
  while (c4002.begin() != true) {
    Serial.println("C4002 begin failed!");
    delay(1000);
  }
  Serial.println("C4002 begin succeed!");
  delay(50);

  // Set the run led to off
  if (c4002.setRunLed(eLedOff)) {
    Serial.println("Set run led succeed!");
  } else {
    Serial.println("Set run led failed!");
  }
  delay(50);

  // Set the out led to off
  if (c4002.setOutLed(eLedOff)) {
    Serial.println("Set out led succeed!");
  } else {
    Serial.println("Set out led failed!");
  }
  delay(50);

  // Set the Resolution mode to 80cm.
  if (c4002.setResolutionMode(eResolution80Cm)) {
    Serial.println("Set resolution mode succeed!");
  } else {
    Serial.println("Set resolution mode failed!");
  }
  delay(50);
  /**
   * Note:
   * 1. eResolution80Cm: This indicates that the resolution of the "distance door" is 80cm.
   *  With a resolution of 80 cm, it supports up to 15 distance doors, with a maximum distance of 11.6 meters.
   * 2. eResolution20Cm: This indicates that the resolution of the "distance door" is 20cm.
   *  With a resolution of 20 cm, it supports up to 25 distance doors, with a maximum distance of 4.9 meters
  */

  // Set the detect range to 0-1000 cm
  uint16_t clostRange = 0;
  uint16_t farRange   = 1000;
  if (c4002.setDetectRange(clostRange, farRange)) {    // Max detect range(0-1200cm)
    Serial.println("Set detect range succeed!");
  } else {
    Serial.println("Set detect range failed!");
  }
  delay(50);

  // Enable the 'distance door'
  // Resolution mode:eResolution80Cm,This means that the number of 'distance doors' we can operate is 15
  uint8_t doorEnable[15] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };    //disenable :10,11,12,13,14  enable:0-9
  // Resolution mode:eResolution20Cm,This means that the number of 'distance doors' we can operate is 25
  // uint8_t doorEnable[25] = {0,0,1,0,0,...,1,0};
  if (c4002.enableDistanceDoor(eMoveDistDoor, doorEnable)) {    // Operation move distance door
    Serial.println("Enable move distance door succeed!");
  }
  delay(50);
  if (c4002.enableDistanceDoor(eExistDistDoor, doorEnable)) {    // Operation exist distance door
    Serial.println("Enable move distance door succeed!");
  }
  delay(50);

  // Set the light threshold to 0 lux.range: 0-50 lux
  if (c4002.setLightThreshold(0)) {
    Serial.println("Set light threshold succeed!");
  } else {
    Serial.println("Set light threshold failed!");
  }
  delay(50);

  // Set distance door threshold to 50 ,range: 0-99
  // Resolution mode:eResolution80Cm,This means that the number of 'distance doors' we can operate is 15
  uint8_t existThreshold[15] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 };
  uint8_t moveThreshold[15]  = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 };
  // Resolution mode:eResolution20Cm,This means that the number of 'distance doors' we can operate is 25
  // uint8_t existThreshold[25] = {50,...,50,50};
  // uint8_t moveThreshold[25]  = {50,...,50,50};
  if (c4002.setDistanceDoorThreshold(eExistDistDoor, existThreshold)) {
    Serial.println("Set exist distance door threshold succeed!");
  } else {
    Serial.println("Set exist distance door threshold failed!");
  }
  delay(50);
  if (c4002.setDistanceDoorThreshold(eMoveDistDoor, moveThreshold)) {
    Serial.println("Set move distance door threshold succeed!");
  } else {
    Serial.println("Set move distance door threshold failed!");
  }
  delay(50);

  // Set the report period to 1s
  if (c4002.setReportPeriod(10)) {
    Serial.println("Set report period succeed!");
  } else {
    Serial.println("Set report period failed!");
  }
  /* note: Calibration and obtaining all data must have a set cycle */

  delay(50);
}

void loop()
{
  // Get all the results of the C4002 sensor,Default loop execution
  sRetResult_t retResult = c4002.getNotInfoLoop();

  if (retResult.noteType == eNoteInfoResult) {
    Serial.println("------- Get all results --------");
    // get the light intensity
    float light = c4002.getLight();
    Serial.print("Light: ");
    Serial.print(light);
    Serial.println(" lux");

    // get Target state
    eTargetState_t targetState = c4002.getTargetState();
    Serial.print("target state: ");
    if (targetState == eNobody) {
      Serial.println("No body");
    } else if (targetState == eExist) {
      Serial.println("Exist");
    } else if (targetState == eMove) {
      Serial.println("Move");
    }
    // get exist distance door index
    uint32_t exitDoorIndex = c4002.getExistDistIndex();
    Serial.print("Exist distance door index: ");
    for (int i = 0; i < 32; i++) {
      if (exitDoorIndex & (1 << i)) {
        Serial.print(i);
        Serial.print(" ");
      }
    }
    Serial.println();

    // get exist distance door target info
    sExistTarget_t exitTarget = c4002.getExistTargetInfo();
    Serial.print("exist distance: ");
    Serial.print(exitTarget.distance);
    Serial.println(" m");
    Serial.print("exist energy: ");
    Serial.println(exitTarget.energy);

    // get move distance door index
    sMoveTarget_t moveTarget = c4002.getMoveTargetInfo();
    Serial.print("move distance: ");
    Serial.print(moveTarget.distance);
    Serial.println(" m");
    Serial.print("move energy: ");
    Serial.println(moveTarget.energy);
    Serial.print("move speed: ");
    Serial.print(moveTarget.speed);
    Serial.println(" m/s");
    Serial.print("Move direction: ");
    if (moveTarget.direction == eAway) {
      Serial.println("Away!");
    } else if (moveTarget.direction == eStay) {
      Serial.println("Directionless!");
    } else if (moveTarget.direction == eNear) {
      Serial.println("Approach!");
    }
    Serial.println("--------------------------------");
  }
  delay(50);
}
