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
DFRobot_C4002 c4002(&Serial1, 115200, /* D2 */ D2, /* D3 */ D3);
#else
DFRobot_C4002 c4002(&Serial1, 115200);
#endif

uint8_t outPin = 6;
/*
  * Set the input pin numbers of the development board and connect the
  * 'out' pin out of the sensor to the designated development board card
  */

void setup()
{

  Serial.begin(115200);

  // Initialize the C4002 sensor
  while (c4002.begin(outPin) != true) {
    Serial.println("C4002 begin failed!");
    delay(1000);
  }
  Serial.println("C4002 begin succeed!");
  delay(50);

  // Set the run led to off
  if (c4002.setRunLed(eLedOn)) {
    Serial.println("Set run led succeed!");
  } else {
    Serial.println("Set run led failed!");
  }
  delay(50);

  // Set the out led to off,blue led,It is connected in parallel with the 'out' pin of the sensor.
  if (c4002.setOutLed(eLedOn)) {
    Serial.println("Set out led succeed!");
  } else {
    Serial.println("Set out led failed!");
  }
  delay(50);

  // Set the output mode to eOutMode3
  if (c4002.setOutMode(eOutMode3)) {
    Serial.println("Set out mode succeed!");
  } else {
    Serial.println("Set out mode failed!");
  }
  /**
   * Note:
   * The output mode can be set to eOutMode1, eOutMode2, or eOutMode3.
   * eOutMode1: A high level will be output when movement is detected.
   * eOutMode2: A high level will be output when exit is detected.
   * eOutMode3: A high level will be output when movement or exit is detected.
  */
}

void loop()
{
  // Get the target state by the output pin
  eTargetState_t targetState = c4002.getOutTargetState();

  // Print the target state
  Serial.println("Target state: ");
  if (targetState == eNobody) {
    Serial.println("No body!");
  } else if (targetState == eExist) {
    Serial.println("Exit!");
  } else if (targetState == eMove) {
    Serial.println("Move!");
  } else if (targetState == eMoveOrNobody) {
    Serial.println("Movement or no body!");
  } else if (targetState == eExistOrNobody) {
    Serial.println("Exit or no body!");
  } else if (targetState == eMoveOrExist) {
    Serial.println("Movement or exit!");
  } else {
    Serial.println("Pin error! please check the connection or pin number!");
  }
  delay(500);
}
