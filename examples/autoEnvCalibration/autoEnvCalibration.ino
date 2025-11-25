/*!
 * @file autoEnvCalibration.ino
 * @brief This is an example to demonstrate how to use the DFRobot_C4002 library to perform environmental calibration.
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

  // Query the versions of software and hardware
  delay(50);
  String softwareVersion = "";
  softwareVersion        = c4002.getVersioninfo(SOFTWARE_VERSION);
  Serial.print("Software version:");
  Serial.println(softwareVersion);
  delay(10);
  String hardwareVersion = "";
  hardwareVersion        = c4002.getVersioninfo(HARDWARE_VERSION);
  Serial.print("Hardware version:");
  Serial.println(hardwareVersion);
  delay(2000);

  // Turn on the run led and out led
  if (c4002.setRunLed(eLedOn)) {
    Serial.println("Set run led succeed!");
  } else {
    Serial.println("Set run led failed!");
  }
  delay(50);
  if (c4002.setOutLed(eLedOn)) {
    Serial.println("Set out led succeed!");
  } else {
    Serial.println("Set out led failed!");
  }

  delay(3000);
  // Set the report period to 1s
  if (c4002.setReportPeriod(10)) {
    Serial.println("Set report period succeed!");
  } else {
    Serial.println("Set report period failed!");
  }
  /* note: Calibration and obtaining all data must have a set cycle */

  // Start environmental calibration
  // Delay time：3s ，Calibration time：15s( 15-65535 s )
  c4002.startEnvCalibration(3, 15);
  Serial.println("Start environmental calibration:");
  /**
   * Note:
   * 1. The calibration process takes about 15 seconds, and the delay time is 3 seconds.
   * 2. When resetting the development board, please find an open area to calibrate it
   * 3. When starting the calibration, there should be no one on either side of the sensor
   *  directly in front of the transmitter, otherwise it will affect the calibration accuracy
   *  of the sensor
   */
}

void loop()
{
  //Obtain the calibration results
  sRetResult_t retResult = c4002.getNotInfoLoop();
  if (retResult.noteType == eNoteInfoCalibration) {
    Serial.print("Calibration countdown:");
    Serial.print(retResult.calibCountdown);
    Serial.println(" s");
    if (retResult.calibCountdown == 0) {
      Serial.println("Environmental calibration complete!");
    }
  }
}
