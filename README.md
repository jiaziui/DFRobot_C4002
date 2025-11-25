# DFRobot_C4002
- [中文版](./README_CN.md)

This is a 24GHz millimeter-wave radar distance sensor with side-mounted motion detection range of 11m and stationary detection range of 10m (top-mounted motion detection range with a diameter of 11m and stationary detection range of 10m). It also features proximity and departure detection, area zoning detection, environmental noise collection, and an onboard light sensor. This sensor is suitable for smart home applications.


![svg]


## Product Link（）

    SKU：SEN0691

## Table of Contents

* [Summary](#Summary)
* [Installation](#Installation)
* [Methods](#Methods)
* [Compatibility](#Compatibility)
* [History](#History)
* [Credits](#Credits)

## Summary

* Supports a detection range of 0-11m
* Supports 5V main controller
* Supports serial communication
* Supports OUT pin output for detection results
* Supports OUT pin output mode setting
* Supports environmental noise collection
* Supports light intensity detection
* Supports distance threshold enable and threshold value setting
* Supports reporting cycle setting
* Supports environmental light threshold setting
* Supports 80cm and 20cm resolution setting
* Supports obtaining target status and related data

## Installation
There are two methods for using this library：
1. Open Arduino IDE, search for "DFRobot_C4002" on the status bar in Tools ->Manager Libraries and install the library.
2. Download the library file before use, paste it into \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
/**
 * @fn begin
 * @brief Initialize the serial port and set the output pin
 * @param outPin: The output pin, default is 255, which means no output pin is used.
 * @return true: Initialization succeeded, false: Initialization failed.
 */
  bool begin(uint8_t outPin = 255);
  /**
   * @fn setRunLed
   * @brief Set the operation led of runing status
   * @param switching
   *      eLedOff: Turn off the operation LED.
   *      eLedOn : Turn on the operation LED.
   * @return true: Operation LED succeeded, false: Operation LED failed.
  */
  bool setRunLed(eLedMode_t switching);

  /**
   * @fn setOutLed
   * @brief Set the output led
   * @param switching
   * @n     eLedOff: Turn off the output LED.
   * @n     eLedOn : Turn on the output LED.
   * @return true: Output LED succeeded, false: Output LED failed.
  */
  bool setOutLed(eLedMode_t switching);

  /**
   * @fn setOutMode
   * @brief Set the output mode
   * @param outMode
   * @n      eOutMode1: Only when motion is detected will a high level be output.
   * @n      eOutMode2: A high level is output only when its presence is detected.
   * @n      eOutMode3: A high level only appears when movement or presence is detected.
   * @return true: Set output mode succeeded, false: Set output mode failed.
  */
  bool setOutMode(eOutMode_t outMode);

  /**
   * @fn startEnvCalibration
   * @brief Start environment calibration
   * @param delayTime: Delay the time when the calibration starts to be executed, unit: s
   * @param contTime : The calibration time, unit: s
   * @return true: Start calibration succeeded, false: Start calibration failed.
  */
  void startEnvCalibration(uint16_t delayTime ,uint16_t contTime);

  /**
   * @fn setDetectRange
   * @brief Set the detection distance range,range: 0-1200cm
   * @param closest : The closest distance to detect,   unit: cm
   * @param farthest: The farthest distance to detect,  unit: cm
   * @return true: Set detection range succeeded, false: Set detection range failed.
  */
  bool setDetectRange(uint16_t closest,uint16_t farthest);

  /**
   * @fn enableDistanceDoor
   * @brief Enable the distance door function
   * @param doorType
   * @n      eMoveDistDoor : Use this parameter when the movement door parameter is enabled
   * @n      eExistDistDoor: Use this parameter when the existence door parameter is enabled
   * @param doorData: An array of type uint8 t is needed, with parameters 0 and 1 representing disabling and enabling respectively
   * @return true: Enable door succeeded, false: Enable door failed.
  */
  bool enableDistanceDoor(eDistanceDoorType_t doorType, uint8_t *doorData);

  /**
   * @fn factoryReset
   * @brief Factory reset the device
   * @return true: Factory reset succeeded, false: Factory reset failed.
   */
  bool factoryReset(void);

  /**
   * @fn setResolutionMode
   * @brief Set the detection distance range,range: 0-1200cm
   * @param mode : The resolution mode,  eResolution80Cm: 80cm, eResolution20Cm: 20cm
   * @return true: Set resolution mode succeeded, false: Set resolution mode failed.
  */
  bool setResolutionMode(eResolutionMode_t mode);

  /**
   * @fn setReportPeriod
   * @brief Set the report period
   * @param period : The report period, range: 0-255, unit: 0.1s
   * @return true: Set report period succeeded, false: Set report period failed.
  */
  bool setReportPeriod(uint8_t period);

  /**
   * @fn setLightThreshold
   * @brief Set the light threshold
   * @param threshold : The light threshold, range: 0-50, unit: lux
   * @return true: Set light threshold succeeded, false: Set light threshold failed.
   */
  bool setLightThreshold(float threshold);

  /**
   * @fn setDistanceDoorThreshold
   * @brief Set the distance door threshold
   * @param doorType
   * @n      eMoveDistDoor : Use this parameter when the movement door parameter is enabled
   * @n      eExistDistDoor: Use this parameter when the existence door parameter is enabled
   * @param threshold : An array of type uint8 t is needed, with parameters 0-99 representing the threshold value of the distance door
   * @return true: Set distance door threshold succeeded, false: Set distance door threshold failed.
  */
  bool setDistanceDoorThreshold(eDistanceDoorType_t doorType,uint8_t * threshold);

  /**
   * @fn setBaudrate
   * @brief Set the baudrate of the serial port
   * @param baud : The baud rate should not be too high; otherwise, it will lead to data loss, unit: bps
   * @return true: Set baudrate succeeded, false: Set baudrate failed.
  */
  bool setBaudrate(uint32_t baud);

  /**
   * @fn getNotInfoLoop
   * @brief Get the detection result and environment calibration information
   * @return  sRetResult_t
   * @n           noteType      : The type of the notification message, eNoteInfoResult: detection result notification message, eNoteInfoCalibration: environment calibration notification message.
   * @n           calibCountdown: The remaining time of the environment calibration, unit: s.
  */
  sRetResult_t getNotInfoLoop(void);

  /**
   * @fn getTargetState
   * @brief Get the current state of the target
   * @return eTargetState_t
   * @n          eNobody        : No target is detected.
   * @n          eExist         : The target is detected.
   * @n          eMove          : The target is moving.
  */
  eTargetState_t getTargetState(void);

  /**
   * @fn getLight
   * @brief Get the current light intensity
   * @return The current light intensity, unit: lux.
  */
  float getLight(void);

  /**
   * @fn getExistDistIndex
   * @brief Get the index of the detected target
   * @return The index of the detected target, range: 0-25.
  */
  uint32_t getExistDistIndex(void);

  /**
   * @fn getExistTargetInfo
   * @brief Get the information of the detected target
   * @return sExistTarget_t
   * @n        distance: The distance of the detected target, unit: m.
   * @n        energy  : The energy of the detected target, range:0-99.
  */
  sExistTarget_t getExistTargetInfo(void);

  /**
   * @fn getMoveTargetInfo
   * @brief Get the information of the detected target
   * @return sMoveTarget_t
   * @n        distance : The distance of the detected target, unit: m.
   * @n        speed    : The speed of the detected target, unit: m/s.
   * @n        energy   : The energy of the detected target, range:0-99.
   * @n        direction: The direction of the detected target, eMoveDirection_t.
   */
  sMoveTarget_t getMoveTargetInfo(void);

  /**
   * @fn getVersioninfo
   * @brief Get the version information of the device
   * @param version : The version type,
   * @n      SOFTWARE_VERSION: software version
   * @n      HARDWARE_VERSION: hardware version
   * @return String :The version information of the device.
  */
  String getVersioninfo(uint8_t version);

  /**
   * @fn getOutTargetState
   * @brief Get the current state of the output target
   * @return eTargetState_t
   * @n          eNobody        : No output target is detected.
   * @n          eExist         : The output target is detected.
   * @n          eMove          : The output target is moving.
   * @n          eMoveOrExist   : The output target is moving or detected.
   * @n          eMoveOrNobody  : The output target is moving or no output target is detected.
   * @n          eExistOrNobody : The output target is detected or no output target is detected.
   * @n          ePinError      : The pin error occurred.
  */
  eTargetState_t getOutTargetState(void);
```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
Micro:bit          |              |              |      √      |


## History

- 2025/11/04 - V1.0.0 version

## Credits

Written by JiaLi(zhixin.liu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
