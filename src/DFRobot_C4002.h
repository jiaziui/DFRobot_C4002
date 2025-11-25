/*!
 * @file DFRobot_C4002.h
 * @brief Define basic struct of DFRobot_C4002 class
 * @details This is the header file for the DFRobot_C4002 class. It contains the declaration of the class and its members.
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author JiaLi(zhixin.liu@dfrobot.com)
 * @version V1.0
 * @date 2025-11-04
 * @url https://github.com/cdjq/DFRobot_C4002
 */
#ifndef __DFROBOT_C4002_H__
#define __DFROBOT_C4002_H__

#include <Arduino.h>

#if defined(ESP8266) || defined(ARDUINO_AVR_UNO)
#include <SoftwareSerial.h>
#else
#include <HardwareSerial.h>
#endif

//#define ENABLE_DEBUG
#ifdef ENABLE_DEBUG
#define DBG(...)                 \
  {                              \
    Serial.print("[");           \
    Serial.print(__FUNCTION__);  \
    Serial.print("(): ");        \
    Serial.print(__LINE__);      \
    Serial.print(" ] ");         \
    Serial.println(__VA_ARGS__); \
  }
#else
#define DBG(...)
#endif

#define TIME_OUT      0x64    ///< time out
#define FRAME_HEADER1 0xFA    ///< frame header1
#define FRAME_HEADER2 0xF5    ///< frame header2
#define FRAME_HEADER3 0xAA    ///< frame header3
#define FRAME_HEADER4 0xA5    ///< frame header4

#define FRAME_TYPE_WRITE_REQUSET 0x00    ///< write request frame type
#define FRAME_TYPE_READ_REQUSET  0x01    ///< read request frame type
#define FRAME_TYPE_WRITE_RESPOND 0x02    ///< write respond frame type
#define FRAME_TYPE_READ_RESPOND  0x03    ///< read respond frame type
#define FRAME_TYPE_NOTIFICATION  0x04    ///< notification frame type
#define FRAME_ERROR              0xFF    ///< error frame type

#define CMD_SET_LED_MODE                0xA1    ///< set led mode
#define CMD_CONFIG_OUT_MODE             0xA0    ///< set output mode
#define CMD_ENVIRNMENT_CALIBRATION      0x60    ///< environment calibration
#define CMD_RESTART                     0x00    ///< restart command
#define CMD_SET_DETECT_RANGE            0x86    ///< set detect sensitivity
#define CMD_FACTORY_RESET               0x80    ///< factory reset command
#define CMD_SET_REPORT_PERIOD           0x83    ///< set report period
#define CMD_SET_LIGHT_THRESHOLD         0x88    ///< set light threshold
#define CMD_SET_DISTANCE_DOOR           0x62    ///< set distance door
#define CMD_GET_VERSION                 0x82    ///< get version command
#define CMD_GET_AND_SET_RESOLUTION_MODE 0x66    ///< get resolution mode command
#define CMD_SET_DISTANCE_DOOR_THRESHOLD 0x63    ///< set distance door threshold
#define CMD_SET_BAUDRATE                0x21    ///< set baudrate command

#define NOTE_RESULT_CMD                 0x60    ///< detection result notification command
#define NOTE_ENVIRNMENT_CALIBRATION_CMD 0x03    ///< environment calibration notification command

#define SOFTWARE_VERSION 0x01    ///< get software version
#define HARDWARE_VERSION 0x00    ///< get hardware version

/**
 * @enum eResolutionMode_t
 * @brief Resolution mode
*/
typedef enum {
  eResolution80Cm = 0x00,
  eResolution20Cm = 0x01
} eResolutionMode_t;

/**
 * @enum eDistanceDoorType_t
 * @brief Distance door type
*/
typedef enum {
  eMoveDistDoor  = 0x00,
  eExistDistDoor = 0x01
} eDistanceDoorType_t;

/**
 * @enum eResponseCode_t
 * @brief Response code
*/
typedef enum {
  eReadAndWriteReq   = 0x00, /*read and write request       */
  eSucceed           = 0x01,
  eCmdErr            = 0x02, /* The CMD does not exist      */
  eAuthenticationErr = 0x03, /* Authentication error        */
  eResourcesBusy     = 0x04, /* Resources are busy          */
  eParamsErr         = 0x05, /* The parameters are illegal  */
  eDataLenErr        = 0x06, /* Abnormal data length        */
  eInternalErr       = 0x07  /* internal error              */
} eResponseCode_t;

/**
 * @enum eOutMode_t
 * @brief Output mode
*/
typedef enum {
  eOutMode1 = 0x01, /* Only when motion is detected will a high level be output */
  eOutMode2 = 0x02, /* A high level is output only when its presence is detected */
  eOutMode3 = 0x03, /* A high level only appears when movement or presence is detected */
  eOutModex = 0xFF  /* reserved                          */
} eOutMode_t;

/**
 * @enum eMoveDirection_t
 * @brief The direction of the movement
*/
typedef enum {
  eAway = 0,
  eStay = 1,
  eNear = 2
} eMoveDirection_t;

/**
 * @enum eTargetState_t
 * @brief The state of the target
*/
typedef enum {
  eNobody        = 0,
  eExist         = 1,
  eMove          = 2,
  eMoveOrExist   = 3,
  eMoveOrNobody  = 4,
  eExistOrNobody = 5,
  ePinError      = 255
} eTargetState_t;

/**
 * @enum eLedMode_t
 * @brief The operation led mode
*/
typedef enum {
  eLedOff  = 0x00,
  eLedOn   = 0x01,
  eLedKeep = 0xFF
} eLedMode_t;

/**
 * @enum eNoteType_t
 * @brief The type of the notification message
*/
typedef enum {
  eNoNote              = 0x00,
  eNoteInfoResult      = 0x01,
  eNoteInfoCalibration = 0x02,
} eNoteType_t;

/**
 * @struct sDetectResult_t
 * @brief The detection result
*/
typedef struct {
  uint8_t  targetStatus;
  uint16_t light;
  uint32_t existDistIndex;
  uint16_t existCountDown;
  uint16_t existTargetDist;
  uint8_t  existTargetEnery;
  uint16_t moveTargetDist;
  int16_t  moveTargetSpeed;
  uint8_t  moveTargetEnery;
  uint8_t  moveTargetDirect;
} sDetectResult_t;

/**
 * @struct sDataHeader_t
 * @brief The data header of the received package
*/
typedef struct {
  uint8_t  cmd;
  uint8_t  respCode;
  uint16_t dataLen;
} sDataHeader_t;

/**
 * @struct sRecvPack_t
 * @brief The received package
*/
typedef struct {
  sDataHeader_t   dataHeader;
  uint8_t         data[50];
  uint8_t         packType;
  eResponseCode_t resPonCode;
} sRecvPack_t;

/**
 * @struct sResData_t
 * @brief The detection result and environment calibration information
*/
typedef struct {
  uint8_t         cmd;
  uint8_t         respCode;
  sDetectResult_t dectResult;
  uint16_t        calibCountdown;
} sResData_t;

/**
 * @struct sMoveTarget_t
 * @brief The movement target
*/
typedef struct {
  float   distance;
  uint8_t energy;
} sExistTarget_t;

/**
 * @struct sMoveTarget_t
 * @brief The movement target
*/
typedef struct {
  float            distance;
  float            speed;
  uint8_t          energy;
  eMoveDirection_t direction;
} sMoveTarget_t;

/**
 *  @struct sRetResult_t
 *  @brief The detection result and environment calibration information
*/
typedef struct {
  eNoteType_t noteType;
  uint16_t    calibCountdown;
} sRetResult_t;

class DFRobot_C4002 {
public:
#if defined(ESP8266) || defined(ARDUINO_AVR_UNO)
  DFRobot_C4002(SoftwareSerial *sSerial, uint32_t baud);
#else
  DFRobot_C4002(HardwareSerial *hSerial, uint32_t baud, uint8_t rxpin = 0, uint8_t txpin = 0);
#endif

  ~DFRobot_C4002() {};

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
  void startEnvCalibration(uint16_t delayTime, uint16_t contTime);

  /**
   * @fn setDetectRange
   * @brief Set the detection distance range,range: 0-1200cm
   * @param closest : The closest distance to detect,   unit: cm
   * @param farthest: The farthest distance to detect,  unit: cm
   * @return true: Set detection range succeeded, false: Set detection range failed.
  */
  bool setDetectRange(uint16_t closest, uint16_t farthest);

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
   * @brief Set the resolution mode
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
  bool setDistanceDoorThreshold(eDistanceDoorType_t doorType, uint8_t *threshold);

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

protected:
  int8_t restart(void);
  bool   setLed(eLedMode_t runLed, eLedMode_t outLed);
  bool   getOutMode(void);
  bool   getResolutionMode(void);

  void        sendPack(void *pdata, uint16_t len, uint8_t msgType);
  sRecvPack_t recvPack();
  bool        checkSum(uint8_t *pdata, uint8_t len);
  uint16_t    getCheckSum(uint8_t *pdata, uint16_t len);
  void        writeReg(uint8_t reg, void *pdata, uint8_t len);
  int16_t     readReg(uint8_t reg, void *pdata, uint8_t len);

private:
#if defined(ESP8266) || defined(ARDUINO_AVR_UNO)
  SoftwareSerial *_serial;
#else
  HardwareSerial *_serial;
#endif

  uint32_t _baud;
  uint8_t  _rxpin;
  uint8_t  _txpin;
  uint8_t  _outpin;

  sDetectResult_t   _detectResult;
  eResolutionMode_t _resolutionMode = eResolution80Cm;
  eOutMode_t        _outMode;
};

#endif
