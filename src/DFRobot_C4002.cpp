/*!
 * @file DFRobot_C4002.cpp
 * @brief Define the basic struct of DFRobot_C4002 class, the implementation of basic method
 * @copyright	Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author JiaLi(zhixin.liu@dfrobot.com)
 * @version V1.0
 * @date 2025-11-04
 * @url https://github.com/cdjq/DFRobot_C4002
 */
#include "DFRobot_C4002.h"

#if defined(ESP8266) || defined(ARDUINO_AVR_UNO)
DFRobot_C4002::DFRobot_C4002(SoftwareSerial *sSerial, uint32_t baud)
{
  _serial = sSerial;
  _baud   = baud;
}

#else
DFRobot_C4002::DFRobot_C4002(HardwareSerial *hSerial, uint32_t baud, uint8_t rxpin, uint8_t txpin)
{
  _serial = hSerial;
  _baud   = baud;
  _rxpin  = rxpin;
  _txpin  = txpin;
}

#endif

bool DFRobot_C4002::begin(uint8_t outPin)
{
#if defined(ESP32)
  _serial->begin(_baud, SERIAL_8N1, _rxpin, _txpin);
#else
  _serial->begin(_baud);
#endif
  _outpin = outPin;

  if (_outpin != 255) {
    pinMode(_outpin, INPUT);
    digitalWrite(_outpin, HIGH);
  }

  bool ret;
  delay(10);
  ret = setReportPeriod(255);
  if (ret == false) {
    return false;
  }

  ret = getResolutionMode();
  if (ret == false) {
    return false;
  }
  ret = getOutMode();

  return ret;
}

bool DFRobot_C4002::getOutMode(void)
{
  uint8_t  sendData[10];
  uint16_t dataLen = 4;
  sendData[0]      = CMD_CONFIG_OUT_MODE;
  sendData[1]      = eReadAndWriteReq;
  sendData[2]      = dataLen >> 0 & 0xFF;
  sendData[3]      = dataLen >> 8 & 0xFF;
  sendPack(sendData, dataLen, FRAME_TYPE_READ_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    _outMode = (eOutMode_t)recPack.data[0];
    return true;
  } else {
    return false;
  }
}

bool DFRobot_C4002::setDistanceDoorThreshold(eDistanceDoorType_t doorType, uint8_t *threshold)
{
  uint8_t  sendData[35];
  uint16_t dataLen   = 0;
  uint16_t temp      = 7;
  int      doorNum   = 0;
  uint8_t  doorIndux = 0x03;
  if (_resolutionMode == eResolution80Cm) {
    doorNum = 15;
  } else if (_resolutionMode == eResolution20Cm) {
    doorNum   = 25;
    doorIndux = 0x04;
  }
  temp += doorNum;
  sendData[dataLen++] = CMD_SET_DISTANCE_DOOR_THRESHOLD;
  sendData[dataLen++] = eReadAndWriteReq;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = (uint8_t)doorType;
  sendData[dataLen++] = doorIndux;
  sendData[dataLen++] = 0x01;

  for (uint8_t i = 0; i < doorNum; i++) {
    sendData[dataLen++] = threshold[i];
  }

  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}

bool DFRobot_C4002::setBaudrate(uint32_t baud)
{
  uint8_t  sendData[10];
  uint16_t dataLen    = 0;
  uint16_t temp       = 8;
  sendData[dataLen++] = CMD_SET_BAUDRATE;
  sendData[dataLen++] = eReadAndWriteReq;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = baud >> 0 & 0xFF;
  sendData[dataLen++] = baud >> 8 & 0xFF;
  sendData[dataLen++] = baud >> 16 & 0xFF;
  sendData[dataLen++] = baud >> 24 & 0xFF;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}

eTargetState_t DFRobot_C4002::getOutTargetState(void)
{
  eTargetState_t ret = ePinError;
  if (_outpin == 255) {
    return ePinError;
  }
  if (_outMode == eOutMode1) {
    if (digitalRead(_outpin) == HIGH) {
      ret = eMove;
    } else {
      ret = eExistOrNobody;
    }
  } else if (_outMode == eOutMode2) {
    if (digitalRead(_outpin) == HIGH) {
      ret = eExist;
    } else {
      ret = eMoveOrNobody;
    }
  } else if (_outMode == eOutMode3) {
    if (digitalRead(_outpin) == HIGH) {
      ret = eMoveOrExist;
    } else {
      ret = eNobody;
    }
  }
  return ret;
}

bool DFRobot_C4002::getResolutionMode(void)
{
  uint8_t  sendData[10];
  uint16_t dataLen = 4;
  sendData[0]      = CMD_GET_AND_SET_RESOLUTION_MODE;
  sendData[1]      = eReadAndWriteReq;
  sendData[2]      = dataLen >> 0 & 0xFF;
  sendData[3]      = dataLen >> 8 & 0xFF;
  sendPack(sendData, dataLen, FRAME_TYPE_READ_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    _resolutionMode = (eResolutionMode_t)recPack.data[0];
    return true;
  } else {
    return false;
  }
}

String DFRobot_C4002::getVersioninfo(uint8_t version)
{
  uint8_t  sendData[10];
  uint16_t dataLen = 5;
  String   ret     = "";

  sendData[0] = CMD_GET_VERSION;
  sendData[1] = eReadAndWriteReq;
  sendData[2] = dataLen >> 0 & 0xFF;
  sendData[3] = dataLen >> 8 & 0xFF;
  sendData[4] = version;

  sendPack(sendData, dataLen, FRAME_TYPE_READ_REQUSET);
  sRecvPack_t recPack = recvPack();

  if (eSucceed == recPack.resPonCode) {
    for (uint16_t i = 0; i < (recPack.dataHeader.dataLen - 4); i++) {
      ret += (char)recPack.data[i];
    }
  } else {
    ret = "Get version error!";
  }

  return ret;
}

bool DFRobot_C4002::setLightThreshold(float threshold)
{
  uint8_t  sendData[10];
  uint16_t dataLen       = 0;
  uint16_t temp          = 6;
  sendData[dataLen++]    = CMD_SET_LIGHT_THRESHOLD;
  sendData[dataLen++]    = eReadAndWriteReq;
  sendData[dataLen++]    = temp >> 0 & 0xFF;
  sendData[dataLen++]    = temp >> 8 & 0xFF;
  uint16_t thresholdTemp = (uint16_t)(threshold * 10);
  sendData[dataLen++]    = thresholdTemp >> 0 & 0xFF;
  sendData[dataLen++]    = thresholdTemp >> 8 & 0xFF;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}

bool DFRobot_C4002::setReportPeriod(uint8_t period)
{
  uint8_t  sendData[10];
  uint16_t dataLen    = 0;
  uint16_t temp       = 5;
  sendData[dataLen++] = CMD_SET_REPORT_PERIOD;
  sendData[dataLen++] = eReadAndWriteReq;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = period;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}

bool DFRobot_C4002::factoryReset(void)
{
  uint8_t  sendData[10];
  uint16_t dataLen = 5;

  sendData[0] = CMD_FACTORY_RESET;
  sendData[1] = eReadAndWriteReq;
  sendData[2] = dataLen >> 0 & 0xFF;
  sendData[3] = dataLen >> 8 & 0xFF;
  sendData[4] = 0x00;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}

bool DFRobot_C4002::setResolutionMode(eResolutionMode_t mode)
{
  uint8_t  sendData[10];
  uint16_t dataLen = 5;
  sendData[0]      = CMD_GET_AND_SET_RESOLUTION_MODE;
  sendData[1]      = eReadAndWriteReq;
  sendData[2]      = dataLen >> 0 & 0xFF;
  sendData[3]      = dataLen >> 8 & 0xFF;
  sendData[4]      = (uint8_t)mode;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    _resolutionMode = mode;
    return true;
  } else {
    return false;
  }
}

bool DFRobot_C4002::enableDistanceDoor(eDistanceDoorType_t doorType, uint8_t *doorData)
{
  uint8_t  sendData[40];
  uint16_t dataLen = 0;
  uint16_t temp    = 5;
  int      doorNum = 0;
  if (_resolutionMode == eResolution80Cm) {
    doorNum = 15;
  } else if (_resolutionMode == eResolution20Cm) {
    doorNum = 25;
  }
  temp += doorNum;

  sendData[dataLen++] = CMD_SET_DISTANCE_DOOR;
  sendData[dataLen++] = eReadAndWriteReq;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = (uint8_t)doorType;
  for (int i = 0; i < doorNum; i++) {
    sendData[dataLen++] = doorData[i];
  }
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}

int8_t DFRobot_C4002::restart(void)
{
  int8_t   ret = 0;
  uint8_t  sendData[10];
  uint16_t dataLen = 5;
  sendData[0]      = CMD_RESTART;
  sendData[1]      = eReadAndWriteReq;
  sendData[2]      = dataLen >> 0 & 0xFF;
  sendData[3]      = dataLen >> 8 & 0xFF;
  sendData[4]      = 0x00;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    ret = 0;
  } else {
    ret = -1;
  }
  delay(100);
  return ret;
}

bool DFRobot_C4002::setDetectRange(uint16_t closest, uint16_t farthest)    // 0-1200cm
{
  uint8_t  sendData[10];
  uint16_t dataLen     = 0;
  uint16_t temp        = 8;
  uint16_t closestTemp = closest, farthestTemp = farthest;
  sendData[dataLen++] = CMD_SET_DETECT_RANGE;
  sendData[dataLen++] = eReadAndWriteReq;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;

  if (farthestTemp > 1200) {
    farthestTemp = 1200;
  }
  if (closestTemp > farthestTemp) {
    return false;
  }
  sendData[dataLen++] = closestTemp >> 0 & 0xFF;
  sendData[dataLen++] = closestTemp >> 8 & 0xFF;
  sendData[dataLen++] = farthestTemp >> 0 & 0xFF;
  sendData[dataLen++] = farthestTemp >> 8 & 0xFF;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();

  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}
/********************************************************************* */

void DFRobot_C4002::startEnvCalibration(uint16_t delayTime, uint16_t contTime)
{
  uint8_t  sendData[10];
  uint16_t dataLen    = 0;
  uint16_t temp       = 9;
  sendData[dataLen++] = CMD_ENVIRNMENT_CALIBRATION;
  sendData[dataLen++] = eReadAndWriteReq;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = delayTime >> 0 & 0xFF;
  sendData[dataLen++] = delayTime >> 8 & 0xFF;
  sendData[dataLen++] = contTime >> 0 & 0xFF;
  sendData[dataLen++] = contTime >> 8 & 0xFF;
  sendData[dataLen++] = 0x01;    //Automatically generate thresholds
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  recvPack();
}

bool DFRobot_C4002::setOutMode(eOutMode_t outMode)
{
  uint8_t  sendData[10];
  uint16_t dataLen    = 0;
  uint16_t temp       = 5;
  sendData[dataLen++] = CMD_CONFIG_OUT_MODE;
  sendData[dataLen++] = eReadAndWriteReq;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = (uint8_t)outMode;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    _outMode = outMode;
    return true;
  } else {
    return false;
  }
}

bool DFRobot_C4002::setLed(eLedMode_t runLed, eLedMode_t outLed)
{
  uint8_t  sendData[10];
  uint16_t dataLen    = 0;
  sendData[dataLen++] = CMD_SET_LED_MODE;
  sendData[dataLen++] = eReadAndWriteReq;
  uint16_t temp       = 6;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = runLed;
  sendData[dataLen++] = outLed;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}

bool DFRobot_C4002::setRunLed(eLedMode_t runLed)
{
  uint8_t  sendData[10];
  uint16_t dataLen    = 0;
  uint16_t temp       = 6;
  sendData[dataLen++] = CMD_SET_LED_MODE;
  sendData[dataLen++] = eReadAndWriteReq;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = runLed;
  sendData[dataLen++] = eLedKeep;

  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}

bool DFRobot_C4002::setOutLed(eLedMode_t outLed)
{
  uint8_t  sendData[10];
  uint16_t dataLen = 0;
  uint16_t temp    = 6;

  sendData[dataLen++] = CMD_SET_LED_MODE;
  sendData[dataLen++] = eReadAndWriteReq;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = eLedKeep;
  sendData[dataLen++] = outLed;
  sendPack(sendData, dataLen, FRAME_TYPE_WRITE_REQUSET);

  sRecvPack_t recPack = recvPack();
  if (eSucceed == recPack.resPonCode) {
    return true;
  } else {
    return false;
  }
}

sRetResult_t DFRobot_C4002::getNotInfoLoop(void)
{
  sRetResult_t ret;
  sRecvPack_t  recData = recvPack();

  if (eSucceed == recData.resPonCode) {
    if (recData.packType == FRAME_TYPE_NOTIFICATION) {    //note
      if (recData.dataHeader.cmd == NOTE_RESULT_CMD) {
        memcpy(&_detectResult, &recData.data[0], sizeof(sDetectResult_t));
        ret.noteType = eNoteInfoResult;
      } else if (recData.dataHeader.cmd == NOTE_ENVIRNMENT_CALIBRATION_CMD) {
        DBG("get calibration result");
        ret.calibCountdown = recData.data[1] << 8 | recData.data[0];
        ret.noteType       = eNoteInfoCalibration;
      } else {
        ret.noteType = eNoNote;
      }
    }
  }

  return ret;
}

eTargetState_t DFRobot_C4002::getTargetState(void)
{
  return (eTargetState_t)_detectResult.targetStatus;
}

float DFRobot_C4002::getLight(void)
{
  return ((float)_detectResult.light * 0.1);
}

uint32_t DFRobot_C4002::getExistDistIndex(void)
{
  return _detectResult.existDistIndex;
}

sExistTarget_t DFRobot_C4002::getExistTargetInfo(void)
{
  sExistTarget_t info;
  info.distance = ((float)_detectResult.existTargetDist * 0.01);
  info.energy   = _detectResult.existTargetEnery;
  return info;
}

sMoveTarget_t DFRobot_C4002::getMoveTargetInfo(void)
{
  sMoveTarget_t info;
  info.distance  = ((float)_detectResult.moveTargetDist * 0.01);
  info.energy    = _detectResult.moveTargetEnery;
  info.speed     = ((float)_detectResult.moveTargetSpeed * 0.01);
  info.direction = (eMoveDirection_t)_detectResult.moveTargetDirect;
  return info;
}

void DFRobot_C4002::sendPack(void *pdata, uint16_t len, uint8_t msgType)
{
  uint8_t sendData[50] = { 0 };

  uint16_t dataLen  = 0;
  uint16_t checkSum = 0;

  sendData[dataLen++] = FRAME_HEADER1;
  sendData[dataLen++] = FRAME_HEADER2;
  sendData[dataLen++] = FRAME_HEADER3;
  sendData[dataLen++] = FRAME_HEADER4;
  uint16_t temp       = len + 10;
  sendData[dataLen++] = temp >> 0 & 0xFF;
  sendData[dataLen++] = temp >> 8 & 0xFF;
  sendData[dataLen++] = 0x00;
  sendData[dataLen++] = msgType;
  memcpy(&sendData[dataLen], pdata, len);
  dataLen += len;
  checkSum = getCheckSum((uint8_t *)sendData, dataLen);

  sendData[dataLen++] = checkSum >> 0 & 0xFF;
  sendData[dataLen++] = checkSum >> 8 & 0xFF;

  while (_serial->available() > 0) _serial->read();
  writeReg(0, sendData, dataLen);
}

sRecvPack_t DFRobot_C4002::recvPack()
{
  sRecvPack_t recvDat;
  memset(&recvDat, 0, sizeof(recvDat));
  uint8_t *pdata = (uint8_t *)malloc(60 * sizeof(uint8_t));
  if (pdata == NULL) {
    recvDat.packType = FRAME_ERROR;
    return recvDat;
  }

  uint16_t recvLen = readReg(0, pdata, 8);
  DBG("recvLen:");
  DBG(recvLen);

  if (recvLen == 8 && pdata[0] == FRAME_HEADER1 && pdata[1] == FRAME_HEADER2 && pdata[2] == FRAME_HEADER3 && pdata[3] == FRAME_HEADER4) {
    uint16_t packLen = (pdata[5] << 8) | pdata[4];
    recvLen          = readReg(0, &pdata[8], packLen - 8);
    if (recvLen == (packLen - 8)) {
      recvDat.packType = pdata[7];
      if (checkSum(pdata, packLen)) {

        uint16_t dataLen = (pdata[11] << 8) | pdata[10];
        memcpy(&recvDat, &pdata[8], dataLen);
        recvDat.resPonCode = (eResponseCode_t)recvDat.dataHeader.respCode;
        DBG("get resPonCode");
        DBG(recvDat.resPonCode);

        if (recvDat.packType == FRAME_TYPE_NOTIFICATION) {    //note
          DBG("get note result");
        } else if (recvDat.packType == FRAME_TYPE_WRITE_RESPOND) {    //write
          DBG("get write respond");
        } else if (recvDat.packType == FRAME_TYPE_READ_RESPOND) {    //read
          DBG("get read respond");
        } else {
          DBG("this is error pack");
          recvDat.resPonCode = eCmdErr;
        }

      } else {
        recvDat.resPonCode = eAuthenticationErr;
        DBG("Authentication error");
      }
    } else {
      recvDat.resPonCode = eDataLenErr;
      DBG(" recvLen error");
    }
  } else {
    recvDat.resPonCode = eAuthenticationErr;
    DBG("Authentication error");
  }
  free(pdata);
  return recvDat;
}

bool DFRobot_C4002::checkSum(uint8_t *pdata, uint8_t len)
{
  uint16_t calculateParity = 0;

  for (uint8_t i = 0; i < len - 2; i++) {
    calculateParity += pdata[i];
  }
  uint16_t temp = (pdata[len - 1] << 8) | pdata[len - 2];
  if (calculateParity == temp) {
    return true;
  }
  return false;
}

uint16_t DFRobot_C4002::getCheckSum(uint8_t *pdata, uint16_t len)
{
  uint16_t parity = 0;
  for (uint16_t i = 0; i < len; i++) {
    parity += pdata[i];
  }
  return parity;
}

void DFRobot_C4002::writeReg(uint8_t reg, void *pdata, uint8_t len)
{
  if (pdata == NULL) {
    return;
  }
  _serial->write((uint8_t *)pdata, len);
  len = reg;
}

int16_t DFRobot_C4002::readReg(uint8_t reg, void *pdata, uint8_t len)
{
  uint16_t i       = 0;
  uint32_t nowtime = millis();
  uint8_t *pdat    = (uint8_t *)pdata;

  while (millis() - nowtime < TIME_OUT) {
    while (_serial->available() > 0) {
      if (i == len)
        return len;
      pdat[i++] = _serial->read();
    }
  }
  len = reg;
  return i;
}
