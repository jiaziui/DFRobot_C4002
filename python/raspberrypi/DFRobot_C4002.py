# -*- coding: utf-8 -*
'''!
@file DFRobot_C4002.py
@brief Define the basic struct of DFRobot_C4002 class, the implementation of basic method
@copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
@license The MIT License (MIT)
@author [JiaLi](zhixin.liu@dfrobot.com)
@version V1.0
@date 2025-11-04
@url https://github.com/cdjq/DFRobot_C4002
'''

import time
import serial
import RPi.GPIO as GPIO
from ctypes import Structure, c_uint8, c_uint16, c_uint32, c_float, c_int16

TIME_OUT = 0x64  # time out
FRAME_HEADER1 = 0xFA  # frame header1
FRAME_HEADER2 = 0xF5  # frame header2
FRAME_HEADER3 = 0xAA  # frame header3
FRAME_HEADER4 = 0xA5  # frame header4
FRAME_TYPE_WRITE_REQUSET = 0x00  # write request frame type
FRAME_TYPE_READ_REQUSET = 0x01  # read request frame type
FRAME_TYPE_WRITE_RESPOND = 0x02  # write respond frame type
FRAME_TYPE_READ_RESPOND = 0x03  # read respond frame type
FRAME_TYPE_NOTIFICATION = 0x04  # notification frame type
FRAME_ERROR = 0xFF  # error frame type
CMD_SET_LED_MODE = 0xA1  # set led mode
CMD_CONFIG_OUT_MODE = 0xA0  # set output mode
CMD_ENVIRNMENT_CALIBRATION = 0x60  # environment calibration
CMD_RESTART = 0x00  # restart command
CMD_SET_DETECT_RANGE = 0x86  # set detect sensitivity
CMD_FACTORY_RESET = 0x80  # factory reset command
CMD_SET_REPORT_PERIOD = 0x83  # set report period
CMD_SET_LIGHT_THRESHOLD = 0x88  # set light threshold
CMD_SET_DISTANCE_DOOR = 0x62  # set distance door
CMD_GET_VERSION = 0x82  # get version command
CMD_GET_AND_SET_RESOLUTION_MODE = 0x66  # get resolution mode command
CMD_SET_DISTANCE_DOOR_THRESHOLD = 0x63  # set distance door threshold
CMD_SET_BAUDRATE = 0x21  # set baudrate command
NOTE_RESULT_CMD = 0x60  # detection result notification command
NOTE_ENVIRNMENT_CALIBRATION_CMD = 0x03  # environment calibration notification command
SOFTWARE_VERSION = 0x01  # get software version
HARDWARE_VERSION = 0x00  # get hardware version


class DetectResultStruct(Structure):
  _fields_ = [
    ("target_status", c_uint8),
    ("light", c_uint16),
    ("exist_door_index", c_uint32),
    ("exist_count_down", c_uint16),
    ("exist_target_distance", c_uint16),
    ("exist_target_energy", c_uint8),
    ("move_target_distance", c_uint16),
    ("move_target_speed", c_int16),
    ("move_target_energy", c_uint8),
    ("move_target_direction", c_uint8),
  ]


class DataHeader(Structure):
  _fields_ = [("cmd", c_uint8), ("resp_code", c_uint8), ("data_len", c_uint16)]


class RecvPack(Structure):
  _fields_ = [("data_header", DataHeader), ("data", c_uint8 * 50), ("pack_type", c_uint8), ("result_pon_code", c_uint8)]


class ResData(Structure):
  _fields_ = [("cmd", c_uint8), ("resp_code", c_uint8), ("dect_result", DetectResultStruct), ("calibration_down", c_uint16)]


class ExistTarget(Structure):
  _fields_ = [("distance", c_float), ("energy", c_uint8)]


class MoveTarget(Structure):
  _fields_ = [("distance", c_float), ("speed", c_float), ("energy", c_uint8), ("direction", c_uint8)]


class RetResult(Structure):
  _fields_ = [("note_type", c_uint8), ("calibration_down", c_uint16)]


class DFRobot_C4002(object):
  RESOLUTION_80CM = 0x00
  RESOLUTION_20CM = 0x01

  MOVE_DISTANCE_DOOR = 0x00
  EXIST_DISTANCE_DOOR = 0x01

  READ_AND_WRITE_REQ = 0x00
  SUCCEED = 0x01
  CMD_ERR = 0x02
  AUTHENTICATION_ERR = 0x03
  RESOURCES_BUSY = 0x04
  PARAMS_ERR = 0x05
  DATA_LEN_ERR = 0x06
  INTERNAL_ERR = 0x07

  OUT_MODE1 = 0x01
  OUT_MODE2 = 0x02
  OUT_MODE3 = 0x03

  AWAY = 0
  STAY = 1
  NEAR = 2

  NO_BODY = 0
  EXIST = 1
  MOVE = 2
  MOVE_OR_EXIST = 3
  MOVE_OR_NO_BODY = 4
  EXIST_OR_NO_BODY = 5
  PIN_ERROR = 255

  LED_OFF = 0x00
  LED_ON = 0x01
  LED_KEEP = 0xFF

  NO_NOTE = 0x00
  NOTE_INFO_RESULT = 0x01
  NOTE_INFO_CALIBRATION = 0x02

  _out_mode = OUT_MODE3
  _resolution_mode = RESOLUTION_80CM
  _detect_result = DetectResultStruct()
  _outpin = 255

  def __init__(self, baud):
    self.ser = serial.Serial('/dev/ttyAMA0', baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=0.5)
    if self.ser.isOpen() == False:
      self.ser.open()

  def begin(self, outpin=255):
    '''!
    @brief begin
    param outpin: output pin, default is 255, which means no output pin is used.
    @return True or False
    '''
    ret = False
    time.sleep(0.5)
    if outpin != 255:
      self._outpin = outpin
      GPIO.setmode(GPIO.BCM)
      GPIO.setup(self._outpin, GPIO.IN)
    ret = self.set_report_period(255)
    if ret == False:
      return ret
    ret = self.get_out_mode()
    if ret == False:
      return ret
    ret = self.get_resolution_mode()
    return ret

  def set_run_led(self, run_led):
    '''!
    @brief set run led
    @param switching LED_OFF:off LED_ON:on LED_KEEP:keep
    @return True or False
    '''
    send_data = [0x00] * 10
    data_len = 6
    ret = False

    send_data[0] = CMD_SET_LED_MODE
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = run_led & 0xFF
    send_data[5] = self.LED_KEEP

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def set_out_led(self, out_led):
    '''!
    @brief set out led
    @param outled LED_OFF:off LED_ON:on LED_KEEP:keep
    @return True or False
    '''
    send_data = [0x00] * 10
    data_len = 6
    ret = False

    send_data[0] = CMD_SET_LED_MODE
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = self.LED_KEEP
    send_data[5] = out_led & 0xFF

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def set_out_mode(self, out_mode):
    '''!
    @brief set output mode
    @param out_mode
    @n  OUT_MODE1:Only when motion is detected will a high level be output.
    @n  OUT_MODE2:A high level is output only when its presence is detected.
    @n  OUT_MODE3:A high level only appears when movement or presence is detected.
    @return True or False
    '''
    send_data = [0x00] * 10
    data_len = 5
    ret = False
    send_data[0] = CMD_CONFIG_OUT_MODE
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = out_mode & 0xFF

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def start_env_calibration(self, delay_time, cont_time):
    '''!
    @brief start environment calibration
    @param delay_time: delay time.range:0-65535s
    @param cont_time : continuous time.range:15-65535s
    '''
    send_data = [0x00] * 10
    data_len = 9

    send_data[0] = CMD_ENVIRNMENT_CALIBRATION
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = delay_time >> 0 & 0xFF
    send_data[5] = delay_time >> 8 & 0xFF
    send_data[6] = cont_time >> 0 & 0xFF
    send_data[7] = cont_time >> 8 & 0xFF
    send_data[8] = 0x01

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    self.recv_pack()

  def set_detect_range(self, closet_distance, farthest_distance):
    '''!
    @brief set detect range,unit:cm,range:0-1100cm
    @param closet_distance   : recent distance
    @param farthest_distance : farthest distance
    @return True or False
    '''
    send_data = [0x00] * 10
    data_len = 8
    ret = False
    closet = closet_distance
    farthest = farthest_distance

    send_data[0] = CMD_SET_DETECT_RANGE
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    if closet > farthest:
      ret = False
      return ret
    if farthest > 1200:
      farthest = 1200
    if closet < 0:
      closet = 0

    send_data[4] = closet >> 0 & 0xFF
    send_data[5] = closet >> 8 & 0xFF
    send_data[6] = farthest >> 0 & 0xFF
    send_data[7] = farthest >> 8 & 0xFF
    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def enable_distance_door(self, door_type, door_data):
    '''!
    @brief enable distance door
    @param door_type distance door type
    @param door_data distance door data
    @return True or False
    '''
    send_data = [0x00] * 40
    data_len = 5
    ret = False
    door_num = 0
    if self._resolution_mode is self.RESOLUTION_80CM:
      door_num = 15
    elif self._resolution_mode is self.RESOLUTION_20CM:
      door_num = 25

    data_len = data_len + door_num

    send_data[0] = CMD_SET_DISTANCE_DOOR
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = door_type & 0xFF
    for i in range(door_num):
      send_data[5 + i] = door_data[i]

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def factory_reset(self):
    '''!
    @brief factory reset,Restart takes effect
    @return True or False
    '''
    send_data = [0x00] * 10
    data_len = 5
    ret = False

    send_data[0] = CMD_FACTORY_RESET
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = 0x00
    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def set_resolution_mode(self, resolution_mode):
    '''!
    @brief enable distance door
    @param door_type：distance door type
    @n              MOVE_DISTANCE_DOOR:move distance door
    @n              EXIST_DISTANCE_DOOR:exist distance door
    @param door_data：The distance gate status data is an array. The length of
    @n                the array depends on the resolution mode. In the 80cm
    @n                mode, 15 gates can be set, and in the 20cm mode, 25 gates
    @n                can be set. Each gate corresponds to the index of
    @n                the array, starting from 0.
    @return True or False
    '''
    send_data = [0x00] * 10
    data_len = 5
    ret = False

    send_data[0] = CMD_GET_AND_SET_RESOLUTION_MODE
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = resolution_mode & 0xFF

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      self._resolution_mode = resolution_mode
      ret = True
    else:
      ret = False
    return ret

  def set_report_period(self, period):
    '''!
    @brief set report period
    @param period report period,unit:0.1s
    @return True or False
    '''
    send_data = [0x00] * 10
    data_len = 5
    ret = False

    send_data[0] = CMD_SET_REPORT_PERIOD
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = period & 0xFF

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def set_light_threshold(self, threshold):
    '''!
    @brief set light threshold
    @param threshold light threshold
    @return True or False
    '''
    send_data = [0x00] * 10
    data_len = 6
    ret = False

    send_data[0] = CMD_SET_LIGHT_THRESHOLD
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    threshold_temp = int(threshold * 10)
    send_data[4] = threshold_temp >> 0 & 0xFF
    send_data[5] = threshold_temp >> 8 & 0xFF

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def set_distance_door_threshold(self, door_type, threshold):
    '''!
    @brief set distance door threshold
    @param door_type: distance door type
    @n              MOVE_DISTANCE_DOOR:move distance door
    @n              EXIST_DISTANCE_DOOR:exist distance door
    @param threshold: Threshold, this is an array. The length of the array depends
    @n                on the resolution mode. In the 80cm mode, 15 gates can be set,
    @n                and in the 20cm mode, 25 gates can be set
    @return True or False
    '''
    send_data = [0x00] * 35
    data_len = 7
    ret = False
    door_num = 0
    door_index = 0x03
    if self._resolution_mode is self.RESOLUTION_80CM:
      door_num = 15
    elif self._resolution_mode is self.RESOLUTION_20CM:
      door_num = 25
      door_indux = 0x04

    data_len = data_len + door_num

    send_data[0] = CMD_SET_DISTANCE_DOOR_THRESHOLD
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = door_type & 0xFF
    send_data[5] = door_index & 0xFF
    send_data[6] = 0x01
    for i in range(door_num):
      send_data[6 + i] = threshold[i]

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def set_baudrate(self, baudrate):
    '''!
    @brief set baudrate
    @param baudrate baudrate
    @return True or False
    '''
    send_data = [0x00] * 10
    data_len = 8
    ret = False

    send_data[0] = CMD_SET_BAUDRATE
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = baudrate >> 0 & 0xFF
    send_data[5] = baudrate >> 8 & 0xFF
    send_data[6] = baudrate >> 16 & 0xFF
    send_data[7] = baudrate >> 24 & 0xFF

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def get_note_info_loop(self):
    '''!
    @brief get note data info loop
    '''
    ret_result = RetResult()
    rec_data = self.recv_pack()

    if rec_data.result_pon_code == self.SUCCEED:
      if rec_data.pack_type == FRAME_TYPE_NOTIFICATION:
        if rec_data.data_header.cmd == NOTE_RESULT_CMD:
          self._detect_result.target_status = rec_data.data[0]
          self._detect_result.light = rec_data.data[1] | rec_data.data[2] << 8
          self._detect_result.exist_door_index = rec_data.data[3] | (rec_data.data[4] << 8) | (rec_data.data[5] << 16) | (rec_data.data[6] << 24)
          self._detect_result.exist_count_down = rec_data.data[7] | (rec_data.data[8] << 8)
          self._detect_result.exist_target_distance = rec_data.data[9] | (rec_data.data[10] << 8)
          self._detect_result.exist_target_energy = rec_data.data[11]
          self._detect_result.move_target_distance = rec_data.data[12] | (rec_data.data[13] << 8)
          self._detect_result.move_target_speed = rec_data.data[14] | (rec_data.data[15] << 8)
          self._detect_result.move_target_energy = rec_data.data[16]
          self._detect_result.move_target_direction = rec_data.data[17]

          ret_result.note_type = self.NOTE_INFO_RESULT
        elif rec_data.data_header.cmd == NOTE_ENVIRNMENT_CALIBRATION_CMD:
          ret_result.calibration_down = rec_data.data[1] << 8 | rec_data.data[0]
          ret_result.note_type = self.NOTE_INFO_CALIBRATION
        else:
          ret_result.note_type = self.NO_NOTE
    return ret_result

  def get_target_state(self):
    '''!
    @brief get target state
    @return detect result
    @n    NO_BODY:no body
    @n    EXIST:exist
    @n    MOVE:move
    '''
    return self._detect_result.target_status

  def get_light(self):
    '''!
    @brief Obtain light intensity,unit:lux
    '''
    return self._detect_result.light * 0.1

  def get_exist_dist_indux(self):
    '''!
    @brief get exist distance and indeX
    '''
    return self._detect_result.exist_door_index

  def get_exist_target_info(self):
    '''!
    @brief get exist target info
    @return ExistTarget object
    @n          distance:distance,unit:cm
    @n          energy:energy,range:0-100
    '''
    info = ExistTarget()
    info.diatance = self._detect_result.exist_target_distance * 0.01
    info.energy = self._detect_result.exist_target_energy
    return info

  def get_move_target_info(self):
    '''!
    @brief get move target info
    @return MoveTarget object
    @n          distance:distance,unit:m
    @n          speed:speed,unit:m/s
    @n          energy:energy,range:0-100
    @n          direction:direction,range:
    @n              AWAY:away
    @n              STAY:nondirectional
    @n              NEAR:near
    '''
    info = MoveTarget()
    info.distance = self._detect_result.move_target_distance * 0.01
    info.speed = self._detect_result.move_target_speed * 0.01
    info.energy = self._detect_result.move_target_energy
    info.direction = self._detect_result.move_target_direction
    return info

  def get_version_info(self, version):
    '''!
    @brief get version info
    @param version version type
    @n  VERSION_SOFTWARE:software version
    @n  VERSION_HARDWARE:hardware version
    @return string info
    '''
    send_data = [0x00] * 10
    data_len = 5
    ret = False
    ret_version = ""
    send_data[0] = CMD_GET_VERSION
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = version & 0xFF

    self.send_pack(send_data, data_len, FRAME_TYPE_READ_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      for i in range(recv_data.data_header.data_len):
        ret_version += chr(recv_data.data[i])
    else:
      ret_version = "Get version error!"

    return ret_version

  def get_out_target_state(self):
    '''!
    @brief get out target state
    @return out target state
    @n  NO_BODY:no body
    @n  MOVE:move
    @n  EXIST:exist
    @n  MOVE_OR_NO_BODY:move or no body
    @n  EXIST_OR_NO_BODY:exist or no body
    @n  MOVE_OR_EXIST:move or exist
    @n  PIN_ERROR:pin error
    '''
    ret = self.PIN_ERROR
    if self._outpin == 255:
      ret = self.PIN_ERROR
      return ret

    input_state = GPIO.input(self._outpin)

    if self._out_mode == self.OUT_MODE1:
      if input_state == GPIO.HIGH:
        ret = self.MOVE
      else:
        ret = self.Exist_OR_NO_BODY
    elif self._out_mode == self.OUT_MODE2:
      if input_state == GPIO.HIGH:
        ret = self.EXIST
      else:
        ret = self.MOVE_OR_NO_BODY
    elif self._out_mode == self.OUT_MODE3:
      if input_state == GPIO.HIGH:
        ret = self.MOVE_OR_EXIST
      else:
        ret = self.NO_BODY

    return ret

  def restart(self):
    ret = 0
    send_data = [0x00] * 10
    data_len = 5

    send_data[0] = CMD_RESTART
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = 0x00

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = 0
    else:
      ret = -1
    return ret

  def get_out_mode(self):
    send_pack = [0x00] * 10
    data_len = 4
    ret = False

    send_pack[0] = CMD_CONFIG_OUT_MODE
    send_pack[1] = self.READ_AND_WRITE_REQ
    send_pack[2] = data_len >> 0 & 0xFF
    send_pack[3] = data_len >> 8 & 0xFF

    self.send_pack(send_pack, data_len, FRAME_TYPE_READ_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      self._out_mode = recv_data.data[0]
      ret = True
    else:
      ret = False
    return ret

  def get_resolution_mode(self):
    send_pack = [0x00] * 10
    data_len = 4
    ret = False

    send_pack[0] = CMD_GET_AND_SET_RESOLUTION_MODE
    send_pack[1] = self.READ_AND_WRITE_REQ
    send_pack[2] = data_len >> 0 & 0xFF
    send_pack[3] = data_len >> 8 & 0xFF

    self.send_pack(send_pack, data_len, FRAME_TYPE_READ_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      self._resolution_mode = recv_data.data[0]
      ret = True
    else:
      ret = False
    return ret

  def set_led(self, run_led, out_led):
    send_data = [0x00] * 10
    data_len = 6
    ret = False

    send_data[0] = CMD_SET_LED_MODE
    send_data[1] = self.READ_AND_WRITE_REQ
    send_data[2] = data_len >> 0 & 0xFF
    send_data[3] = data_len >> 8 & 0xFF
    send_data[4] = run_led & 0xFF
    send_data[5] = out_led & 0xFF

    self.send_pack(send_data, data_len, FRAME_TYPE_WRITE_REQUSET)

    recv_data = self.recv_pack()

    if recv_data.result_pon_code == self.SUCCEED:
      ret = True
    else:
      ret = False
    return ret

  def send_pack(self, send_data, data_len, msg_type):
    send_pack = [0x00] * 50
    pack_len = 0x0000
    checksum = 0x0000
    send_pack[0] = FRAME_HEADER1
    pack_len += 1
    send_pack[1] = FRAME_HEADER2
    pack_len += 1
    send_pack[2] = FRAME_HEADER3
    pack_len += 1
    send_pack[3] = FRAME_HEADER4
    pack_len += 1
    temp = data_len + 10
    send_pack[4] = temp >> 0 & 0xFF
    pack_len += 1
    send_pack[5] = temp >> 8 & 0xFF
    pack_len += 1
    send_pack[6] = 0x00
    pack_len += 1
    send_pack[7] = msg_type
    pack_len += 1

    send_pack[pack_len : pack_len + data_len] = send_data
    pack_len += data_len
    checksum = self.get_check_sum(send_pack, pack_len)

    send_pack[pack_len] = checksum >> 0 & 0xFF
    pack_len += 1
    send_pack[pack_len] = checksum >> 8 & 0xFF
    pack_len += 1

    self.ser.write(send_pack[:pack_len])

  def recv_pack(self):
    recv_dat = RecvPack()
    recv_pack = [0x00] * 60

    recv_pack = self.read_reg(0, 8)

    if recv_pack[0] == FRAME_HEADER1 and recv_pack[1] == FRAME_HEADER2 and recv_pack[2] == FRAME_HEADER3 and recv_pack[3] == FRAME_HEADER4:
      packlen = (recv_pack[5] << 8) | recv_pack[4]

      recv_pack += bytearray(self.read_reg(0, packlen - 8))

      if len(recv_pack) == packlen:
        if self.check_sum(recv_pack, packlen):
          recv_dat.pack_type = recv_pack[7]
          recv_dat.data_header.cmd = recv_pack[8]
          recv_dat.data_header.resp_code = recv_pack[9]
          recv_dat.data_header.data_len = recv_pack[10] | (recv_pack[11] << 8)
          payload = recv_pack[12 : 12 + recv_dat.data_header.data_len]
          n = min(len(payload), 50)
          for i in range(50):
            recv_dat.data[i] = 0
          for i in range(n):
            recv_dat.data[i] = payload[i]
          recv_dat.result_pon_code = recv_dat.data_header.resp_code

          if not (recv_dat.pack_type == FRAME_TYPE_WRITE_RESPOND or recv_dat.pack_type == FRAME_TYPE_READ_RESPOND or recv_dat.pack_type == FRAME_TYPE_NOTIFICATION):
            recv_dat.result_pon_code = self.CMD_ERR
        else:
          recv_dat.result_pon_code = self.AUTHENTICATION_ERR
      else:
        recv_dat.result_pon_code = self.DATA_LEN_ERR
    else:
      recv_dat.result_pon_code = self.AUTHENTICATION_ERR
    return recv_dat

  def get_check_sum(self, send_data, data_len):
    parity = 0x0000
    for i in range(data_len):
      parity += send_data[i]
    return parity & 0xFFFF

  def check_sum(self, recv_data, data_len):
    parity = 0x0000
    for i in range(data_len - 2):
      parity += recv_data[i]
    temp = (recv_data[data_len - 1] << 8) | recv_data[data_len - 2]

    if temp == parity:
      return True
    return False

  def wrirte_reg(self, reg, data, length):
    send = bytes(data[:length])
    self.ser.flushInput()
    try:
      self.ser.write(send)
      return
    except:
      print("please check connect or mode!")
    return

  def read_reg(self, reg, length):
    recv = [0x00] * length
    timenow = time.time()
    while (time.time() - timenow) <= 0.1:
      count = self.ser.inWaiting()
      if count != 0:
        recv = self.ser.read(length)
        return recv
    return recv
