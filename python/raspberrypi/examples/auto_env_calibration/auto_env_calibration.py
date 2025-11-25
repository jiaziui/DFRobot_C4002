# -*- coding: utf-8 -*
'''!
@file auto_env_calibration.py
@brief This demo can automatically calibrate the environmental parameters of the C4002 sensor.
@copyright Copyright (c) 2025 DFRobot Co.Ltd (http://www.dfrobot.com)
@license The MIT License (MIT)
@author [JiaLi](zhixin.liu@dfrobot.com)
@version V1.0
@date 2025-11-04
@url https://github.com/cdjq/DFRobot_C4002
'''

import time
import sys

sys.path.append("../../")
from DFRobot_C4002 import *

c4002 = DFRobot_C4002(115200)


def setup():
  while c4002.begin() != True:
    print("C4002 begin faild!")
    time.sleep(1)

  c4002.set_report_period(70)
  print("C4002 begin success!")
  time.sleep(0.05)

  # Query the versions of software and hardware
  software_version = c4002.get_version_info(SOFTWARE_VERSION)
  print("Software version: ", software_version)
  time.sleep(0.05)

  hardware_version = c4002.get_version_info(HARDWARE_VERSION)
  print("Hardware version: ", hardware_version)
  time.sleep(0.05)

  # Turn on the run led and out led
  if c4002.set_run_led(c4002.LED_ON):
    print("Set run led success!")
  else:
    print("Set run led faild!")
  time.sleep(0.05)
  if c4002.set_out_led(c4002.LED_ON):
    print("Set out led success!")
  else:
    print("Set out led faild!")

  time.sleep(3)

  # Set the report period to 1s
  if c4002.set_report_period(10):
    print("Set report period success!")
  else:
    print("Set report period faild!")
  ## note: Calibration and obtaining all data must have a set cycle

  # Start environmental calibration
  # delay_time：3s ，Calibration_time：15s( 15-65535 s )
  c4002.start_env_calibration(3, 15)
  print("Start environmental calibration:")

  # Note:
  ## 1. The calibration process takes about 15 seconds, and the delay time is 3 seconds.
  ## 2. When resetting the development board, please find an open area to calibrate it
  ## 3. When starting the calibration, there should be no one on either side of the sensor
  ## directly in front of the transmitter, otherwise it will affect the calibration accuracy
  ## of the sensor


def loop():
  # Obtain the calibration results
  ret_result = c4002.get_note_info_loop()

  if ret_result.note_type == c4002.NOTE_INFO_CALIBRATION:
    print("Calibration countdown:", ret_result.calibration_down, " s")
    if ret_result.calibration_down == 0:
      print("Calibration success!")
  time.sleep(0.3)


if __name__ == "__main__":
  setup()
  while True:
    loop()
