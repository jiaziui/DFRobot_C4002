# -*- coding: utf-8 -*
'''!
@file get_all_results.py
@brief This is an example to show how to use the DFRobot_C4002 library to get all the results of the C4002 sensor.
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
  print("C4002 begin success!")
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
  time.sleep(0.05)

  if c4002.set_resolution_mode(c4002.RESOLUTION_80CM):
    print("Set resolution mode success!")
  else:
    print("Set resolution mode faild!")
  time.sleep(0.05)
  # Note
  ## 1.eResolution80Cm: This indicates that the resolution of the "distance door" is 80cm.
  ## With a resolution of 80 cm, it supports up to 15 distance doors, with a maximum distance of 11.6 meters.
  ## 2.eNoteInfoCalibration: This indicates that the calibration is in progress.
  ## With a resolution of 20 cm, it supports up to 25 distance doors, with a maximum distance of 4.9 meters

  # Set the detect range to 0-1000 cm
  clost_range = 0
  far_range = 1000
  if c4002.set_detect_range(clost_range, far_range):  # Max detect range(0-1200cm)
    print("Set detect range success!")
  else:
    print("Set detect range faild!")
  time.sleep(0.05)

  # Enable the 'distance door'
  ## Resolution mode:eResolution80Cm,This means that the number of 'distance doors' we can operate is 15
  door_enable = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]  # 15 doors, disenable :10,11,12,13,14  enable:0-9
  ## Resolution mode:eResolution20Cm,This means that the number of 'distance doors' we can operate is 25
  ## door_enable = [1, 1, 1, 1, 1,...1,1,1] 25 doors

  if c4002.enable_distance_door(c4002.MOVE_DISTANCE_DOOR, door_enable):  # Operation move distance door
    print("Enable move distance door succeed!")
  else:
    print("Enable move distance door succeed!")
  time.sleep(0.05)
  if c4002.enable_distance_door(c4002.EXIST_DISTANCE_DOOR, door_enable):  # Operation exist distance door
    print("Enable exist distance door succeed!")
  else:
    print("Enable exist distance door succeed!")
  time.sleep(0.05)

  # Set the light threshold to 0 lux.range: 0-50 lux
  if c4002.set_light_threshold(0):
    print("Set light threshold success!")
  else:
    print("Set light threshold faild!")
  time.sleep(0.05)

  # Set distance door threshold to 50 ,range: 0-99
  ## Resolution mode:eResolution80Cm,This means that the number of 'distance doors' we can operate is 15
  exist_threshold = [50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50]  # 15 doors
  move_threshold = [50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50]  # 15 doors
  ## Resolution mode:eResolution20Cm,This means that the number of 'distance doors' we can operate is 25
  ## exist_threshold = [50, 50, 50,..., 50, 50, 50] #25 doors
  ## move_threshold  = [50, 50, 50,..., 50, 50, 50] #25 doors
  if c4002.set_distance_door_threshold(c4002.EXIST_DISTANCE_DOOR, exist_threshold):
    print("Set exist distance door threshold succeed!")
  else:
    print("Set exist distance door threshold failed!")
  time.sleep(0.05)
  if c4002.set_distance_door_threshold(c4002.MOVE_DISTANCE_DOOR, move_threshold):
    print("Set move distance door threshold succeed!")
  else:
    print("Set move distance door threshold failed!")
  time.sleep(0.05)

  # Set the report period to 1s
  if c4002.set_report_period(10):
    print("Set report period succeed!")
  else:
    print("Set report period failed!")
  time.sleep(0.05)
  ## note: Calibration and obtaining all data must have a set cycle


def loop():
  # Obtain the calibration results
  ret_result = c4002.get_note_info_loop()

  if ret_result.note_type == c4002.NOTE_INFO_RESULT:
    print("------- Get all results --------")
    # get the light intensity
    light = c4002.get_light()
    print("Light: ", round(light, 2), " lux")

    # get Target state
    target_state = c4002.get_target_state()
    print("Target state: ", end="")
    if target_state == c4002.NO_BODY:
      print("No body")
    elif target_state == c4002.EXIST:
      print("Exist")
    elif target_state == c4002.MOVE:
      print("Move")

    # get exist distance door index
    exist_door_index = c4002.get_exist_dist_indux()
    print("Exist distance door index: ", end="")
    for i in range(32):
      if exist_door_index & (1 << i):
        print(i, end=" ")
    print()

    # get exist distance door target info
    exist_target = ExistTarget()
    exist_target = c4002.get_exist_target_info()
    print("exist distance: ", round(exist_target.distance, 2), " m")
    print("exist energy: ", exist_target.energy)

    # get move distance door index
    move_target = MoveTarget()
    move_target = c4002.get_move_target_info()
    print("move distance: ", round(move_target.distance, 2), " m")
    print(
      "move energy: ",
      move_target.energy,
    )
    print("move speed: ", round(move_target.speed, 2), " m/s")
    print("move direction: ", end="")
    if move_target.direction == c4002.AWAY:
      print("Away.")
    elif move_target.direction == c4002.NEAR:
      print("Approach.")
    elif move_target.direction == c4002.STAY:
      print("Directionless.")
    else:
      print("Unknown direction.")

    print("--------------------------------")


if __name__ == "__main__":
  setup()
  while True:
    loop()
