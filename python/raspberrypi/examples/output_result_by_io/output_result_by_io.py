# -*- coding: utf-8 -*
'''!
@file output_result_by_io.py
@brief This example shows how to restore factory Settings.
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

# output pin,BCM coding
out_pin = 18
## Set the input pins of the Raspberry PI，BCM coding
## Connect them to the C4002 sensor out pins


def setup():
  while c4002.begin(out_pin) != True:
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

  # Set the output mode to output the detection result
  if c4002.set_out_mode(c4002.OUT_MODE3):
    print("Set out mode success!")
  else:
    print("Set out mode faild!")
  time.sleep(0.05)
  ## The output mode can be set to eOutMode1, eOutMode2, or eOutMode3.
  ## eOutMode1: A high level will be output when movement is detected.
  ## eOutMode2: A high level will be output when exit is detected.
  ## eOutMode3: A high level will be output when movement or exit is detected.


def loop():
  # Get the target state by the output pin
  target_state = c4002.get_out_target_state()

  # Print the target state
  print("Target state: ", end="")
  if target_state == c4002.NO_BODY:
    print("No body!")
  elif target_state == c4002.MOVE:
    print("Move!")
  elif target_state == c4002.EXIST:
    print("Exit!")
  elif target_state == c4002.MOVE_OR_EXIST:
    print("Move or exit!")
  elif target_state == c4002.MOVE_OR_NO_BODY:
    print("Move or no body!")
  elif target_state == c4002.EXIST_OR_NO_BODY:
    print("Exit or no body!")
  else:
    print("Pin error! please check the connection or pin number!")

  time.sleep(0.5)


if __name__ == "__main__":
  setup()
  while True:
    loop()
