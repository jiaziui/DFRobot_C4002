# -*- coding: utf-8 -*
'''!
@file factory_reset.py
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


def setup():
  while c4002.begin() != True:
    print("C4002 begin faild!")
    time.sleep(1)

  print("C4002 begin success!")
  time.sleep(0.05)

  # Query the versions of software and hardware
  software_version = c4002.get_version_info(SOFTWARE_VERSION)
  print("Software version: ", software_version)
  time.sleep(0.05)

  hardware_version = c4002.get_version_info(HARDWARE_VERSION)
  print("Hardware version: ", hardware_version)

  time.sleep(2)

  # Restore factory settings
  print("Restore factory settings...")
  if c4002.factory_reset():
    print("Factory reset succeed!")
    print("After restoring the factory Settings, a restart is required!")
  else:
    print("Factory reset failed!")


def loop():
  time.sleep(1)


if __name__ == "__main__":
  setup()
  while True:
    loop()
