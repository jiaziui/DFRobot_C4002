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
Download the library file before use, paste it into the custom directory for Raspberry Pi, then open the examples folder and run the demo in the folder.

## Methods

```python
  def begin(self,outpin = 255):
    '''!
      @brief begin
      param outpin: output pin, default is 255, which means no output pin is used.
      @return True or False
    '''

  def set_run_led(self, run_led):
    '''!
      @brief set run led
      @param switching LED_OFF:off LED_ON:on LED_KEEP:keep
      @return True or False
    '''

  def set_out_led(self, out_led):
    '''!
      @brief set out led
      @param outled LED_OFF:off LED_ON:on LED_KEEP:keep
      @return True or False
    '''

  def set_out_mode(self, out_mode):
    '''!
      @brief set output mode
      @param out_mode
      @n  OUT_MODE1:Only when motion is detected will a high level be output.
      @n  OUT_MODE2:A high level is output only when its presence is detected.
      @n  OUT_MODE3:A high level only appears when movement or presence is detected.
      @return True or False
    '''

  def start_env_calibration(self,delay_time,cont_time):
    '''!
      @brief start environment calibration
      @param delay_time: delay time.range:0-65535s
      @param cont_time : continuous time.range:15-65535s
    '''

  def set_detect_range(self, closet_distance, farthest_distance):
    '''!
      @brief set detect range,unit:cm,range:0-1100cm
      @param closet_distance   : recent distance
      @param farthest_distance : farthest distance
      @return True or False
    '''


  def enable_distance_door(self, door_type, door_data):
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

  def factory_reset(self):
    '''!
      @brief factory reset,Restart takes effect
      @return True or False
    '''

  def set_resolution_mode(self, resolution_mode):
    '''!
      @brief set resolution mode
      @param resolution_mode: resolution mode
      @n              RESOLUTION_80CM:80cm
      @n              RESOLUTION_20CM:20cm
      @return True or False
    '''

  def set_report_period(self, period):
    '''!
      @brief set report period
      @param period report period,unit:0.1s
      @return True or False
    '''

  def set_light_threshold(self, threshold):
    '''!
      @brief set light threshold
      @param threshold light threshold
      @return True or False
    '''

  def set_distance_door_threshold(self,door_type , threshold):
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

  def set_baudrate(self, baudrate):
    '''!
      @brief set baudrate
      @param baudrate baudrate
      @return True or False
    '''

  def get_note_info_loop(self):
    '''!
      @brief get note data info loop
    '''

  def get_target_state(self):
    '''!
      @brief get target state
      @return detect result
      @n    NO_BODY:no body
      @n    EXIST:exist
      @n    MOVE:move
    '''

  def get_exist_target_info(self):
    '''!
      @brief get exist target info
      @return ExistTarget object
      @n          distance:distance,unit:cm
      @n          energy:energy,range:0-100
    '''

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

  def get_version_info(self,version):
    '''!
      @brief get version info
      @param version version type
      @n  VERSION_SOFTWARE:software version
      @n  VERSION_HARDWARE:hardware version
      @return string info
    '''

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
```

## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |     √     |            |          |         |
| RaspberryPi4 |           |            |    √     |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |           |            |    √     |         |


## History

- 2025/11/04 - V1.0.0 version

## Credits

Written by JiaLi(zhixin.liu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
