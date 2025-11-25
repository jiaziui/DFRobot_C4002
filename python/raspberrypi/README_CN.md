# DFRobot_C4002
- [English Version](./README.md)

这是⼀款侧装运动检测11m，静⽌检测10m的24Ghz毫⽶波测距雷达传感器（顶装运动检测范围直径11m，静⽌检测10m），同时具备靠近远离检测功能、区域分区检测功能、环境底噪采集功能，板载光线检测传感器。此传感器适⽤于智能家居应⽤场景。

![svg]()

## 产品链接（）

    SKU：SEN0691

## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)

## 概述

- 支持量程为0-11m
- 支持5V主控器
- 支持串口通信
- 支持out引脚输出检测结果
- 支持out引脚输出模式设置
- 支持环境底噪采集
- 支持光照强度检测
- 支持距离门使能和阀值设置
- 支持汇报周期设置
- 支持环境光照阈值设置
- 支持80cm和20cm分辨率设置
- 支持获取目标状态以及相关数据

## 库安装
使用此库前，请首先下载库文件，将其粘贴到树莓派的自定义目录中，然后打开examples文件夹并在该文件夹中运行演示。

## 方法

```python
  def begin(self,outpin = 255):
    '''!
      @brief begin
      param outpin: out引脚，默认为255，即不输出检测结果，可根据自己的需求进行设定
      @return True or False
    '''

  def set_run_led(self, run_led):
    '''!
      @brief set run led
      @param switching LED_OFF:关闭 LED_ON:开启 LED_KEEP:保持原样
      @return True or False
    '''

  def set_out_led(self, out_led):
    '''!
      @brief 设置out引脚输出状态
      @param outled LED_OFF:关闭 LED_ON:开启 LED_KEEP:保持原样
      @return True or False
    '''

  def set_out_mode(self, out_mode):
    '''!
      @brief 设置out模式
      @param out_mode
      @n  OUT_MODE1:只有目标运动时输出高电平。
      @n  OUT_MODE2:只有目标存在时输出高电平。
      @n  OUT_MODE3:只有目标存在或运动时输出高电平。
      @return True or False
    '''

  def start_env_calibration(self,delay_time,cont_time):
    '''!
      @brief 开始底噪校准
      @param delay_time: 延时时间.range:0-65535s
      @param cont_time : 持续采样校准时间.range:15-65535s
    '''

  def set_detect_range(self, closet_distance, farthest_distance):
    '''!
      @brief 设置检测范围,单位:cm,范围:0-1100cm
      @param closet_distance   : 最近距离
      @param farthest_distance : 最远距离
      @return True or False
    '''


  def enable_distance_door(self, door_type, door_data):
    '''!
      @brief 使能距离门
      @param door_type: 距离门类型
      @n              MOVE_DISTANCE_DOOR:运动距离门
      @n              EXIST_DISTANCE_DOOR:存在距离门
      @param door_data：距离门状态数据，这是一个数组，数组的长度取决于分辨率模式
      @n              80cm模式下，可设置的门为15个，20cm模式下，可设置的门为25个，
      @n              每个门对应数组的下标，从0开始
      @return True or False
    '''

  def factory_reset(self):
    '''!
      @brief 恢复出厂设置
      @return True or False
    '''

  def set_resolution_mode(self, resolution_mode):
    '''!
      @brief 设置分辨率模式
      @param resolution_mode: 分辨率模式
      @n              RESOLUTION_80CM:80cm
      @n              RESOLUTION_20CM:20cm
      @return True or False
    '''

  def set_report_period(self, period):
    '''!
      @brief 设置上报周期
      @param period 上报周期，单位：0.1s
      @return True or False
    '''

  def set_light_threshold(self, threshold):
    '''!
      @brief 设置环境光照阈值
      @param threshold 环境光照阈值
      @return True or False
    '''

  def set_distance_door_threshold(self,door_type , threshold):
    '''!
      @brief 设置距离门阈值
      @param door_type: 距离门类型
      @n              MOVE_DISTANCE_DOOR:运动距离门
      @n              EXIST_DISTANCE_DOOR:存在距离门
      @param threshold: 阈值，这是一个数组，数组的长度取决于分辨率模式
      @n                ，80cm模式下，可设置的门为15个，20cm模式下，
      @n                可设置的门为25个
      @return True or False
    '''

  def set_baudrate(self, baudrate):
    '''!
      @brief 设置波特率
      @param baudrate 波特率
      @return True or False
    '''

  def get_note_info_loop(self):
    '''!
      @brief 循环获取周期上报的目标信息
    '''

  def get_target_state(self):
    '''!
      @brief 得到目标状态
      @return 目标状态
      @n     NO_BODY:no body
      @n     EXIST:exist
      @n     MOVE:move
    '''

  def get_exist_target_info(self):
    '''!
      @brief 得到存在目标信息
      @return 存在目标信息
      @n          distance:距离,单位:m
      @n          energy:能量,范围:0-100
    '''

  def get_move_target_info(self):
    '''!
      @brief 得到运动目标信息
      @return 运动目标信息
      @n          distance:运动距离,单位:m
      @n          speed:运动速度,单位:m/s
      @n          energy:运动能量,范围:0-100
      @n          direction:方向信息
      @n              AWAY:远离
      @n              STAY:无方向
      @n              NEAR:靠近
    '''

  def get_version_info(self,version):
    '''!
      @brief 获取版本信息
      @param 版本类型
      @n  VERSION_SOFTWARE:固件版本
      @n  VERSION_HARDWARE:硬件版本
      @return 字符串版本号
    '''

  def get_out_target_state(self):
    '''!
      @brief 得到out引脚输出状态
      @return 目标状态
      @n  NO_BODY:没有目标
      @n  MOVE:目标运动
      @n  EXIST:目标存在
      @n  MOVE_OR_NO_BODY:运动或没有目标
      @n  EXIST_OR_NO_BODY:存在或没有目标
      @n  MOVE_OR_EXIST:运动或存在目标
      @n  PIN_ERROR:引脚错误
    '''
```

## 兼容性

* RaspberryPi Version

| Board        | 正常运行  | 运行失败   | 未测试    | 备注
| ------------ | :-------: | :--------: | :------: | :-----: |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python版本

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |           |            |    √     |         |


## 历史

- 2025/11/04 - V1.0.0 版本

## 创作者

Written by JiaLi(zhixin.liu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
