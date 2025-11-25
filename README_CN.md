# DFRobot_C4002
- [English Version](./README.md)

这是⼀款侧装运动检测11m，静⽌检测10m的24Ghz毫⽶波测距雷达传感器（顶装运动检测范围直径11m，静⽌检测10m），同时具备靠近远离检测功能、区域分区检测功能、环境底噪采集功能，板载光线检测传感器。此传感器适⽤于智能家居应⽤场景。

![svg]


## 产品链接（）

    SKU：SEN0691

## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性y)
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
这里提供两种使用本库的方法：
1.打开Arduino IDE,在状态栏中的Tools--->Manager Libraries 搜索"DFRobot_C4002"并安装本库.
2.首先下载库文件,将其粘贴到\Arduino\libraries目录中,然后打开examples文件夹并在该文件夹中运行演示.

## 方法

```C++
/**
 * @fn begin
 * @brief 初始化串口并设置输出引脚.
 * @param outPin: 输出引脚, 默认255, 即不使用输出引脚.
 * @return true: 初始化成功. false: 初始化失败.
 */
  bool begin(uint8_t outPin = 255);
  /**
   * @fn setRunLed
   * @brief 设置run Led状态.
   * @param switching
   *      eLedOff: 关闭run LED.
   *      eLedOn : 打开run LED.
   * @return true: 操作LED成功. false: 操作LED失败.
  */
  bool setRunLed(eLedMode_t switching);

  /**
   * @fn setOutLed
   * @brief 设置输出Led状态.
   * @param switching
   * @n     eLedOff: 关闭输出LED.
   * @n     eLedOn : 打开输出LED.
   * @return true: 输出LED成功. false: 输出LED失败.
  */
  bool setOutLed(eLedMode_t switching);

  /**
   * @fn setOutMode
   * @brief 设置out脚输出模式
   * @param outMode
   * @n      eOutMode1: 只有当检测到运动时，才会输出高电平。
   * @n      eOutMode2: 只有在检测到存在时，才会输出高电平。
   * @n      eOutMode3: 当检测到运动或存在时，都会输出高电平。
   * @return true: 设置输出模式成功. false: 设置输出模式失败.
  */
  bool setOutMode(eOutMode_t outMode);

  /**
   * @fn startEnvCalibration
   * @brief 开始底噪校准
   * @param delayTime: 延迟开始执行校准采样的时间, 单位: s
   * @param contTime : 校准持续时间, 单位: s
   * @return true: 开始校准成功. false: 开始校准失败.
  */
  void startEnvCalibration(uint16_t delayTime ,uint16_t contTime);

  /**
   * @fn setDetect范围
   * @brief 设置检测距离范围,范围: 0-1200cm
   * @param closest : 最短距离, 单位: cm
   * @param farthest: 最长距离, 单位: cm
   * @return true: 设置检测范围成功. false: 设置检测范围失败.
  */
  bool setDetect范围(uint16_t closest,uint16_t farthest);

  /**
   * @fn enableDistanceDoor
   * @brief 使能/失能距离门
   * @param doorType
   * @n      eMoveDistDoor : 设置运动距离门参数时使用此参数
   * @n      eExistDistDoor: 设置存在距离门参数时使用此参数
   * @param doorData: 需要一个uint8t类型的数组，参数0和1分别表示禁用和启用距离门
   * @return true: 操作成功. false: 操作失败.
  */
  bool enableDistanceDoor(eDistanceDoorType_t doorType, uint8_t *doorData);

  /**
   * @fn factoryReset
   * @brief 恢复默认设置
   * @return true: 恢复默认设置成功. false: 恢复默认设置失败.
   */
  bool factoryReset(void);

  /**
   * @fn setResolutionMode
   * @brief 设置分辨率模式
   * @param mode : 分辨率模式, eResolution80Cm: 80cm, eResolution20Cm: 20cm
   * @return true: 设置分辨率模式成功. false: 设置分辨率模式失败.
  */
  bool setResolutionMode(eResolutionMode_t mode);

  /**
   * @fn setReportPeriod
   * @brief 设置数据汇报周期
   * @param period : 数据汇报周期, 范围: 0-255, 单位: 0.1s
   * @return true: 设置数据汇报周期成功, false: 设置数据汇报周期失败.
  */
  bool setReportPeriod(uint8_t period);

  /**
   * @fn setLightThreshold
   * @brief 设置光照阈值
   * @param threshold : 光照阈值, 范围: 0-50, 单位: lux
   * @return true: 设置光照阈值成功. false: 设置光照阈值失败.
   */
  bool setLightThreshold(float threshold);

  /**
   * @fn setDistanceDoorThreshold
   * @brief 设置距离门阈值
   * @param doorType
   * @n      eMoveDistDoor : 设置运动距离门参数时使用此参数
   * @n      eExistDistDoor: 设置存在距离门参数时使用此参数
   * @param threshold : 需要一个uint8t类型的数组，参数0-99表示距离门的阈值
   * @return true: 设置距离门阈值成功. false: 设置距离门阈值失败.
  */
  bool setDistanceDoorThreshold(eDistanceDoorType_t doorType,uint8_t * threshold);

  /**
   * @fn setBaudrate
   * @brief 设置串口波特率
   * @param baud : 波特率, 范围: 9600-115200, 单位: bps
   * @return true: 设置波特率成功. false: 设置波特率失败.
  */
  bool setBaudrate(uint32_t baud);

  /**
   * @fn getNotInfoLoop
   * @brief 得到通知信息
   * @return  sRetResult_t
   * @n           noteType      : 通知消息的类型. eNoteInfoResult: 数据信息通知消息. eNoteInfoCalibration: 底噪校准通知消息.
   * @n           calibCountdown: 环境校准剩余时间, 单位: s.
  */
  sRetResult_t getNotInfoLoop(void);

  /**
   * @fn getTargetState
   * @brief 获取目标状态
   * @n          eNobody        : 无目标.
   * @n          eExist         : 存在目标.
   * @n          eMove          : 目标移动.
  */
  eTargetState_t getTargetState(void);

  /**
   * @fn getLight
   * @brief 获取当前光照强度
   * @return 当前光照强度, 单位: lux.
  */
  float getLight(void);

  /**
   * @fn getExistDistIndex
   * @brief 获取存在目标的距离索引
   * @return 存在目标的距离索引, 范围: 0-25.
  */
  uint32_t getExistDistIndex(void);

  /**
   * @fn getExistTargetInfo
   * @brief 获取存在目标的信息
   * @return sExistTarget_t
   * @n        distance: 存在目标的距离, 单位: m.
   * @n        energy  : 存在目标的能量, 范围:0-99.
  */
  sExistTarget_t getExistTargetInfo(void);

  /**
   * @fn getMoveTargetInfo
   * @brief 获取移动目标的信息
   * @n        distance : 移动目标的距离, 单位: m.
   * @n        speed    : 移动目标的速度, 单位: m/s.
   * @n        energy   : 移动目标的能量, 范围:0-99.
   * @n        direction: 移动目标的方向, eMoveDirection_t.
   */
  sMoveTarget_t getMoveTargetInfo(void);

  /**
   * @fn getVersioninfo
   * @brief 获取传感器版本信息
   * @param version : 版本类型
   * @n      SOFTWARE_VERSION: 软件版本
   * @n      HARDWARE_VERSION: 硬件版本
   * @return String : 版本信息.
  */
  String getVersioninfo(uint8_t version);

  /**
   * @fn getOutTargetState
   * @brief 获取输出目标状态
   * @return eTargetState_t
   * @n          eNobody        : 没有输出目标.
   * @n          eExist         : 存在输出目标.
   * @n          eMove          : 输出目标移动.
   * @n          eMoveOrExist   : 输出目标移动或存在.
   * @n          eMoveOrNobody  : 输出目标移动或没有目标.
   * @n          eExistOrNobody : 输出目标存在或没有目标.
   * @n          ePinError      : 输出引脚错误.
  */
  eTargetState_t getOutTargetState(void);
```

## 兼容性

| 主板        | 通过 | 未通过 | 未测试 | 备注 |
| ----------- | :--: | :----: | :----: | ---- |
| Arduino uno |  √   |        |        |      |
| Mega2560    |  √   |        |        |      |
| Leonardo    |  √   |        |        |      |
| ESP32       |  √   |        |        |      |
| micro:bit   |      |        |   √    |      |


## 历史

- 2025/11/04 - V1.0.0 版本


## 创作者

Written by JiaLi(zhixin.liu@dfrobot.com), 2025. (Welcome to our [website](https://www.dfrobot.com/))
