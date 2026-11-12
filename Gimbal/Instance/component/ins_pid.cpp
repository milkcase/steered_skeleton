/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"
#include "pid_config.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
// 云台板不创建 yaw 控制回路，返回空指针；实际位置环在底盘板执行。
hello_world::pid::Pid *GetPidMotorYaw() {
  /* TODO【新人任务】:
   * 返回对应的 PID 单例。
   *
   * 你要做：
   *   1. 先自己实现一个 PID 类（位置式 / 增量式，含 Kp/Ki/Kd、积分限幅、输出限幅），
   *      建议单独新建文件，类名放到 hello_world::pid 命名空间并命名为 Pid（或改这里
   *      的返回类型以匹配你的类名）。
   *   2. 静态定义各控制回路的 PID 对象并整定参数（考核会检查稳 / 快 / 无超调）。
   */
  // yaw 由底盘板执行，本板不创建第二个控制回路；后续经 CAN 发送目标。
  return nullptr;
}

// 返回云台板唯一的 pitch 位置环 PID，使用独立参数及历史状态。
hello_world::pid::Pid *GetPidMotorPitch() {
  /* TODO【新人任务】:
   * 返回 pitch 关节位置环 PID 单例，做法同 GetPidMotorYaw()。
   */
  // 仅创建纯计算对象，默认参数为零，不接入任何电机输出。
  static hello_world::pid::Pid controller(pid_config::kPitch);
  return &controller;
}

/* Private function definitions ----------------------------------------------*/
