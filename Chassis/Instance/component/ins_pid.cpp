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
// 返回指定轮位独立的速度环 PID；下标须在 0～3，越界返回空指针。
hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  /* TODO【新人任务】:
   * 返回对应的 PID 单例。
   *
   * 你要做：
   *   1. 先自己实现一个 PID 类（位置式 / 增量式，含 Kp/Ki/Kd、积分限幅、输出限幅），
   *      建议单独新建文件，类名放到 hello_world::pid 命名空间并命名为 Pid（或改这里
   *      的返回类型以匹配你的类名）。
   *   2. 静态定义各控制回路的 PID 对象并整定参数（考核会检查稳 / 快 / 无超调）。
   */
  // 越界索引不访问数组；每个回路均有独立的积分与历史误差。
  if (idx < 0 || idx >= 4) return nullptr;
  static hello_world::pid::Pid controllers[4] = {
      hello_world::pid::Pid(pid_config::kWheel[0]),
      hello_world::pid::Pid(pid_config::kWheel[1]),
      hello_world::pid::Pid(pid_config::kWheel[2]),
      hello_world::pid::Pid(pid_config::kWheel[3]),
  };
  return &controllers[idx];
}

// 返回指定轮位独立的舵角环 PID；下标须在 0～3，越界返回空指针。
hello_world::pid::Pid *GetPidMotorSteer(int idx) {
  /* TODO【新人任务】:
   * 返回第 idx 号舵电机角度环 PID 单例，做法同 GetPidMotorWheel()。
   */
  // 越界索引不访问数组；每个回路均有独立的积分与历史误差。
  if (idx < 0 || idx >= 4) return nullptr;
  static hello_world::pid::Pid controllers[4] = {
      hello_world::pid::Pid(pid_config::kSteer[0]),
      hello_world::pid::Pid(pid_config::kSteer[1]),
      hello_world::pid::Pid(pid_config::kSteer[2]),
      hello_world::pid::Pid(pid_config::kSteer[3]),
  };
  return &controllers[idx];
}

/* Private function definitions ----------------------------------------------*/

// 返回底盘板负责的 yaw 位置环 PID 单例；仅计算数值，不直接控制电机。
hello_world::pid::Pid *GetPidMotorYaw() {
  // 共线不等于重复控制：yaw 的位置环和电机实例都只在底盘板执行。
  static hello_world::pid::Pid controller(pid_config::kYaw);
  return &controller;
}
