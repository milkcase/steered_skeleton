/**
 *******************************************************************************
 * @file      :ins_robot.cpp
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
#include "ins_robot.hpp"
// 隔离顶层硬件依赖，保留本文件原有模块 getter 的独立可编译性。
robot::Robot *GetRegisteredRobotInstance();
#include "gimbal.hpp"
#include "gimbal_control_config.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
// 返回云台顶层状态机单例，首次获取不触碰硬件或发送电机命令。
robot::Robot *GetRobot() {
  /* TODO【新人任务】:
   * 返回本板顶层状态机 robot::Robot 单例。
   *
   * 你要做：
   *   1. 在本文件静态定义一个 robot::Robot 对象（静态存储期，程序启动即构造）。
   *   2. 返回它的地址。
   *
   * 约束 / 提示：
   *   - robot::Robot 类的定义由你在 RobotModules/component/robot.hpp 补全（见该文件 TODO），
   *     本文件只负责实例化并返回地址。
   */
  return GetRegisteredRobotInstance();
}

// 返回云台板的 pitch 位置环；绑定已配置的 J4310 实例和独立 PID，不使能、不发送。
robot::Gimbal *GetGimbal() {
  /* TODO【新人任务】:
   * 返回 Gimbal 模块 robot::Gimbal 单例，做法同 GetRobot()。
   * robot::Gimbal 类定义由你在 RobotModules/component/gimbal.hpp 补全。
   */
  static robot::Gimbal gimbal(robot::Gimbal::Axis::Pitch,
      static_cast<hello_world::motor::DaMiao *>(GetMotorPitch()),GetPidMotorPitch(),gimbal_control_config::kPitch);
  return &gimbal;
}

/* Private function definitions ----------------------------------------------*/
