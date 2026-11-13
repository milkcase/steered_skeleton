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
#include "chassis.hpp"
#include "ins_chassis_iksolver.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
// 返回底盘顶层状态机单例，首次获取仅构造内存状态，硬件由 MainTaskInit 注册。
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

// 返回只负责目标解算的底盘模块，绑定同一个逆解器；Robot 状态机仍留待第九项。
robot::Chassis *GetChassis() {
  /* TODO【新人任务】:
   * 返回 Chassis 模块 robot::Chassis 单例，做法同 GetRobot()。
   * robot::Chassis 类定义由你在 RobotModules/component/chassis.hpp 补全。
   */
  static robot::Chassis chassis(GetChassisIkSolver());
  return &chassis;
}

/* Private function definitions ----------------------------------------------*/
