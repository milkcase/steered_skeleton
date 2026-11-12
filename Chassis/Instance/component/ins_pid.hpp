/**
 *******************************************************************************
 * @file      :ins_pid.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INSTANCE_INS_PID_HPP_
#define CHASSIS_INSTANCE_INS_PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include "pid.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hello_world {
namespace pid {
class Pid;
}  // namespace pid
}  // namespace hello_world
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* yaw 实接共享 CAN2，位置环由底盘板执行，保持与电机实例的责任归属一致。 */
hello_world::pid::Pid *GetPidMotorYaw();
/* 获取第 idx 号轮电机速度环 PID 单例（idx ∈ [0,4)）。 */
hello_world::pid::Pid *GetPidMotorWheel(int idx);
/* 获取第 idx 号舵电机角度环 PID 单例（idx ∈ [0,4)）。 */
hello_world::pid::Pid *GetPidMotorSteer(int idx);

#endif /* CHASSIS_INSTANCE_INS_PID_HPP_ */
