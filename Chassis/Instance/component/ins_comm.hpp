/**
 *******************************************************************************
 * @file      :ins_comm.hpp
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
#ifndef CHASSIS_INSTANCE_INS_COMM_HPP_
#define CHASSIS_INSTANCE_INS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
#include "dbus_receiver.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取遥控器解码对象（DT7）单例。 */
hello_world::remote_control::DT7 *GetRemoteControl();

/* 仅用于接收层接线，不能把取得此对象当作遥控器已经在线。 */
remote_input::DbusReceiver *GetRemoteControlReceiver();
/* 控制层读取入口：原子复制快照；返回 false 时输入无效，禁止驱动电机。 */
bool ReadRemoteControl(hello_world::remote_control::DT7 &snapshot);

#endif /* CHASSIS_INSTANCE_INS_COMM_HPP_ */
