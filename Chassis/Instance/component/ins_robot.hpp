/**
 *******************************************************************************
 * @file      :ins_robot.hpp
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
#ifndef CHASSIS_INSTANCE_INS_ROBOT_HPP_
#define CHASSIS_INSTANCE_INS_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot {
class Robot;
class Chassis;
class Gimbal;
}  // namespace robot
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取本板顶层状态机单例（robot::Robot），对象在 ins_robot.cpp 中静态定义。 */
robot::Robot *GetRobot();
/* 获取底盘运动模块单例（robot::Chassis），负责底盘运动 / 工作模式逻辑。 */
robot::Chassis *GetChassis();
/* 取得底盘负责的 yaw 位置环对象；与云台板 pitch 对象分别执行，不跨板共享变量。 */
robot::Gimbal *GetGimbal();

#endif /* CHASSIS_INSTANCE_INS_ROBOT_HPP_ */
