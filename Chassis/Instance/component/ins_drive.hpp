#ifndef STEERED_INS_DRIVE_HPP_
#define STEERED_INS_DRIVE_HPP_
#include "drive_controller.hpp"
// 返回底盘独有的八电机闭环单例；创建不启动硬件，也不放行运动。
robot::DriveController *GetDriveController();
#endif
