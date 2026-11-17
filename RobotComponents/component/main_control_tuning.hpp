#pragma once
#include "robot_tuning.hpp"
// 默认主程序开关；历史回归显式置0，正常固件默认启用，动作仍受Working/反馈/IMU门控。
#ifndef ROBOT_NORMAL_MAIN_ENABLED
#if (defined(ROBOT_MOTOR_DEBUG_POWER_ENABLED) && ROBOT_MOTOR_DEBUG_POWER_ENABLED) || \
    (defined(ROBOT_MOTOR_DEBUG_PREVIEW_ENABLED) && ROBOT_MOTOR_DEBUG_PREVIEW_ENABLED)
#define ROBOT_NORMAL_MAIN_ENABLED 0
#else
#define ROBOT_NORMAL_MAIN_ENABLED 1
#endif
#endif
// 主程序模式与单位常量；所有可调整的电机数值都在robot_tuning.hpp。
namespace main_tuning {
inline constexpr bool enabled=ROBOT_NORMAL_MAIN_ENABLED==1;
inline constexpr float rpm=0.10471975511965977f;
}
