#ifndef HW_STEERED_PID_CONFIG_HPP_
#define HW_STEERED_PID_CONFIG_HPP_
#include "pid.hpp"

namespace pid_config {
using Params = hello_world::pid::Pid::Params;
inline constexpr float kAnglePeriodRad = 6.2831853071795864769f;
// 下标为左前、左后、右后、右前；每台电机使用独立 PID 状态，可独立整定参数。
// kWheel：轮速(rad/s)->C620原始值；kSteer：舵角(rad)->目标舵速(rad/s)。
// 尚未实车整定，增益、积分上限、输出上限均先为零，不填入未经验证的动力参数。
inline constexpr Params kWheel[4] = {};
inline constexpr Params kSteer[4] = {
    {.error_period = kAnglePeriodRad}, {.error_period = kAnglePeriodRad},
    {.error_period = kAnglePeriodRad}, {.error_period = kAnglePeriodRad},
};
// 转向位置环kSteer输出目标角速度(rad/s)，下面四个独立速度环输出电流(A)。
// 四轮可复制同一组已验收参数，但积分和微分历史由各自对象保存；普通固件仍默认零增益。
inline constexpr Params kSteerSpeed[4] = {};
// yaw 的闭环执行责任在底盘板；pitch 在云台板，pitch 使用有机械限位的线性误差。
inline constexpr Params kYaw{.error_period = kAnglePeriodRad};
inline constexpr Params kPitch{};
// yaw/pitch外环输出rad/s，以下速度内环输出Nm；由云台模块参数引用，仍默认零动力。
inline constexpr Params kYawSpeed{},kPitchSpeed{};
}  // namespace pid_config
#endif
