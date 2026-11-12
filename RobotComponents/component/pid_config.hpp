#ifndef HW_STEERED_PID_CONFIG_HPP_
#define HW_STEERED_PID_CONFIG_HPP_
#include "pid.hpp"

namespace pid_config {
using Params = hello_world::pid::Pid::Params;
inline constexpr float kAnglePeriodRad = 6.2831853071795864769f;
// 下标为左前、左后、右后、右前；每台电机使用独立 PID 状态，可独立整定参数。
// 尚未实车整定，增益、积分上限、输出上限均先为零，不填入未经验证的动力参数。
inline constexpr Params kWheel[4] = {};
inline constexpr Params kSteer[4] = {
    {.error_period = kAnglePeriodRad}, {.error_period = kAnglePeriodRad},
    {.error_period = kAnglePeriodRad}, {.error_period = kAnglePeriodRad},
};
// yaw 的闭环执行责任在底盘板；pitch 在云台板，pitch 使用有机械限位的线性误差。
inline constexpr Params kYaw{.error_period = kAnglePeriodRad};
inline constexpr Params kPitch{};
}  // namespace pid_config
#endif
