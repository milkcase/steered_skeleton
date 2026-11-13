#ifndef STEERED_GIMBAL_CONTROL_CONFIG_HPP_
#define STEERED_GIMBAL_CONTROL_CONFIG_HPP_
#include "gimbal_controller.hpp"
namespace gimbal_control_config {
// 两轴力矩及目标变化率默认零；PID 参数仍由第三项 pid_config.hpp 单独维护。
inline constexpr robot::Gimbal::Params kYaw{};
// 考核说明未给 pitch 安全范围，确认前 false，不能用电机编码量程 ±π 代替机械限位。
inline constexpr robot::Gimbal::Params kPitch{};
}
#endif
