#ifndef STEERED_GIMBAL_CONTROL_CONFIG_HPP_
#define STEERED_GIMBAL_CONTROL_CONFIG_HPP_
#include "gimbal_controller.hpp"
namespace gimbal_control_config {
// 两轴力矩及目标变化率默认零；PID 参数仍由第三项 pid_config.hpp 单独维护。
inline constexpr robot::Gimbal::Params kYaw{};
// 考核说明未给 pitch 安全范围，确认前 false，不能用电机编码量程 ±π 代替机械限位。
// 将用户确认的静态安全区间换算到本次水平零位；仅确认范围，不放行力矩或目标变化率。
inline constexpr robot::Gimbal::Params kPitch = [] {
  robot::Gimbal::Params params;
  params.pitch_limits_confirmed = true;
  params.pitch_min_rad = -0.419f;
  params.pitch_max_rad = 0.515f;
  // 其余默认值保留：力矩和目标变化率均为0，边界保护区为0.03rad。
  return params;
}();
}
#endif
