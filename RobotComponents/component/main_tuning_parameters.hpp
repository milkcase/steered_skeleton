#pragma once
#include "main_tuning_link.hpp"
#include "robot_control_config.hpp"
#include "gimbal_control_config.hpp"
namespace main_tuning {
// 从唯一调参文件间接取得工程单位值，发到云台的正是底盘本次编译参数，而非云台旧宏。
inline Bundle parameters() {
  const auto &r=robot_control::kControl;const auto &p=pid_config::kPitch;const auto &g=gimbal_control_config::kPitch;
  return {{r.max_vx_m_s,r.max_vy_m_s,r.yaw_rate_rad_s,r.pitch_rate_rad_s,r.aim_acceleration_rad_s2,
    p.kp,p.ki,p.kd,p.integral_limit,p.output_limit,g.speed_pid.kp,g.speed_pid.ki,g.speed_pid.kd,g.speed_pid.integral_limit,
    g.max_torque_nm,g.max_target_rate_rad_s,g.actual_speed_limit_rad_s,g.pitch_min_rad,g.pitch_max_rad,g.pitch_guard_rad,g.pitch_position_limits_enabled?1.0f:0.0f,
    g.gravity_ff_enabled?1.0f:0.0f,g.gravity_cos_nm,g.gravity_sin_nm,g.gravity_bias_nm,g.gravity_fit_min_rad,g.gravity_fit_max_rad}};
}
}
