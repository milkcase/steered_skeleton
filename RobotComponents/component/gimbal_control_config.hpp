#ifndef STEERED_GIMBAL_CONTROL_CONFIG_HPP_
#define STEERED_GIMBAL_CONTROL_CONFIG_HPP_
#include "gimbal_controller.hpp"
#include "pid_config.hpp"
namespace gimbal_control_config {
// 两轴限制统一从robot_tuning.hpp读取；本文件只换算为rad/s和模块参数。
inline constexpr robot::Gimbal::Params kYaw=[] {
  robot::Gimbal::Params p;p.speed_pid=pid_config::kYawSpeed;
  if(main_tuning::enabled) {p.max_torque_nm=ROBOT_YAW_TORQUE_NM;p.max_target_rate_rad_s=ROBOT_YAW_TARGET_RATE_RPM*main_tuning::rpm;
    p.max_speed_rad_s=pid_config::kYaw.output_limit;p.max_dt_s=0.003f;p.actual_speed_limit_rad_s=ROBOT_YAW_TRIP_RPM*main_tuning::rpm;}
  return p;
}();
// pitch使用已现场确认的软件安全区；机械行程不能由电机编码量程±π推断。
inline constexpr robot::Gimbal::Params kPitch = [] {
  robot::Gimbal::Params params;
  params.speed_pid=pid_config::kPitchSpeed;
  params.pitch_limits_confirmed = true;params.pitch_guard_rad=ROBOT_PITCH_GUARD_RAD;
  params.pitch_min_rad = ROBOT_PITCH_MIN_RAD;
  params.pitch_max_rad = ROBOT_PITCH_MAX_RAD;
  if(main_tuning::enabled) {params.max_torque_nm=ROBOT_PITCH_TORQUE_NM;
    params.max_target_rate_rad_s=ROBOT_PITCH_TARGET_RATE_RPM*main_tuning::rpm;
    params.max_speed_rad_s=pid_config::kPitch.output_limit;params.max_dt_s=0.003f;
    params.actual_speed_limit_rad_s=ROBOT_PITCH_TRIP_RPM*main_tuning::rpm;}
  // 历史回归关闭Main时保持零动力；云台主程序最终使用底盘下发的整组参数。
  return params;
}();
// Main两轴始终启用独立超速保护；填0不能绕过保护，旧零动力回归不受此断言影响。
static_assert(!main_tuning::enabled || (kYaw.actual_speed_limit_rad_s>kYaw.max_speed_rad_s &&
  kPitch.actual_speed_limit_rad_s>kPitch.max_speed_rad_s),"云台实际超速保护必须严格高于位置环输出速度上限");
}
#endif
