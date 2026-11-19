#ifndef STEERED_CHASSIS_IK_CONFIG_HPP_
#define STEERED_CHASSIS_IK_CONFIG_HPP_
#include "chassis_iksolver.hpp"
#include "main_control_tuning.hpp"
#include "drive_tuning.hpp"
#include "steer_limits.hpp"
namespace chassis_ik_config {
// Main采用统一调参文件的轮速/舵角变化率；关闭Main的历史回归仍保留零动力配置。
// 几何量固定读取已确认的 vehicle_motor_config.hpp，本文件不重复维护尺寸。
inline constexpr hello_world::chassis_ik_solver::ChassisIkSolver::Limits kLimits=[] {
  hello_world::chassis_ik_solver::ChassisIkSolver::Limits p;
  if(main_tuning::enabled) {p.max_wheel_speed_rad_s=robot_debug::drive::kMaxTargetSpeed;
    p.max_steer_rate_rad_s=robot_debug::steer_safety::kTargetRateRadS;p.max_dt_s=0.003f;p.project_drive_speed=true;}
  return p;
}();
}
#endif
