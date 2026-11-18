#ifndef STEERED_ROBOT_CONTROL_CONFIG_HPP_
#define STEERED_ROBOT_CONTROL_CONFIG_HPP_
#include <cmath>
#include <cstdint>
#include "pid.hpp"
#include "main_control_tuning.hpp"
#include "drive_debug_tuning.hpp"
#include "vehicle_motor_config.hpp"
namespace robot_control {
// 两板统一的遥控映射：右下急停、右中回中自检、右上运行；Main左上分离、左中跟随、左下小陀螺。
struct Params {
  bool allow_spin=true;
  bool start_level_pitch=false;
  float aim_acceleration_rad_s2=0;
  bool remote_mapping_confirmed=false;
  float max_vx_m_s=0, max_vy_m_s=0, max_w_rad_s=0;
  float yaw_rate_rad_s=0, pitch_rate_rad_s=0;
  float deadzone=0.05f;
  uint32_t neutral_hold_ms=200, enable_window_ms=1000;
  float max_dt_s=0.003f;
  hello_world::pid::Pid::Params follow{.error_period=6.283185307179586f};
  float follow_deadband_rad=0,follow_accel_rad_s2=0;
  float spin_target_rad_s=0,spin_max_rad_s=0,spin_accel_rad_s2=0;
};
// 从统一调参文件生成遥控映射上限；关闭Main时保留旧零动力配置供历史回归。
inline constexpr Params kControl = [] {
  Params params;
  params.remote_mapping_confirmed = true;
  if(main_tuning::enabled) {
    params.start_level_pitch=true;
    params.max_vx_m_s=params.max_vy_m_s=robot_debug::drive::kMaxTargetSpeed*vehicle_config::kWheelRadiusM;
    const float follow_max=ROBOT_FOLLOW_MAX_RPM*main_tuning::rpm;
    params.spin_target_rad_s=ROBOT_SPIN_TARGET_RPM*main_tuning::rpm;
    params.spin_max_rad_s=ROBOT_SPIN_MAX_RPM*main_tuning::rpm;
    params.spin_accel_rad_s2=ROBOT_SPIN_ACCEL_RPM_S*main_tuning::rpm;
    params.max_w_rad_s=follow_max>params.spin_max_rad_s?follow_max:params.spin_max_rad_s;
    // 两轴满杆稳态瞄准速度分别换算；下面共用的加速度仅决定达到各自速度需要多久。
    params.yaw_rate_rad_s=ROBOT_YAW_AIM_RPM*main_tuning::rpm;
    params.pitch_rate_rad_s=ROBOT_PITCH_AIM_RPM*main_tuning::rpm;
    params.aim_acceleration_rad_s2=ROBOT_AIM_ACCEL_RPM_S*main_tuning::rpm;
    params.follow={.kp=ROBOT_FOLLOW_KP,.ki=ROBOT_FOLLOW_KI,.kd=ROBOT_FOLLOW_KD,
      .integral_limit=ROBOT_FOLLOW_INTEGRAL_LIMIT_RPM*main_tuning::rpm,.output_limit=follow_max,.error_period=6.283185307179586f};
    params.follow_deadband_rad=ROBOT_FOLLOW_DEADBAND_RAD;
    params.follow_accel_rad_s2=ROBOT_FOLLOW_ACCEL_RPM_S*main_tuning::rpm;
  }
  return params;
}();
// 单位车体自转速度对应的最大轮轴速度，由最远轮的力臂和轮半径决定。
inline float spinWheelSpeedRatio() {
  constexpr float max_radius_sq=[] {
    float result=0;
    for(const auto &wheel:vehicle_config::kWheelPositions) {
      const float radius_sq=wheel.x_m*wheel.x_m+wheel.y_m*wheel.y_m;
      if(radius_sq>result)result=radius_sq;
    }
    return result;
  }();
  return std::sqrt(max_radius_sq)/vehicle_config::kWheelRadiusM;
}
// 目标上限严格低于实测超速保护值，实际保护判断仍在驱动控制器。
inline float spinWheelCeiling() {
  return std::nextafter(robot_debug::drive::kMaxActualSpeed,0.0f);
}
// 先保留普通模式的满杆轮速，再把剩余轮速换算为可用的车体自转速度。
inline float spinWheelRateLimit() {
  return (spinWheelCeiling()-robot_debug::drive::kMaxTargetSpeed)/spinWheelSpeedRatio();
}
// 小陀螺的轮轴上限覆盖满杆平移和当前自转同向叠加的最坏情况。
inline float spinWheelTargetLimit(float spin_rate_rad_s) {
  const float requested=robot_debug::drive::kMaxTargetSpeed+std::fabs(spin_rate_rad_s)*spinWheelSpeedRatio();
  return std::fmin(requested,spinWheelCeiling());
}
// 两板IMU安装矩阵已经各自实测确认，不随普通电机调参改变。
inline constexpr bool kChassisImuConfirmed=true, kGimbalImuConfirmed=true;
// 底盘已按水平、车头抬高和左侧抬高实测确认：车体 X=传感器 X、Y=-传感器 Y、Z=-传感器 Z。
// 该旋转同时用于加速度和角速度；云台安装矩阵仍单独核对，不能直接复制。
inline constexpr float kChassisRotation[9]={1,0,0,0,-1,0,0,0,-1};
// 云台按水平与枪口抬高实测录入：云台 X=传感器 Y、Y=传感器 X、Z=-传感器 Z。
// 与底盘独立配置，同时变换加速度和角速度；更新后仍须实机复核 yaw 正方向。
inline constexpr float kGimbalRotation[9]={0,1,0,1,0,0,0,0,-1};
}
#endif
