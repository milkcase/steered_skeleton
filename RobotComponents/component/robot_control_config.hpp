#ifndef STEERED_ROBOT_CONTROL_CONFIG_HPP_
#define STEERED_ROBOT_CONTROL_CONFIG_HPP_
#include <cstdint>
#include "pid.hpp"
#include "main_control_tuning.hpp"
#include "drive_debug_tuning.hpp"
#include "vehicle_motor_config.hpp"
namespace robot_control {
// 两板统一的遥控映射：右下急停、右中回中自检、右上运行；左上/中/下为分离/跟随/小陀螺。
// 用户已确认拨杆映射；当前Main限定分离模式，动力仍需所有安全门控通过。
struct Params {
  bool separate_only=false;
  bool start_level_pitch=false;
  float aim_acceleration_rad_s2=0;
  bool remote_mapping_confirmed=false;
  float max_vx_m_s=0, max_vy_m_s=0, max_w_rad_s=0;
  float yaw_rate_rad_s=0, pitch_rate_rad_s=0;
  float deadzone=0.05f;
  uint32_t neutral_hold_ms=200, enable_window_ms=1000;
  float max_dt_s=0.003f;
  hello_world::pid::Pid::Params follow{.error_period=6.283185307179586f};
};
// 从统一调参文件生成遥控映射上限；关闭Main时保留旧零动力配置供历史回归。
inline constexpr Params kControl = [] {
  Params params;
  params.remote_mapping_confirmed = true;
  if(main_tuning::enabled) {
    params.separate_only=true;
    params.start_level_pitch=true;
    params.max_vx_m_s=params.max_vy_m_s=robot_debug::drive::kMaxTargetSpeed*vehicle_config::kWheelRadiusM;
    params.max_w_rad_s=0.1f;
    // 两轴满杆稳态瞄准速度分别换算；下面共用的加速度仅决定达到各自速度需要多久。
    params.yaw_rate_rad_s=ROBOT_YAW_AIM_RPM*main_tuning::rpm;
    params.pitch_rate_rad_s=ROBOT_PITCH_AIM_RPM*main_tuning::rpm;
    params.aim_acceleration_rad_s2=ROBOT_AIM_ACCEL_RPM_S*main_tuning::rpm;
    params.follow={.kp=1,.output_limit=0.1f,.error_period=6.283185307179586f};
  }
  return params;
}();
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
