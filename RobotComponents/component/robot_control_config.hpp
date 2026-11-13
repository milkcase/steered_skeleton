#ifndef STEERED_ROBOT_CONTROL_CONFIG_HPP_
#define STEERED_ROBOT_CONTROL_CONFIG_HPP_
#include <cstdint>
#include "pid.hpp"
namespace robot_control {
// 两板统一的遥控映射：右下急停、右中回中自检、右上运行；左上/中/下为分离/跟随/小陀螺。
// 任务书没有指定拨杆映射及动力值，现场核对后再确认；零参数不会放行电机。
struct Params {
  bool remote_mapping_confirmed=false;
  float max_vx_m_s=0, max_vy_m_s=0, max_w_rad_s=0;
  float yaw_rate_rad_s=0, pitch_rate_rad_s=0;
  float deadzone=0.05f;
  uint32_t neutral_hold_ms=200, enable_window_ms=1000;
  float max_dt_s=0.003f;
  hello_world::pid::Pid::Params follow{.error_period=6.283185307179586f};
};
inline constexpr Params kControl{};
// 两板安装矩阵仍需按第四项核对；这里只给出候选值，不把候选值当作已确认。
inline constexpr bool kChassisImuConfirmed=false, kGimbalImuConfirmed=false;
inline constexpr float kChassisRotation[9]={1,0,0,0,1,0,0,0,1};
inline constexpr float kGimbalRotation[9]={1,0,0,0,1,0,0,0,1};
}
#endif
