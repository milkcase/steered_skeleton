#ifndef HW_STEERED_PID_CONFIG_HPP_
#define HW_STEERED_PID_CONFIG_HPP_
#include "pid.hpp"
#include "main_control_tuning.hpp"
#include "drive_debug_tuning.hpp"
#include "steer_debug_safety.hpp"

namespace pid_config {
using Params = hello_world::pid::Pid::Params;
inline constexpr float kAnglePeriodRad = 6.2831853071795864769f;
// 下标为左前、左后、右后、右前；每台电机使用独立 PID 状态，可独立整定参数。
// kWheel：轮速(rad/s)->C620原始值；kSteer：舵角(rad)->目标舵速(rad/s)。
// 所有数值入口在robot_tuning.hpp；这里转换为控制器单位，不另存一份整定值。
// M3508已验收值由A转换为C620原始控制量；四份数值相同，控制器历史各自独立。
inline constexpr Params wheel=[] {Params p;if(main_tuning::enabled) {
  p={.kp=robot_debug::drive::kKp*robot_debug::drive::kRawPerAmp,.ki=robot_debug::drive::kKi*robot_debug::drive::kRawPerAmp,
    .kd=robot_debug::drive::kKd*robot_debug::drive::kRawPerAmp,.integral_limit=robot_debug::drive::kIntegralA*robot_debug::drive::kRawPerAmp,
    .output_limit=robot_debug::drive::kCurrentA*robot_debug::drive::kRawPerAmp,.max_dt_s=0.003f};}return p;}();
inline constexpr Params kWheel[4] = {wheel,wheel,wheel,wheel};
#if ROBOT_NORMAL_MAIN_ENABLED
inline constexpr Params steer=[] {auto p=robot_debug::direction::positionParams();p.output_limit=robot_debug::steer_safety::kTargetSpeedLimit;return p;}();
inline constexpr Params kSteer[4]={steer,steer,steer,steer};
inline constexpr Params steer_speed=robot_debug::direction::speedParams();
inline constexpr Params kSteerSpeed[4]={steer_speed,steer_speed,steer_speed,steer_speed};
#else
inline constexpr Params kSteer[4] = {
    {.error_period = kAnglePeriodRad}, {.error_period = kAnglePeriodRad},
    {.error_period = kAnglePeriodRad}, {.error_period = kAnglePeriodRad},
};
// 转向位置环kSteer输出目标角速度(rad/s)，下面四个独立速度环输出电流(A)。
// 四轮可复制同一组已验收参数，但积分和微分历史由各自对象保存；关闭Main的历史回归仍使用零增益。
inline constexpr Params kSteerSpeed[4] = {};
#endif
// yaw 的闭环执行责任在底盘板；pitch 在云台板，pitch 使用有机械限位的线性误差。
#if ROBOT_NORMAL_MAIN_ENABLED
inline constexpr Params kYaw{.kp=ROBOT_YAW_POSITION_KP,.ki=ROBOT_YAW_POSITION_KI,.kd=ROBOT_YAW_POSITION_KD,.integral_limit=ROBOT_YAW_POSITION_INTEGRAL_RAD_S,.output_limit=ROBOT_YAW_POSITION_SPEED_RPM*main_tuning::rpm,.max_dt_s=0.003f,.error_period=kAnglePeriodRad};
inline constexpr Params kPitch{.kp=ROBOT_PITCH_POSITION_KP,.ki=ROBOT_PITCH_POSITION_KI,.kd=ROBOT_PITCH_POSITION_KD,.integral_limit=ROBOT_PITCH_POSITION_INTEGRAL_RAD_S,.output_limit=ROBOT_PITCH_POSITION_SPEED_RPM*main_tuning::rpm,.max_dt_s=0.003f};
inline constexpr Params kYawSpeed{.kp=ROBOT_YAW_SPEED_KP,.ki=ROBOT_YAW_SPEED_KI,.kd=ROBOT_YAW_SPEED_KD,.integral_limit=ROBOT_YAW_SPEED_INTEGRAL_NM,.output_limit=ROBOT_YAW_TORQUE_NM,.max_dt_s=0.003f};
inline constexpr Params kPitchSpeed{.kp=ROBOT_PITCH_SPEED_KP,.ki=ROBOT_PITCH_SPEED_KI,.kd=ROBOT_PITCH_SPEED_KD,.integral_limit=ROBOT_PITCH_SPEED_INTEGRAL_NM,.output_limit=ROBOT_PITCH_TORQUE_NM,.max_dt_s=0.003f};
#else
inline constexpr Params kYaw{.error_period=kAnglePeriodRad},kPitch{};
// yaw/pitch外环输出rad/s，以下速度内环输出Nm；由云台模块参数引用，仍默认零动力。
inline constexpr Params kYawSpeed{},kPitchSpeed{};
#endif
}  // namespace pid_config
#endif
