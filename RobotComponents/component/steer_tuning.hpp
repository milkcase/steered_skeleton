#ifndef STEERED_STEER_TUNING_HPP_
#define STEERED_STEER_TUNING_HPP_
#include <cmath>
#include <cstdint>
#include <limits>
#include "main_control_tuning.hpp"
#include "pid.hpp"
#include "robot_tuning.hpp"
namespace robot_debug::direction {
inline constexpr float kPi=3.14159265358979323846f;
// 四舵调参直接使用填写值，不量化。只拒绝非有限/负增益与物理不可能的电流配置。
constexpr bool nonnegative(float v) {return v>=0 && v<=std::numeric_limits<float>::max();}
inline constexpr float kPositionKp=ROBOT_STEER_POSITION_KP,kSpeedKp=ROBOT_STEER_SPEED_KP;
inline constexpr float kPositionKi=ROBOT_STEER_POSITION_KI,kPositionKd=ROBOT_STEER_POSITION_KD;
inline constexpr float kPositionIntegralLimit=ROBOT_STEER_POSITION_INTEGRAL_LIMIT_RAD_S,kSpeedIntegralLimit=ROBOT_STEER_SPEED_INTEGRAL_LIMIT_A;
inline constexpr float kSpeedKi=ROBOT_STEER_SPEED_KI,kSpeedKd=ROBOT_STEER_SPEED_KD;
inline constexpr float kCurrentA=ROBOT_STEER_CURRENT_LIMIT_A,kVelocityLimit=0.25f;
static_assert(nonnegative(kPositionKp) && nonnegative(kSpeedKp) && nonnegative(kSpeedKi) && nonnegative(kSpeedKd),"PID必须是有限非负数");
static_assert(nonnegative(kPositionKi) && nonnegative(kPositionKd) && nonnegative(kPositionIntegralLimit) && nonnegative(kSpeedIntegralLimit),"I/D及积分输出限幅必须是有限非负数");
static_assert(kCurrentA>0 && kCurrentA<=3.0f,"GM6020限流必须有效且不超过驱动允许的3A");
// 生成位置环参数：周期误差避免跨±π时绕远；四舵模式直接采用外环PID和积分限幅。
inline constexpr hello_world::pid::Pid::Params positionParams() {
  return {.kp=kPositionKp,.ki=kPositionKi,.kd=kPositionKd,.integral_limit=kPositionIntegralLimit,
      .output_limit=kVelocityLimit,.max_dt_s=0.003f,.error_period=2*kPi};
}
// 生成速度环参数：四舵可独立设置积分输出限幅，最终电流始终受总限流约束。
inline constexpr hello_world::pid::Pid::Params speedParams() {
  return {.kp=kSpeedKp,.ki=kSpeedKi,.kd=kSpeedKd,.integral_limit=kSpeedIntegralLimit,
      .output_limit=kCurrentA,.max_dt_s=0.003f};
}
}
#endif
