#ifndef STEERED_STEER_DIRECTION_TUNING_HPP_
#define STEERED_STEER_DIRECTION_TUNING_HPP_
#include <cmath>
#include <cstdint>
#include <limits>
#include "pid.hpp"
#include "motor_debug_tuning.hpp"
#ifndef ROBOT_FOUR_STEER_ENABLED
#define ROBOT_FOUR_STEER_ENABLED 0
#endif

// 新的双环整定参数；旧 motor_debug_tuning.hpp 中的8A/rad不能当作速度环P照搬。
// 外环输出rad/s，内环输出A；FourSteer模式两环均支持PID，首次升级两板，之后改数值只烧底盘。
// 旧SteerDirection保留历史档位；FourSteer直接使用有限非负小数，增益不会改变速度和电流上限。
#ifndef ROBOT_STEER_POSITION_KP
#define ROBOT_STEER_POSITION_KP 10.0f
#endif
#ifndef ROBOT_STEER_POSITION_KI
#define ROBOT_STEER_POSITION_KI 0.0f
#endif
#ifndef ROBOT_STEER_POSITION_KD
#define ROBOT_STEER_POSITION_KD 0.0f
#endif
// 积分限幅约束的是积分输出项：外环单位rad/s，内环单位A；默认保持原来行为。
#ifndef ROBOT_STEER_POSITION_INTEGRAL_LIMIT_RAD_S
#define ROBOT_STEER_POSITION_INTEGRAL_LIMIT_RAD_S 0.0f
#endif
#ifndef ROBOT_STEER_SPEED_KP
#define ROBOT_STEER_SPEED_KP 0.25f
#endif
#ifndef ROBOT_STEER_SPEED_KI
#define ROBOT_STEER_SPEED_KI 0.0f
#endif
#ifndef ROBOT_STEER_SPEED_KD
#define ROBOT_STEER_SPEED_KD 0.00f
#endif
#ifndef ROBOT_STEER_CURRENT_LIMIT_A
#define ROBOT_STEER_CURRENT_LIMIT_A 0.500f
#endif
#ifndef ROBOT_STEER_SPEED_INTEGRAL_LIMIT_A
#define ROBOT_STEER_SPEED_INTEGRAL_LIMIT_A (ROBOT_STEER_CURRENT_LIMIT_A*0.5f)
#endif
#ifndef ROBOT_STEER_DIRECTION_ENABLED
#define ROBOT_STEER_DIRECTION_ENABLED 0
#endif
static_assert(ROBOT_STEER_DIRECTION_ENABLED==0 || ROBOT_STEER_DIRECTION_ENABLED==1);

namespace robot_debug::direction {
inline constexpr bool kFour=ROBOT_FOUR_STEER_ENABLED==1;
inline constexpr bool kEnabled=ROBOT_STEER_DIRECTION_ENABLED==1;
inline constexpr float kPi=3.14159265358979323846f;
#if ROBOT_FOUR_STEER_ENABLED || (defined(ROBOT_DRIVE_TUNING_ENABLED) && ROBOT_DRIVE_TUNING_ENABLED)
// 四舵调参直接使用填写值，不量化。只拒绝非有限/负增益与物理不可能的电流配置。
constexpr bool nonnegative(float v) {return v>=0 && v<=std::numeric_limits<float>::max();}
inline constexpr float kHardMaxPositionKp=std::numeric_limits<float>::max(),kHardMaxCurrentA=3.0f;
inline constexpr float kPositionKp=ROBOT_STEER_POSITION_KP,kSpeedKp=ROBOT_STEER_SPEED_KP;
inline constexpr float kPositionKi=ROBOT_STEER_POSITION_KI,kPositionKd=ROBOT_STEER_POSITION_KD;
inline constexpr float kPositionIntegralLimit=ROBOT_STEER_POSITION_INTEGRAL_LIMIT_RAD_S,kSpeedIntegralLimit=ROBOT_STEER_SPEED_INTEGRAL_LIMIT_A;
inline constexpr float kSpeedKi=ROBOT_STEER_SPEED_KI,kSpeedKd=ROBOT_STEER_SPEED_KD;
inline constexpr float kCurrentA=ROBOT_STEER_CURRENT_LIMIT_A,kVelocityLimit=0.25f;
inline constexpr uint32_t kProfile=0;
static_assert(nonnegative(kPositionKp) && nonnegative(kSpeedKp) && nonnegative(kSpeedKi) && nonnegative(kSpeedKd),"PID必须是有限非负数");
static_assert(nonnegative(kPositionKi) && nonnegative(kPositionKd) && nonnegative(kPositionIntegralLimit) && nonnegative(kSpeedIntegralLimit),"I/D及积分输出限幅必须是有限非负数");
static_assert(kCurrentA>0 && kCurrentA<=3.0f,"GM6020限流必须有效且不超过驱动允许的3A");
#else
inline constexpr float kHardMaxPositionKp=100.0f;
inline constexpr float kHardMaxCurrentA=0.500f;
inline constexpr uint32_t kPositionStep=tuning::stepIndex(ROBOT_STEER_POSITION_KP,0.5f,200);
inline constexpr uint32_t kSpeedStep=tuning::stepIndex(ROBOT_STEER_SPEED_KP,0.25f,400);
inline constexpr uint32_t kIntegralStep=ROBOT_STEER_SPEED_KI==0?1:tuning::stepIndex(ROBOT_STEER_SPEED_KI,0.1f,15)+1;
inline constexpr uint32_t kDerivativeStep=ROBOT_STEER_SPEED_KD==0?1:tuning::stepIndex(ROBOT_STEER_SPEED_KD,0.001f,10)+1;
inline constexpr uint32_t kCurrentStep=tuning::stepIndex(ROBOT_STEER_CURRENT_LIMIT_A,0.05f,10);
static_assert(kPositionStep && kSpeedStep && kCurrentStep,
    "位置P:0.5~100；速度P:0.25~100；电流:0.05~0.50A，步长0.05");
static_assert(ROBOT_STEER_SPEED_KI==0 || tuning::stepIndex(ROBOT_STEER_SPEED_KI,0.1f,15),"速度I:0~1.5，步长0.1");
static_assert(ROBOT_STEER_SPEED_KD==0 || tuning::stepIndex(ROBOT_STEER_SPEED_KD,0.001f,10),"速度D:0~0.01，步长0.001");
inline constexpr float kPositionKp=kPositionStep*0.5f,kSpeedKp=kSpeedStep*0.25f;
inline constexpr float kSpeedKi=(kIntegralStep-1)*0.1f,kSpeedKd=(kDerivativeStep-1)*0.001f;
inline constexpr float kCurrentA=kCurrentStep*0.05f,kVelocityLimit=0.25f;
inline constexpr uint8_t kTarget=33; // 新控制算法版本，旧单位置环固件不能授权本版。
// 位置8位、速度9位、I/D/电流各4位互不重叠；只是本地记录，不参与双板增益校验。
static_assert(kPositionStep<=256 && kSpeedStep<=512 && kIntegralStep<=16 && kDerivativeStep<=16 && kCurrentStep<=16);
inline constexpr uint32_t kProfile=0x80000000u|((kPositionStep-1)<<21)|((kSpeedStep-1)<<12)|
    ((kIntegralStep-1)<<8)|((kDerivativeStep-1)<<4)|(kCurrentStep-1);
#endif
inline constexpr int kRawCurrentLimit=static_cast<int>(kCurrentA*16384.0f/3.0f);
// 生成位置环参数：周期误差避免跨±π时绕远；四舵模式直接采用外环PID和积分限幅。
inline constexpr hello_world::pid::Pid::Params positionParams() {
#if ROBOT_FOUR_STEER_ENABLED || (defined(ROBOT_DRIVE_TUNING_ENABLED) && ROBOT_DRIVE_TUNING_ENABLED)
  return {.kp=kPositionKp,.ki=kPositionKi,.kd=kPositionKd,.integral_limit=kPositionIntegralLimit,
      .output_limit=kVelocityLimit,.max_dt_s=0.003f,.error_period=2*kPi};
#else
  return {.kp=kPositionKp,.output_limit=kVelocityLimit,.max_dt_s=0.003f,.error_period=2*kPi};
#endif
}
// 生成速度环参数：四舵可独立设置积分输出限幅，最终电流始终受总限流约束。
inline constexpr hello_world::pid::Pid::Params speedParams() {
#if ROBOT_FOUR_STEER_ENABLED || (defined(ROBOT_DRIVE_TUNING_ENABLED) && ROBOT_DRIVE_TUNING_ENABLED)
  return {.kp=kSpeedKp,.ki=kSpeedKi,.kd=kSpeedKd,.integral_limit=kSpeedIntegralLimit,
      .output_limit=kCurrentA,.max_dt_s=0.003f};
#else
  return {.kp=kSpeedKp,.ki=kSpeedKi,.kd=kSpeedKd,.integral_limit=kCurrentA*0.5f,
      .output_limit=kCurrentA,.max_dt_s=0.003f};
#endif
}
// 将有限角度变为[-π,π)，与整车逆解的最短角差约定相同。
inline float wrap(float angle) {
  float v=std::fmod(angle,2*kPi);if(v>=kPi)v-=2*kPi;if(v< -kPi)v+=2*kPi;return v;
}
struct StickDirection {bool valid=false,active=false;float angle=0;};
// 左杆前为+x、左为+y，因此右正通道要取负；幅值只判定死区，方向由atan2决定。
// 输入仍是云台坐标，底盘收到后再加本地yaw关节角，等价于已有逆解R(+delta)。
inline StickDirection solve(float forward,float right) {
  if(!std::isfinite(forward) || !std::isfinite(right) || std::fabs(forward)>1.001f || std::fabs(right)>1.001f)return {};
  const bool active=forward*forward+right*right>0.05f*0.05f;
  return {true,active,active?std::atan2(-right,forward):0};
}
}
#endif
