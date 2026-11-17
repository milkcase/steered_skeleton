/*
 * @Author: milkcase choose68@163.com
 * @Date: 2026-11-15 22:02:37
 * @LastEditors: milkcase choose68@163.com
 * @LastEditTime: 2026-11-15 23:11:44
 * @FilePath: \Chassisd:\RM\steered_project\steered_skeleton\RobotComponents\component\steer_debug_safety.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef STEERED_STEER_DEBUG_SAFETY_HPP_
#define STEERED_STEER_DEBUG_SAFETY_HPP_
#include "steer_direction_tuning.hpp"
// 转向速度数值集中在robot_tuning.hpp；此文件只验证、换算以及兼容历史协议。
// Main/FourSteer/持续驱动使用自由小数；旧SteerDirection的档位仅为历史回归保留。
namespace robot_debug::steer_safety {
#if ROBOT_NORMAL_MAIN_ENABLED || ROBOT_FOUR_STEER_ENABLED || (defined(ROBOT_DRIVE_TUNING_ENABLED) && ROBOT_DRIVE_TUNING_ENABLED)
// 新协议云台仅授权真实遥控/会话，所有数值限幅由底盘本地执行；改值不必重烧云台。
inline constexpr float kTargetRpm=ROBOT_STEER_TARGET_SPEED_RPM,kRateRpm=ROBOT_STEER_TARGET_RATE_RPM,kTripRpm=ROBOT_STEER_TRIP_SPEED_RPM;
static_assert(direction::nonnegative(kTargetRpm) && direction::nonnegative(kRateRpm) && direction::nonnegative(kTripRpm) &&
    kTargetRpm>0 && kRateRpm>0 && kRateRpm<=kTargetRpm && kTripRpm>kTargetRpm,"速度必须有限有效，变化率不超过目标，保护严格高于目标");
inline constexpr float kRpmToRadS=direction::kPi/30.0f;
inline constexpr float kTargetSpeedLimit=kTargetRpm*kRpmToRadS,kTargetRateRadS=kRateRpm*kRpmToRadS,kActualSpeedLimit=kTripRpm*kRpmToRadS;
inline constexpr uint16_t kProfile=0xF104;inline constexpr uint8_t kTarget=60;inline constexpr uint32_t kAlgorithmVersion=41;
#else
inline constexpr uint32_t kMaxTargetSteps=24,kMaxTripSteps=48,kSpeedCombinations=9500;
inline constexpr uint32_t kCurrentSlots=10,kSafetyProfiles=kSpeedCombinations*kCurrentSlots;
inline constexpr uint16_t kProfileBase=0x1000;
inline constexpr uint32_t kProfileSpan=60000,kTargetBase=44;
inline constexpr uint32_t kTargetStep=tuning::stepIndex(ROBOT_STEER_TARGET_SPEED_RPM,5.0f,kMaxTargetSteps);
inline constexpr uint32_t kRateStep=tuning::stepIndex(ROBOT_STEER_TARGET_RATE_RPM,5.0f,kMaxTargetSteps);
inline constexpr uint32_t kTripStep=tuning::stepIndex(ROBOT_STEER_TRIP_SPEED_RPM,5.0f,kMaxTripSteps);
static_assert(kTargetStep && kRateStep && kTripStep,"目标/变化率:5~120rpm步长5；超速保护:5~240rpm步长5");
static_assert(ROBOT_STEER_TARGET_RATE_RPM<=ROBOT_STEER_TARGET_SPEED_RPM,"目标角变化率不能超过速度环目标上限");
static_assert(ROBOT_STEER_TRIP_SPEED_RPM>ROBOT_STEER_TARGET_SPEED_RPM,"实际超速保护必须严格高于目标速度");
inline constexpr float kTargetRpm=kTargetStep*5.0f,kRateRpm=kRateStep*5.0f,kTripRpm=kTripStep*5.0f;
inline constexpr float kRpmToRadS=direction::kPi/30.0f;
inline constexpr float kTargetSpeedLimit=kTargetRpm*kRpmToRadS,kTargetRateRadS=kRateRpm*kRpmToRadS;
inline constexpr float kActualSpeedLimit=kTripRpm*kRpmToRadS;
// 按“变化率≤目标<保护”给9500种合法速度组合连续编号，避免散列碰撞。
constexpr uint32_t speedIndex(uint32_t target,uint32_t rate,uint32_t trip) {
  if(target<1 || target>kMaxTargetSteps || rate<1 || rate>target || trip<=target || trip>kMaxTripSteps)return 0xFFFFFFFFu;
  uint32_t index=0;for(uint32_t t=1;t<target;++t)index+=t*(kMaxTripSteps-t);
  return index+(rate-1)*(kMaxTripSteps-target)+(trip-target-1);
}
struct SpeedSteps {uint32_t target=0,rate=0,trip=0;};
// 编号反解用于编译校验，确保不同合法速度组合不会共用同一双板授权配置。
constexpr SpeedSteps decodeSpeedIndex(uint32_t index) {
  for(uint32_t t=1;t<=kMaxTargetSteps;++t) {
    const uint32_t span=kMaxTripSteps-t,bucket=t*span;
    if(index<bucket)return {t,index/span+1,index%span+t+1};
    index-=bucket;
  }
  return {};
}
inline constexpr uint32_t kSpeedIndex=speedIndex(kTargetStep,kRateStep,kTripStep);
// 速度9500档×电流10档连续编码；增益不在其中，仍可只重烧底盘调整PID。
constexpr uint32_t configurationCode(uint32_t speed_index,uint32_t current_step) {
  return speed_index<kSpeedCombinations && current_step>=1 && current_step<=kCurrentSlots?
      speed_index*kCurrentSlots+current_step-1:0xFFFFFFFFu;
}
// 用两个协议版本槽分担95000种包络，profile仍为16位；不会截断或散列不同安全参数。
constexpr uint16_t safetyProfile(uint32_t speed_index,uint32_t current_step) {
  const auto code=configurationCode(speed_index,current_step);
  return code!=0xFFFFFFFFu?static_cast<uint16_t>(kProfileBase+code%kProfileSpan):0;
}
// target是固定左前调试的配置标识，不是物理电机ID；不同段与旧版本不能互相授权。
constexpr uint8_t safetyTarget(uint32_t speed_index,uint32_t current_step) {
  const auto code=configurationCode(speed_index,current_step);
  return code!=0xFFFFFFFFu?static_cast<uint8_t>(kTargetBase+code/kProfileSpan):0;
}
inline constexpr uint16_t kProfile=safetyProfile(kSpeedIndex,direction::kCurrentStep);
inline constexpr uint8_t kTarget=safetyTarget(kSpeedIndex,direction::kCurrentStep);
inline constexpr uint32_t kAlgorithmVersion=40;
static_assert(kProfileBase+kProfileSpan-1<=0xFFFFu && kTargetBase+(kSafetyProfiles-1)/kProfileSpan<=255,
    "安全包络必须完整装入target和profile协议字段");
#endif
// 保留用户正在调的外环增益，只替换目标速度上限，避免覆盖PID文件中的用户修改。
inline constexpr hello_world::pid::Pid::Params positionParams() {
  auto p=direction::positionParams();p.output_limit=kTargetSpeedLimit;return p;
}
}
#endif
