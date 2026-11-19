/*
 * @Author: milkcase choose68@163.com
 * @Date: 2026-11-15 22:02:37
 * @LastEditors: milkcase choose68@163.com
 * @LastEditTime: 2026-11-15 23:11:44
 * @FilePath: \Chassisd:\RM\steered_project\steered_skeleton\RobotComponents\component\steer_debug_safety.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef STEERED_STEER_LIMITS_HPP_
#define STEERED_STEER_LIMITS_HPP_
#include "steer_tuning.hpp"
// 转向速度数值集中在robot_tuning.hpp；此文件验证并换算整车转向速度上限。
namespace robot_debug::steer_safety {
inline constexpr float kTargetRpm=ROBOT_STEER_TARGET_SPEED_RPM,kRateRpm=ROBOT_STEER_TARGET_RATE_RPM,kTripRpm=ROBOT_STEER_TRIP_SPEED_RPM;
static_assert(direction::nonnegative(kTargetRpm) && direction::nonnegative(kRateRpm) && direction::nonnegative(kTripRpm) &&
    kTargetRpm>0 && kRateRpm>0 && kRateRpm<=kTargetRpm && kTripRpm>kTargetRpm,"速度必须有限有效，变化率不超过目标，保护严格高于目标");
inline constexpr float kRpmToRadS=direction::kPi/30.0f;
inline constexpr float kTargetSpeedLimit=kTargetRpm*kRpmToRadS,kTargetRateRadS=kRateRpm*kRpmToRadS,kActualSpeedLimit=kTripRpm*kRpmToRadS;
// 保留用户正在调的外环增益，只替换目标速度上限，避免覆盖PID文件中的用户修改。
inline constexpr hello_world::pid::Pid::Params positionParams() {
  auto p=direction::positionParams();p.output_limit=kTargetSpeedLimit;return p;
}
}
#endif
