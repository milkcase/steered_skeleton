/*
 * @Author: milkcase choose68@163.com
 * @Date: 2026-11-15 21:54:59
 * @LastEditors: milkcase choose68@163.com
 * @LastEditTime: 2026-11-17 21:08:58
 * @FilePath: \Chassisd:\RM\steered_project\steered_skeleton\RobotComponents\component\drive_debug_tuning.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef STEERED_DRIVE_TUNING_HPP_
#define STEERED_DRIVE_TUNING_HPP_
#include "pid.hpp"
#include "robot_tuning.hpp"
#include <limits>
// 所有M3508数值在robot_tuning.hpp；这里仅换算工程单位并兼容旧rad/s编译宏。
#ifndef ROBOT_DRIVE_TARGET_SPEED_RAD_S
#define ROBOT_DRIVE_TARGET_SPEED_RAD_S (ROBOT_DRIVE_TARGET_SPEED_RPM*0.10471975511965977f)
#endif
#ifndef ROBOT_DRIVE_ACCELERATION_RAD_S2
#define ROBOT_DRIVE_ACCELERATION_RAD_S2 (ROBOT_DRIVE_ACCELERATION_RPM_S*0.10471975511965977f)
#endif
#ifndef ROBOT_DRIVE_TRIP_SPEED_RAD_S
#define ROBOT_DRIVE_TRIP_SPEED_RAD_S (ROBOT_DRIVE_TRIP_SPEED_RPM*0.10471975511965977f)
#endif

namespace robot_debug::drive {
// 连续调参直接采用小数，不分档；有限数和协议物理电流边界始终保留。
constexpr bool nonnegative(float v) {return v>=0 && v<=std::numeric_limits<float>::max();}
inline constexpr float kKp=ROBOT_DRIVE_SPEED_KP,kKi=ROBOT_DRIVE_SPEED_KI,kKd=ROBOT_DRIVE_SPEED_KD;
inline constexpr float kCurrentA=ROBOT_DRIVE_CURRENT_LIMIT_A,kIntegralA=ROBOT_DRIVE_INTEGRAL_LIMIT_A;
inline constexpr float kMaxTargetSpeed=ROBOT_DRIVE_TARGET_SPEED_RAD_S,kMaxAcceleration=ROBOT_DRIVE_ACCELERATION_RAD_S2;
inline constexpr float kMaxActualSpeed=ROBOT_DRIVE_TRIP_SPEED_RAD_S;
static_assert(nonnegative(kKp) && nonnegative(kKi) && nonnegative(kKd) && nonnegative(kIntegralA),"PID和积分限幅须有限非负");
static_assert(kCurrentA>0 && kCurrentA<=20.0f,"C620电流上限须为正且不超过协议20A边界");
static_assert(nonnegative(kMaxTargetSpeed) && nonnegative(kMaxAcceleration) && nonnegative(kMaxActualSpeed) &&
    kMaxTargetSpeed>0 && kMaxAcceleration>0 && kMaxActualSpeed>kMaxTargetSpeed,"速度/加速度须有限为正，超速保护须高于目标上限");
// C620原始±16384对应±20A；此换算只在应用发送层使用，不修改提供的驱动。
inline constexpr float kRawPerAmp=16384.0f/20.0f;
}
#endif
