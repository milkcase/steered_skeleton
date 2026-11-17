/*
 * @Author: milkcase choose68@163.com
 * @Date: 2026-11-15 21:54:59
 * @LastEditors: milkcase choose68@163.com
 * @LastEditTime: 2026-11-16 17:38:35
 * @FilePath: \Chassisd:\RM\steered_project\steered_skeleton\RobotComponents\component\drive_debug_tuning.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef STEERED_DRIVE_DEBUG_TUNING_HPP_
#define STEERED_DRIVE_DEBUG_TUNING_HPP_
#include "motor_debug_tuning.hpp"
#include "pid.hpp"
#include <limits>
#ifndef ROBOT_VECTOR_DRIVE_ENABLED
#define ROBOT_VECTOR_DRIVE_ENABLED 0
#endif
#ifndef ROBOT_DRIVE_TUNING_ENABLED
#define ROBOT_DRIVE_TUNING_ENABLED 0
#endif
static_assert(ROBOT_DRIVE_TUNING_ENABLED==0 || ROBOT_DRIVE_TUNING_ENABLED==1,"持续驱动开关只能为0或1");
#ifndef ROBOT_DRIVE_INTEGRAL_LIMIT_A
#define ROBOT_DRIVE_INTEGRAL_LIMIT_A (ROBOT_DRIVE_CURRENT_LIMIT_A*0.5f)
#endif
// ==================== M3508调速入口：改下列三个rpm数值，只重建/烧录底盘DriveSpeed ====================
// 轮轴目标转速绝对上限；左杆满偏对应此值，自由小数无档位，当前保留原0.5rad/s。
#ifndef ROBOT_DRIVE_TARGET_SPEED_RPM
#define ROBOT_DRIVE_TARGET_SPEED_RPM 4.7746483f
#endif
// 目标转速每秒最多变化多少rpm；当前保留原1rad/s²，不是持续加速命令。
#ifndef ROBOT_DRIVE_ACCELERATION_RPM_S
#define ROBOT_DRIVE_ACCELERATION_RPM_S 9.5492966f
#endif
// 实际轮轴超过此转速全组归零，须大于目标上限；自由小数无档位，当前保留原1rad/s。
#ifndef ROBOT_DRIVE_TRIP_SPEED_RPM
#define ROBOT_DRIVE_TRIP_SPEED_RPM 9.5492966f
#endif
// 以下为内部单位及旧rad/s宏兼容；日常只改上面的rpm入口，不同时设置两组数值。
#ifndef ROBOT_DRIVE_TARGET_SPEED_RAD_S
#define ROBOT_DRIVE_TARGET_SPEED_RAD_S (ROBOT_DRIVE_TARGET_SPEED_RPM*0.10471975511965977f)
#endif
#ifndef ROBOT_DRIVE_ACCELERATION_RAD_S2
#define ROBOT_DRIVE_ACCELERATION_RAD_S2 (ROBOT_DRIVE_ACCELERATION_RPM_S*0.10471975511965977f)
#endif
#ifndef ROBOT_DRIVE_TRIP_SPEED_RAD_S
#define ROBOT_DRIVE_TRIP_SPEED_RAD_S (ROBOT_DRIVE_TRIP_SPEED_RPM*0.10471975511965977f)
#endif

// 驱动轮独立速度环：反馈/目标均为减速后的轮轴rad/s，输出为A，不复用GM6020参数。
#ifndef ROBOT_DRIVE_SPEED_ENABLED
#define ROBOT_DRIVE_SPEED_ENABLED 0
#endif
#ifndef ROBOT_DRIVE_SPEED_KP
#define ROBOT_DRIVE_SPEED_KP 50.0f
#endif
#ifndef ROBOT_DRIVE_SPEED_KI
#define ROBOT_DRIVE_SPEED_KI 0.0f
#endif
#ifndef ROBOT_DRIVE_SPEED_KD
#define ROBOT_DRIVE_SPEED_KD 0.0f
#endif
#ifndef ROBOT_DRIVE_CURRENT_LIMIT_A
#define ROBOT_DRIVE_CURRENT_LIMIT_A 2.000f
#endif
static_assert(ROBOT_DRIVE_SPEED_ENABLED==0 || ROBOT_DRIVE_SPEED_ENABLED==1);
namespace robot_debug::drive {
inline constexpr bool kEnabled=ROBOT_DRIVE_SPEED_ENABLED==1;
inline constexpr bool kTuning=ROBOT_DRIVE_TUNING_ENABLED==1;
inline constexpr bool kVector=ROBOT_VECTOR_DRIVE_ENABLED==1;
static_assert(!kVector || (kEnabled && kTuning),"二维模式须启用持续驱动");
#if ROBOT_DRIVE_TUNING_ENABLED || !ROBOT_DRIVE_SPEED_ENABLED
// 连续调参直接采用小数，不分档；有限数和协议物理电流边界始终保留。
constexpr bool nonnegative(float v) {return v>=0 && v<=std::numeric_limits<float>::max();}
inline constexpr float kKp=ROBOT_DRIVE_SPEED_KP,kKi=ROBOT_DRIVE_SPEED_KI,kKd=ROBOT_DRIVE_SPEED_KD;
inline constexpr float kCurrentA=ROBOT_DRIVE_CURRENT_LIMIT_A,kIntegralA=ROBOT_DRIVE_INTEGRAL_LIMIT_A;
inline constexpr float kMaxTargetSpeed=ROBOT_DRIVE_TARGET_SPEED_RAD_S,kMaxAcceleration=ROBOT_DRIVE_ACCELERATION_RAD_S2;
inline constexpr float kMaxActualSpeed=ROBOT_DRIVE_TRIP_SPEED_RAD_S,kMaxDisplacement=0;
inline constexpr uint32_t kDurationMs=0,kLocalTuningCode=0;
inline constexpr uint8_t kTarget=kVector?20:19;
inline constexpr uint16_t kProfile=kVector?0xD102:0xD101; // 二维/单轴协议隔离，数值由底盘执行。
static_assert(nonnegative(kKp) && nonnegative(kKi) && nonnegative(kKd) && nonnegative(kIntegralA),"PID和积分限幅须有限非负");
static_assert(kCurrentA>0 && kCurrentA<=20.0f,"C620电流上限须为正且不超过协议20A边界");
static_assert(nonnegative(kMaxTargetSpeed) && nonnegative(kMaxAcceleration) && nonnegative(kMaxActualSpeed) &&
    kMaxTargetSpeed>0 && kMaxAcceleration>0 && kMaxActualSpeed>kMaxTargetSpeed,"速度/加速度须有限为正，超速保护须高于目标上限");
#else
inline constexpr uint32_t kP=tuning::stepIndex(ROBOT_DRIVE_SPEED_KP,0.2f,16);
inline constexpr uint32_t kI=ROBOT_DRIVE_SPEED_KI==0?0:tuning::stepIndex(ROBOT_DRIVE_SPEED_KI,0.5f,7);
inline constexpr uint32_t kD=ROBOT_DRIVE_SPEED_KD==0?0:tuning::stepIndex(ROBOT_DRIVE_SPEED_KD,0.001f,3);
inline constexpr uint32_t kCurrent=tuning::stepIndex(ROBOT_DRIVE_CURRENT_LIMIT_A,0.1f,8);
static_assert(kP && kCurrent,"驱动P:0.2~3.2步长0.2；电流:0.1~0.8A步长0.1");
static_assert(ROBOT_DRIVE_SPEED_KI==0 || kI,"驱动I:0~3.5步长0.5");
static_assert(ROBOT_DRIVE_SPEED_KD==0 || kD,"驱动D:0~0.003步长0.001");
inline constexpr float kKp=kP*0.2f,kKi=kI*0.5f,kKd=kD*0.001f,kCurrentA=kCurrent*0.1f;
inline constexpr float kMaxTargetSpeed=0.5f,kMaxAcceleration=1.0f,kMaxActualSpeed=1.0f,kMaxDisplacement=0.35f;
inline constexpr uint32_t kDurationMs=500;
inline constexpr uint8_t kTarget=18; // 新版仅校验安全包络，拒绝旧版17及转向授权。
// 为4096种本地整定组合生成无碰撞编号；非法输入返回0，这不是新版的授权编号。
constexpr uint16_t profileFor(uint32_t p,uint32_t i,uint32_t d,uint32_t current) {
  return p>=1 && p<=16 && i<=7 && d<=3 && current>=1 && current<=8?
      static_cast<uint16_t>(0x4000u|((p-1)<<8)|(i<<5)|(d<<3)|(current-1)):0;
}
inline constexpr uint16_t kLocalTuningCode=profileFor(kP,kI,kD,kCurrent);
// PID仅底盘执行，授权编号只比较限流；增益改动后不用重新烧录云台。
inline constexpr uint16_t kProfile=0x7000u|(kCurrent-1);
inline constexpr float kIntegralA=kCurrentA*0.5f;
#endif
// C620原始±16384对应±20A；此换算只在应用发送层使用，不修改提供的驱动。
inline constexpr float kRawPerAmp=16384.0f/20.0f;
inline constexpr int kRawLimit=static_cast<int>(kCurrentA*kRawPerAmp);
// 构造驱动速度PID，采用线性速度误差，积分和最终输出均按电流单位限制。
inline hello_world::pid::Pid::Params params() {
  return {.kp=kKp,.ki=kKi,.kd=kKd,.integral_limit=kIntegralA,.output_limit=kCurrentA,.max_dt_s=0.003f};
}
}
#endif
