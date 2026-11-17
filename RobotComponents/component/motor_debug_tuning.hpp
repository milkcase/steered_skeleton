/*
 * @Author: milkcase choose68@163.com
 * @Date: 2026-11-15 20:11:20
 * @LastEditors: milkcase choose68@163.com
 * @LastEditTime: 2026-11-15 20:43:22
 * @FilePath: \Chassisd:\RM\steered_project\steered_skeleton\RobotComponents\component\motor_debug_tuning.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef STEERED_MOTOR_DEBUG_TUNING_HPP_
#define STEERED_MOTOR_DEBUG_TUNING_HPP_
#include <cstdint>

#include "robot_tuning.hpp"
// 历史单位置环参数已迁入robot_tuning.hpp末组；下方保留旧协议档位与编码。

namespace robot_debug::tuning {
// 以下为本轮软件硬边界及配置编码，不属于现场可调旋钮；运动范围、时限和遥控死区另保留原值。
inline constexpr float kHardMaxKp=8.0f,kHardMaxCurrentA=0.200f;

// 将输入限制为离散档位；非法范围、非有限值或非档位值返回0，由编译检查拒绝。
constexpr uint32_t stepIndex(float value,float step,uint32_t max_steps) {
  if(!(value>=step && value<=step*max_steps))return 0;
  const auto index=static_cast<uint32_t>(value/step+0.5f);
  const float difference=value-step*index;
  return index>=1 && index<=max_steps && difference>=-0.000001f && difference<=0.000001f?index:0;
}
inline constexpr uint32_t kKpStep=stepIndex(ROBOT_MOTOR_DEBUG_KP,0.5f,16);
inline constexpr uint32_t kCurrentStep=stepIndex(ROBOT_MOTOR_DEBUG_CURRENT_LIMIT_A,0.025f,8);
static_assert(kKpStep!=0,"调试P必须为0.5～8.0，步长0.5");
static_assert(kCurrentStep!=0,"调试电流上限必须为0.025～0.200A，步长0.025A");

// 每个P/电流组合使用不同的8位编号，无哈希碰撞；两板参数不同就不能完成动力握手。
constexpr uint16_t profileFor(uint32_t kp_step,uint32_t current_step) {
  return kp_step>=1 && kp_step<=16 && current_step>=1 && current_step<=8?
      static_cast<uint16_t>(0x80u|((kp_step-1u)<<3)|(current_step-1u)):0;
}
// 采用规范档位值，避免用户小数表示误差造成相同编号却使用不同浮点参数。
inline constexpr float kPositionKp=kKpStep*0.5f,kCurrentLimitA=kCurrentStep*0.025f;
inline constexpr uint16_t kProfile=profileFor(kKpStep,kCurrentStep);
// GM6020电流协议±16384对应±3A，发送层另检查这一档可编码的最大绝对整数。
inline constexpr int kRawCurrentLimit=static_cast<int>(kCurrentLimitA*16384.0f/3.0f);
static_assert(kPositionKp<=kHardMaxKp && kCurrentLimitA<=kHardMaxCurrentA);
static_assert(kProfile>=128 && kProfile<=255,"参数编号必须装入现有协议一个字节");
}
#endif
