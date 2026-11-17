#ifndef STEERED_MOTOR_DEBUG_MODE_HPP_
#define STEERED_MOTOR_DEBUG_MODE_HPP_
#include "steer_direction_tuning.hpp"
#include "drive_debug_tuning.hpp"
#include "steer_debug_safety.hpp"
static_assert(!(robot_debug::drive::kEnabled && robot_debug::direction::kEnabled),"转向和驱动独立调试模式不能同时启用");
static_assert(!robot_debug::direction::kFour || robot_debug::direction::kEnabled,"FourSteer必须启用方向双环格式");
static_assert(!robot_debug::drive::kTuning || robot_debug::drive::kEnabled,"驱动持续调参须启用驱动速度环");
namespace robot_debug::mode {
inline constexpr bool kResumable=direction::kEnabled || drive::kTuning;
// 编译时选择，诊断镜像和Ozone写变量均不能改变调试对象或扩大电流。
inline constexpr bool kWideSession=direction::kEnabled || drive::kEnabled;
inline constexpr uint8_t kTarget=drive::kEnabled?drive::kTarget:(direction::kEnabled?steer_safety::kTarget:1);
inline constexpr uint16_t kProfile=drive::kEnabled?drive::kProfile:(direction::kEnabled?steer_safety::kProfile:tuning::kProfile);
inline constexpr float kCurrentA=drive::kEnabled?drive::kCurrentA:(direction::kEnabled?direction::kCurrentA:tuning::kCurrentLimitA);
inline constexpr int kRawLimit=drive::kEnabled?drive::kRawLimit:(direction::kEnabled?direction::kRawCurrentLimit:tuning::kRawCurrentLimit);
}
#endif
