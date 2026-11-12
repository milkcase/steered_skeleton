#ifndef HW_STEERED_PID_HPP_
#define HW_STEERED_PID_HPP_

#include <cmath>

namespace hello_world::pid {
// 位置式 PID：每次直接算出本周期输出，不依赖电机、HAL 或机器人工作模式。
class Pid {
 public:
  struct Params {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_limit = 0.0f;  // 积分“输出项”的绝对上限，与 PID 输出单位一致。
    float output_limit = 0.0f;    // 最终输出的绝对上限；0 表示始终输出零。
    float max_dt_s = 0.010f;      // 控制间隔上限，单位秒；错过太多周期时清状态、输出零。
    float error_period = 0.0f;    // 0 为线性误差；角度环可设为 2π，单位 rad。
  };

  // 构造默认 PID，所有增益及输出上限为零；只创建计算状态，不输出到电机。
  Pid() = default;  // 默认全部增益和输出上限均为零，不产生控制输出。
  // 按给定参数构造 PID 并清空历史；参数非法时控制器不可用。
  explicit Pid(const Params &params) { configure(params); }

  // 重新配置时一定清除积分与历史误差；参数不合法则禁用计算并返回 false。
  bool configure(const Params &params);
  // 清空积分、历史误差、输出和上次计算状态，保留已配置的参数。
  void reset();

  // dt 单位为秒；目标与反馈的物理单位必须一致，默认按 1 ms 控制周期计算。
  float update(float target, float feedback, float dt_s = 0.001f);
  // 已经由上层计算好误差时使用此入口；仍保留限幅、有效性检查及周期处理。
  float updateError(float error, float dt_s = 0.001f);

  // 返回当前参数的只读引用，便于检查增益、限幅和误差周期。
  const Params &params() const { return params_; }
  // 返回上次更新后的限幅输出；异常或复位后为零。
  float output() const { return output_; }
  // 返回当前积分输出项，单位与最终控制输出一致，供调试整定使用。
  float integralTerm() const { return integral_term_; }
  // 返回参数配置是否合法；不代表机器人处于允许运动的状态。
  bool configured() const { return configured_; }
  // 返回上次计算是否成功；false 时调用者应按无效控制结果处理。
  bool lastUpdateOk() const { return last_update_ok_; }

 private:
  // 按配置周期归一化误差；线性回路直接返回原值。
  float normalizeError(float error) const;
  // 清空计算历史并返回零，统一处理非法输入、时间间隔和数值溢出。
  float fail();
  Params params_{};
  float integral_term_ = 0.0f;
  float previous_error_ = 0.0f;
  float output_ = 0.0f;
  bool has_previous_error_ = false;
  bool configured_ = true;
  bool last_update_ok_ = false;
};

// 共享头文件中的定义均为 inline，两板复用且不产生重复链接符号。

namespace detail {
// 将有限数值限制在正负 limit 之间，用于积分项和最终输出限幅。
inline float clampSymmetric(float value, float limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}
}  // namespace detail

// 保存并检查所有参数，同时复位历史；返回配置是否合法，非法时禁用计算。
inline bool Pid::configure(const Params &params) {
  reset();
  params_ = params;
  // 本实现允许负增益以兼容上层的符号约定，但不接受非有限数或负的限制值。
  configured_ = std::isfinite(params.kp) && std::isfinite(params.ki) &&
      std::isfinite(params.kd) && std::isfinite(params.integral_limit) &&
      std::isfinite(params.output_limit) && std::isfinite(params.max_dt_s) &&
      std::isfinite(params.error_period) && params.integral_limit >= 0.0f &&
      params.output_limit >= 0.0f && params.max_dt_s > 0.0f &&
      params.error_period >= 0.0f;
  return configured_;
}

// 清除积累的积分、历史误差与输出；恢复后的首周期不计算微分项。
inline void Pid::reset() {
  integral_term_ = 0.0f;
  previous_error_ = 0.0f;
  output_ = 0.0f;
  has_previous_error_ = false;
  last_update_ok_ = false;
}

// 异常处理公共出口：调用 reset 清历史，并返回零输出。
inline float Pid::fail() {
  reset();
  return 0.0f;
}

// 周期大于零时返回最短周期误差；周期为零时保持线性误差。
inline float Pid::normalizeError(float error) const {
  if (params_.error_period == 0.0f) return error;
  // 先取模再调整到 [-周期/2, 周期/2)，避免目标跨过角度零点时绕远路。
  float wrapped = std::fmod(error, params_.error_period);
  const float half = params_.error_period * 0.5f;
  if (wrapped >= half) wrapped -= params_.error_period;
  if (wrapped < -half) wrapped += params_.error_period;
  return wrapped;
}

// 检查目标与反馈的有限性，再将二者之差交给误差计算入口；dt_s 单位为秒。
inline float Pid::update(float target, float feedback, float dt_s) {
  if (!std::isfinite(target) || !std::isfinite(feedback)) return fail();
  return updateError(target - feedback, dt_s);
}

// 按误差和时间间隔计算位置式 PID，包含积分限幅、抗饱和与输出限幅。
// 成功后记录本周期状态；输入或计算异常时清历史并返回零。
inline float Pid::updateError(float error, float dt_s) {
  if (!configured_ || !std::isfinite(error) || !std::isfinite(dt_s) ||
      dt_s <= 0.0f || dt_s > params_.max_dt_s) return fail();
  error = normalizeError(error);
  // 第一步：比例项反映当前误差；首次计算不求微分，避免拿默认历史值制造冲击。
  const float proportional = params_.kp * error;
  float derivative = 0.0f;
  if (has_previous_error_ && params_.kd != 0.0f) {
    const float error_change = normalizeError(error - previous_error_);
    derivative = params_.kd * (error_change / dt_s);
  }
  // 第二步：按实际时间累加积分输出项，先检查溢出，再限制其绝对大小。
  const float increment = params_.ki * error * dt_s;
  const float proposed_integral = integral_term_ + increment;
  if (!std::isfinite(proportional) || !std::isfinite(derivative) ||
      !std::isfinite(increment) || !std::isfinite(proposed_integral)) return fail();
  float next_integral = detail::clampSymmetric(proposed_integral, params_.integral_limit);
  const float proposed_output = proportional + next_integral + derivative;
  if (!std::isfinite(proposed_output)) return fail();

  // 第三步：输出已朝某方向饱和时，不再积累会把饱和推得更深的积分。
  // 若积分变化能帮助退出饱和则允许更新，避免“限住之后回不来”。
  const float integral_change = next_integral - integral_term_;
  if ((proposed_output > params_.output_limit && integral_change > 0.0f) ||
      (proposed_output < -params_.output_limit && integral_change < 0.0f)) {
    next_integral = integral_term_;
  }
  const float sum = proportional + next_integral + derivative;
  if (!std::isfinite(sum)) return fail();

  // 第四步：提交本周期状态，最后限幅；本类只返回数值，不直接下发给电机。
  integral_term_ = next_integral;
  output_ = detail::clampSymmetric(sum, params_.output_limit);
  previous_error_ = error;
  has_previous_error_ = true;
  last_update_ok_ = true;
  return output_;
}

}  // namespace hello_world::pid
#endif
