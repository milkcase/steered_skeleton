#ifndef STEERED_WATCHDOG_OUTPUTS_HPP_
#define STEERED_WATCHDOG_OUTPUTS_HPP_
#include <cmath>
namespace robot_watchdog {
// 未配置的空对象不产生输入；非控制状态必须实际为零，控制状态仍拒绝 NaN/Inf。
template <typename Motor>
bool inputSafe(const Motor *motor,bool controlling) {
  if(motor==nullptr)return true;
  const float input=motor->getInput();
  return std::isfinite(input) && (controlling || input==0);
}
}
#endif
