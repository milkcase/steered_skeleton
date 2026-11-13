#ifndef STEERED_CONTROL_CLOCK_HPP_
#define STEERED_CONTROL_CLOCK_HPP_
#include <cstdint>
namespace robot_control {
// DWT 计算真实周期，并监测 HAL 毫秒是否停止推进，防止低优先级 SysTick 被饿死后旧输入永不过期。
class ControlClock {
 public:
  // 输入本次 CPU 周期、HAL 时间与频率；时钟失效或 HAL 连续 3ms 不推进时返回零 dt。
  float step(uint32_t cycles,uint32_t milliseconds,uint32_t cpu_hz,bool counter_running) {
    const uint32_t elapsed=seen_?cycles-previous_cycles_:0;
    const bool first=!seen_;seen_=true;previous_cycles_=cycles;
    const uint32_t limit=(cpu_hz/1000)*3;
    if(first || milliseconds!=previous_ms_)stalled_cycles_=0;
    else if(stalled_cycles_<limit) {
      const uint32_t remaining=limit-stalled_cycles_;
      stalled_cycles_+=elapsed>=remaining?remaining:elapsed;
    }
    previous_ms_=milliseconds;
    if(!counter_running || cpu_hz==0 || limit==0 || stalled_cycles_>=limit)return 0;
    return first?0.001f:static_cast<float>(elapsed)/cpu_hz;
  }
 private:
  uint32_t previous_cycles_=0,previous_ms_=0,stalled_cycles_=0;
  bool seen_=false;
};
}
#endif
