#ifndef STEERED_WATCHDOG_SUPERVISOR_HPP_
#define STEERED_WATCHDOG_SUPERVISOR_HPP_
#include "watchdog_config.hpp"
#include "control_clock.hpp"
namespace robot_watchdog {
enum class Fault : uint32_t {
  None=0,NotInitialized=1,Order=2,NoControl=3,Clock=4,
  PhaseOverrun=5,ForegroundStale=6,UnsafeOutputs=7,RefreshFailed=8,TestInhibit=9
};
// 纯喂狗策略：只发放刷新许可，不操作 IWDG、外设或电机；硬件复位仍由真实 IWDG 完成。
class Supervisor {
 public:
  // 在 DWT 启动后初始化；未选择可靠周期计数时不允许喂狗。
  bool init(uint32_t cycles,uint32_t milliseconds,uint32_t cpu_hz,bool running) {
    *this=Supervisor{};cpu_hz_=cpu_hz;started_at_=cycles;
    initialized_=cpu_hz>=1000000 && running;
    if(!initialized_) {fault_=Fault::Clock;return false;}
    clock_.step(cycles,milliseconds,cpu_hz,running);return true;
  }
  // 前台循环完成/开始一次传感器工作时记录进度，初始化前的心跳不构成许可。
  void foreground(uint32_t cycles) {if(initialized_) {foreground_seen_=true;last_foreground_=cycles;}}
  // 标记本轮控制开始，检查不能跳过上轮通讯、重复嵌套或在失效时钟下继续喂狗。
  bool controlBegin(uint32_t cycles,uint32_t milliseconds,uint32_t cpu_hz,bool running) {
    if(!usable())return false;
    if(control_running_ || comm_running_ || fresh_control_)return fail(Fault::Order);
    const float dt=clock_.step(cycles,milliseconds,cpu_hz,running);
    if(cpu_hz!=cpu_hz_ || dt<=0 || dt>kPhaseBudgetMs*0.001f)return fail(Fault::Clock);
    control_running_=true;phase_start_=cycles;cycle_start_=cycles;return true;
  }
  // 只有控制函数正常返回且未超预算，才能为本轮通讯发放一次新控制完成标记。
  bool controlEnd(uint32_t cycles) {
    if(!usable())return false;
    if(!control_running_)return fail(Fault::Order);
    if(cycles-phase_start_>cyclesFor(kPhaseBudgetMs))return fail(Fault::PhaseOverrun);
    control_running_=false;fresh_control_=true;++control_count_;return true;
  }
  // 通讯必须消费本轮新控制结果；仅反复运行 CommTask 不能喂狗。
  bool commBegin(uint32_t cycles) {
    if(!usable())return false;
    if(control_running_ || comm_running_)return fail(Fault::Order);
    if(!fresh_control_)return fail(Fault::NoControl);
    fresh_control_=false;comm_running_=true;phase_start_=cycles;return true;
  }
  // 通讯正常返回、前台/时钟有进展且输出安全后，才按 20ms 间隔申请一次 IWDG 刷新。
  bool commEnd(uint32_t cycles,uint32_t milliseconds,bool outputs_safe,bool refresh_enabled) {
    if(!usable())return false;
    if(!comm_running_)return fail(Fault::Order);
    comm_running_=false;++comm_count_;
    if(cycles-phase_start_>cyclesFor(kPhaseBudgetMs))return fail(Fault::PhaseOverrun);
    if(cycles-cycle_start_>cyclesFor(kPhaseBudgetMs))return fail(Fault::PhaseOverrun);
    if(!outputs_safe)return fail(Fault::UnsafeOutputs);
    if(!refresh_enabled)return fail(Fault::TestInhibit);
    // 首次前台运行前只等待，不误判首个 TIM6 中断为故障；超过启动宽限则锁住保护。
    if(!foreground_seen_) {
      if(cycles-started_at_>=cyclesFor(kForegroundTimeoutMs))return fail(Fault::ForegroundStale);
      return false;
    }
    if(cycles-last_foreground_>=cyclesFor(kForegroundTimeoutMs))return fail(Fault::ForegroundStale);
    return !fed_once_ || milliseconds-last_feed_ms_>=kFeedIntervalMs;
  }
  // HAL 确认刷新成功后才更新时间和次数，不把申请刷新当成已喂狗。
  void refreshed(uint32_t milliseconds) {if(fault_==Fault::None) {fed_once_=true;last_feed_ms_=milliseconds;++feeds_;}}
  // HAL 刷新失败时锁住故障，后续任务心跳不能自行解除。
  void refreshFailed() {fail(Fault::RefreshFailed);}
  // 返回首个锁存故障；保留第一现场，后续重复错误不能覆盖原因。
  Fault fault() const {return fault_;}
  // 返回实际成功刷新的次数，供调试确认 Dead/校准时仍可健康运行。
  uint32_t feeds() const {return feeds_;}
  // 返回最近一次成功刷新时间，单位 HAL 毫秒。
  uint32_t lastFeedMs() const {return last_feed_ms_;}
  // 返回完成控制次数，便于排查控制调度是否停顿。
  uint32_t controlCount() const {return control_count_;}
  // 返回完成通讯次数，便于核对控制后通讯的执行链。
  uint32_t commCount() const {return comm_count_;}
 private:
  // 将短时间预算换成 CPU 周期，均远短于 32 位 DWT 的回绕周期。
  uint32_t cyclesFor(uint32_t ms) const {return (cpu_hz_/1000)*ms;}
  // 检查初始化和锁存故障，不允许未启动或已失败的策略发放许可。
  bool usable() {if(!initialized_)return fail(Fault::NotInitialized);return fault_==Fault::None;}
  // 记录首个故障并返回 false，供所有不合法路径统一退出。
  bool fail(Fault fault) {if(fault_==Fault::None)fault_=fault;return false;}
  robot_control::ControlClock clock_{};
  Fault fault_=Fault::None;
  uint32_t cpu_hz_=0,started_at_=0,last_foreground_=0,phase_start_=0,cycle_start_=0,last_feed_ms_=0;
  uint32_t control_count_=0,comm_count_=0,feeds_=0;
  bool initialized_=false,foreground_seen_=false,control_running_=false,comm_running_=false;
  bool fresh_control_=false,fed_once_=false;
};
}
#endif
