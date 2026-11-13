#pragma once
#include "platform.hpp"
#include "usart.h"
namespace remote_input {
struct Receiver {};
class InterruptLock {
 public:
  // 模拟保存中断状态，允许检查嵌套保护不把原已关中断状态打开。
  InterruptLock() : previous_(irq_mask) {irq_mask=1;}
  // 恢复调用前的模拟中断状态。
  ~InterruptLock() {irq_mask=previous_;}
 private:uint32_t previous_;
};
class DbusUart {
 public:
  // 测试适配器不启动硬件。
  DbusUart(UART_HandleTypeDef &,Receiver &) {}
  // 记录原 CommTaskInit 的 DBUS 初始化仍只执行一次。
  void init() {++uart_inits;}
  // 在真实 CommTask 运行时记录阶段，模拟少量执行周期。
  void poll() {stage=2;test_dwt.CYCCNT+=1000;}
  // 原 UART 回调在本项不注入接收事件。
  void onReceive(uint16_t) {}
  // 原错误回调在本项不注入串口错误。
  void onError() {}
};
}
