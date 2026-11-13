#pragma once
#include <cstdint>
extern uint32_t test_irq;
namespace remote_input {
class InterruptLock {
 public:
  // 保存模拟屏蔽状态，检查快照读取是否使用临界区。
  InterruptLock():saved_(test_irq) {test_irq=1;}
  // 恢复原屏蔽状态，不修改宿主电脑中断。
  ~InterruptLock() {test_irq=saved_;}
 private:
  uint32_t saved_;
};
}
