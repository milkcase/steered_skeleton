#ifndef DBUS_UART_HPP_
#define DBUS_UART_HPP_

#include "dbus_receiver.hpp"
#include "stm32_hal.hpp"

namespace remote_input {
// 保存并恢复原来的屏蔽状态，防止在中断内错误地提前开放中断。
class InterruptLock {
 public:
  // 进入临界区：保存原中断屏蔽状态并关中断，防止快照读写交错。
  InterruptLock() : saved_(__get_PRIMASK()) { __disable_irq(); }
  // 退出临界区：恢复进入时的屏蔽状态，支持中断内调用和嵌套使用。
  ~InterruptLock() { __set_PRIMASK(saved_); }
  // 禁止复制临界区对象，避免重复恢复同一份中断状态。
  InterruptLock(const InterruptLock &) = delete;
  // 禁止赋值临界区对象，保证中断状态由原对象独立恢复。
  InterruptLock &operator=(const InterruptLock &) = delete;
 private:
  uint32_t saved_;
};

class DbusUart {
 public:
  // 绑定串口和遥控数据对象，不在构造时启动 DMA 或使能电机。
  DbusUart(UART_HandleTypeDef &uart, DbusReceiver &receiver)
      : uart_(uart), receiver_(receiver) {}

  // 清空遥控输入，停止已启动的接收后重新启动 DMA；重复初始化不会沿用旧指令。
  void init() {
    InterruptLock lock;
    receiver_.invalidate();
    if (started_) {
      if (!stop()) return;
    }
    start();
  }

  // 周期检查遥控超时，并重试未成功的 DMA 启动；不在函数内循环等待。
  void poll() {
    InterruptLock lock;
    receiver_.refresh(HAL_GetTick());
    // 启动失败时留给下一次任务重试，不在中断中死循环重试。
    if (!started_) start();
  }

  // 处理接收事件，size 为 HAL 报告的字节数；忽略半传输，只解码空闲边界的合法帧。
  // 先停止 DMA 再读取缓冲区，完成校验后启动下一轮接收；异常时输入失效。
  void onReceive(uint16_t size) {
    InterruptLock lock;
    if (!started_) return;
    const auto event = HAL_UARTEx_GetRxEventType(&uart_);
    if (event == HAL_UART_RXEVENT_HT) return;
    const bool clean = uart_.ErrorCode == HAL_UART_ERROR_NONE;
    // 原工程为循环 DMA。先停 DMA，保证校验、解码期间缓冲区不会继续被覆盖。
    if (!stop()) {
      receiver_.invalidate();
      return;
    }
    // 一帧数据必须由 IDLE 确认边界。满 36 字节意味着超长/粘连，直接丢弃。
    if (event == HAL_UART_RXEVENT_IDLE && clean) {
      receiver_.accept(buffer_, size, HAL_GetTick());
    } else {
      receiver_.invalidate();
    }
    start();
  }

  // 处理串口错误：立即清空输入，停止接收并清错误标志，然后尝试恢复 DMA。
  void onError() {
    InterruptLock lock;
    receiver_.invalidate();
    if (!stop()) return;
    // STM32F4 通过读 SR/DR 清理校验、帧错误、噪声和溢出标志，再开始新一轮。
    __HAL_UART_CLEAR_PEFLAG(&uart_);
    start();
  }

 private:
  // 停止本轮 UART/DMA 接收并清启动标记；返回是否成功，失败由外层保持输入无效。
  bool stop() {
    started_ = false;
    return HAL_UART_AbortReceive(&uart_) == HAL_OK;
  }

  // 尝试启动 36 字节 DMA 缓冲区并关闭半传输中断；启动失败即清空输入。
  void start() {
    if (uart_.hdmarx == nullptr) {
      receiver_.invalidate();
      return;
    }
    // 18 字节是协议帧长；36 字节是 DMA 容量，用来在空闲边界识别超长帧。
    // 若容量只设 18，循环 DMA 满帧回调后可能无法收到本帧的 IDLE 回调。
    started_ = HAL_UARTEx_ReceiveToIdle_DMA(&uart_, buffer_, sizeof(buffer_)) == HAL_OK;
    if (started_) {
      __HAL_DMA_DISABLE_IT(uart_.hdmarx, DMA_IT_HT);
    } else {
      receiver_.invalidate();
    }
  }

  UART_HandleTypeDef &uart_;
  DbusReceiver &receiver_;
  uint8_t buffer_[36]{};
  bool started_ = false;
};
}  // namespace remote_input
#endif
