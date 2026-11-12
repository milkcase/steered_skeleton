#ifndef DBUS_UART_HPP_
#define DBUS_UART_HPP_

#include "dbus_receiver.hpp"
#include "stm32_hal.hpp"

namespace remote_input {
// 保存并恢复原来的屏蔽状态，防止在中断内错误地提前开放中断。
class InterruptLock {
 public:
  InterruptLock() : saved_(__get_PRIMASK()) { __disable_irq(); }
  ~InterruptLock() { __set_PRIMASK(saved_); }
  InterruptLock(const InterruptLock &) = delete;
  InterruptLock &operator=(const InterruptLock &) = delete;
 private:
  uint32_t saved_;
};

class DbusUart {
 public:
  DbusUart(UART_HandleTypeDef &uart, DbusReceiver &receiver)
      : uart_(uart), receiver_(receiver) {}

  void init() {
    InterruptLock lock;
    receiver_.invalidate();
    if (started_) {
      if (!stop()) return;
    }
    start();
  }

  void poll() {
    InterruptLock lock;
    receiver_.refresh(HAL_GetTick());
    // 启动失败时留给下一次任务重试，不在中断中死循环重试。
    if (!started_) start();
  }

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

  void onError() {
    InterruptLock lock;
    receiver_.invalidate();
    if (!stop()) return;
    // STM32F4 通过读 SR/DR 清理校验、帧错误、噪声和溢出标志，再开始新一轮。
    __HAL_UART_CLEAR_PEFLAG(&uart_);
    start();
  }

 private:
  bool stop() {
    started_ = false;
    return HAL_UART_AbortReceive(&uart_) == HAL_OK;
  }

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
