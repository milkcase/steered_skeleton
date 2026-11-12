#ifndef DBUS_RECEIVER_HPP_
#define DBUS_RECEIVER_HPP_

#include <cstddef>
#include <cstdint>
#include "DT7.hpp"

namespace remote_input {
// 仅管理遥控数据和有效期，不依赖 HAL，不具备电机控制或使能能力。
// 中断写入与任务读取之间的互斥由外层负责。
class DbusReceiver {
 public:
  static constexpr uint32_t kTimeoutMs = 100;

  // 校验并解码一帧 DBUS，now_ms 为接收时刻的毫秒数；成功才刷新在线时间。
  // 返回 true 表示帧合法；失败立即清除旧输入，返回 false。
  bool accept(const uint8_t *data, std::size_t size, uint32_t now_ms) {
    if (!isValidFrame(data, size)) {
      invalidate();
      return false;
    }
    data_.decode(data);
    last_valid_ms_ = now_ms;
    valid_ = true;
    return true;
  }

  // 使遥控输入立即失效并清空全部控制字段；不会产生任何电机指令。
  void invalidate() {
    // 清除整份旧输入，包括摇杆、拨轮、鼠标和键盘；默认拨杆值不代表动作许可。
    data_ = hello_world::remote_control::DT7{};
    valid_ = false;
  }

  // 按当前毫秒数检查有效期；超时 100 ms 则清缓存，返回是否仍在线。
  bool refresh(uint32_t now_ms) {
    // 无符号减法兼容毫秒计数自然回绕；上电未收到有效帧时始终无效。
    if (!valid_ || static_cast<uint32_t>(now_ms - last_valid_ms_) >= kTimeoutMs) {
      invalidate();
    }
    return valid_;
  }

  // 检查有效期并复制完整快照；返回 false 时快照已清空，调用者必须禁止输出。
  bool read(hello_world::remote_control::DT7 &snapshot, uint32_t now_ms) {
    const bool online = refresh(now_ms);
    snapshot = data_;
    return online;
  }

  // 保留骨架的底层 getter 接口；控制层必须使用带有效期检查的快照读取接口。
  // 返回内部 DT7 缓存地址供原骨架兼容使用；本接口不检查在线状态。
  hello_world::remote_control::DT7 *raw() { return &data_; }

 private:
  // 检查 18 字节帧长、五通道范围、拨杆及鼠标按键；只校验，不改状态。
  static bool isValidFrame(const uint8_t *p, std::size_t size) {
    if (p == nullptr || size != 18) return false;
    const uint16_t channels[5] = {
        static_cast<uint16_t>((p[0] | (p[1] << 8)) & 0x7FF),
        static_cast<uint16_t>(((p[1] >> 3) | (p[2] << 5)) & 0x7FF),
        static_cast<uint16_t>(((p[2] >> 6) | (p[3] << 2) | (p[4] << 10)) & 0x7FF),
        static_cast<uint16_t>(((p[4] >> 1) | (p[5] << 7)) & 0x7FF),
        static_cast<uint16_t>(p[16] | (p[17] << 8)),
    };
    // 与提供的 DT7 解码比例一致：1024 为中位，偏差最多 660。
    for (uint16_t value : channels) {
      if (value < 364 || value > 1684) return false;
    }
    const uint8_t left_switch = (p[5] >> 6) & 3;
    const uint8_t right_switch = (p[5] >> 4) & 3;
    // 拨杆合法值为 1/2/3；鼠标按键只能为 0/1，异常数据不刷新在线时间。
    return left_switch != 0 && right_switch != 0 && p[12] <= 1 && p[13] <= 1;
  }

  hello_world::remote_control::DT7 data_{};
  uint32_t last_valid_ms_ = 0;
  bool valid_ = false;
};
}  // namespace remote_input
#endif
