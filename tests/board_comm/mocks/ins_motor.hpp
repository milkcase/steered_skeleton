#pragma once
#include <cstdint>
namespace hello_world::motor {
struct MotorInfo {uint8_t id=1;};
class Motor {
 public:
  MotorInfo info{};int decodes=0;
  // 返回测试设备 ID，不涉及真实电机。
  const MotorInfo &motor_info() const {return info;}
  // 只累计反馈解码次数，以验证协议帧没有被误分发到电机。
  bool decode(const uint8_t *) {++decodes;return true;}
};
struct M3508 {
  // 返回提供驱动约定的驱动反馈 ID。
  static uint32_t GetRxId(uint8_t id) {return 0x200+id;}
};
struct GM6020 {
  // 返回提供驱动约定的转向反馈 ID。
  static uint32_t GetRxId(uint8_t id) {return 0x204+id;}
};
struct DaMiao {
  // 返回本工程约定的达妙反馈 ID。
  static uint32_t GetRxId(uint8_t id) {return 0x10+id;}
};
}
// 提供测试轮电机数组中的对象，索引对应本板轮位。
hello_world::motor::Motor *GetMotorWheel(int);
// 提供测试转向电机数组中的对象。
hello_world::motor::Motor *GetMotorSteer(int);
// 提供底盘 yaw 的记录对象。
hello_world::motor::Motor *GetMotorYaw();
// 提供云台 pitch 的记录对象。
hello_world::motor::Motor *GetMotorPitch();
