#ifndef STEERED_MOTOR_FEEDBACK_HPP_
#define STEERED_MOTOR_FEEDBACK_HPP_
#include <cstdint>
namespace robot_control {
// 本板独立的反馈记录。0..3 驱动、4..7 转向、8 本板 yaw/pitch，时间只由本地 CAN 接收刷新。
class MotorFeedback {
 public:
  static constexpr int kAxis=8;
  static constexpr uint32_t kTimeoutMs=100;
  // 记录提供驱动解码后的 DJI 帧；编码器越界或异常温度立即撤销该槽有效性。
  void recordDji(int slot,const uint8_t *data,uint32_t now) {
    if(slot<0 || slot>=8)return;
    valid_[slot]=data!=nullptr && ((data[0]<<8)|data[1])<8192 && data[6]<=125;
    if(valid_[slot])last_[slot]=now;
  }
  // 达妙只接受匹配设备的状态反馈，不把寄存器应答或故障码当作有效关节反馈。
  void recordAxis(uint8_t id,const uint8_t *data,uint32_t now,bool waiting_ack=false) {
    valid_[kAxis]=data!=nullptr && !waiting_ack && (data[0]&15)==id &&
        (data[0]>>4)<=1 && data[6]<=125 && data[7]<=125;
    enabled_=valid_[kAxis] && (data[0]>>4)==1;
    if(valid_[kAxis])last_[kAxis]=now;
  }
  // 查询本地接收时间，初始、坏帧以及达到 100 ms 均为无效。
  bool fresh(int slot,uint32_t now) const {
    return slot>=0 && slot<9 && valid_[slot] && now-last_[slot]<kTimeoutMs;
  }
  // 只有新鲜且报告已使能的轴反馈，才可用于关节闭环。
  bool axisEnabled(uint32_t now) const {return fresh(kAxis,now) && enabled_;}
 private:
  uint32_t last_[9]{};
  bool valid_[9]{},enabled_=false;
};
// 每份固件拥有独立记录；头文件 inline 保证同一固件的接收和控制看到同一个实例。
inline MotorFeedback &MotorFeedbacks() {static MotorFeedback feedback;return feedback;}
}
#endif
