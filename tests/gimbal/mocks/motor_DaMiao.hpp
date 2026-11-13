#pragma once
namespace hello_world::motor {
enum class Status { kOk,kInputValueOverflow };
enum class InputType { kTorq,kRaw };
struct MotorInfo { float torq_input_lim=7.0f; };
// 测试替身只记录输入；没有 enable 或 CAN 接口，位置环若尝试调用它们将无法编译。
class DaMiao {
 public:
  float position=0,input=0,mit_position=0,mit_velocity=0,mit_kp=0,mit_kd=0;
  bool enabled=true,reject_nonzero=false;
  InputType input_type=InputType::kTorq;
  MotorInfo info{};
  // 提供模拟的已标定角度，可注入 NaN 或软件限位外反馈。
  float angle() const { return position; }
  // 返回模拟使能状态；控制器不得自行改变该状态。
  bool is_enabled() const { return enabled; }
  // 返回输入单位，使测试能够验证错误输入类型会被拒绝。
  InputType get_input_type() const { return input_type; }
  // 返回模拟驱动的力矩上限，检查上层不能越过驱动限制。
  const MotorInfo &motor_info() const { return info; }
  // 模拟提供驱动的 setInput 语义，清 MIT 位置/速度/Kp/Kd；支持非零写入失败注入。
  Status setInput(float value) {
    if(reject_nonzero && value!=0)return Status::kInputValueOverflow;
    input=value;mit_position=mit_velocity=mit_kp=mit_kd=0;
    return Status::kOk;
  }
};
}
