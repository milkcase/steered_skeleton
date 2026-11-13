#include "motor_GM6020.hpp"
#include "motor_M3508.hpp"
#include "motor_DaMiao.hpp"
#include "support.hpp"
#include <cmath>
// 交叉核对提供的驱动协议，而非重复发送层的硬编码；反馈低半字节变化必须按位域生效。
extern "C" int run_tests() {
  using namespace hello_world::motor;
  CHECK(GM6020::GetTxId(1,InputType::kRaw)==0x1FF);
  CHECK(GM6020::GetTxId(4,InputType::kRaw)==0x1FF);
  CHECK(GM6020::GetTxId(5,InputType::kRaw)==0x2FF);
  CHECK(GM6020::GetTxId(1,InputType::kCurr)==0x1FE && M3508::GetTxId(1)==0x200);
  OptionalParams opt;opt.input_type=InputType::kTorq;opt.angle_range=AngleRange::kNegPiToPosPi;
  DM_J4310 axis;axis.init(1,opt,false,false);
  uint8_t data[8]={0x11,0x80,0,0x80,0,0,30,30};
  CHECK(axis.decode(data));const float before=axis.vel(),angle=axis.angle(),torque=axis.torq();
  data[4]=0xF0;CHECK(axis.decode(data));
  CHECK(axis.vel()>before);
  CHECK(std::fabs(axis.angle()-angle)<0.000001f && std::fabs(axis.torq()-torque)<0.000001f);
  CHECK(std::fabs(axis.vel()-before-15.0f*42.0f/4095.0f)<0.00001f);
  return 0;
}
