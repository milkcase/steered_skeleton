#include "motor_DaMiao.hpp"
#include "test_support.hpp"
#include <cmath>
// 使用真实提供的达妙驱动复现初始化覆盖，并在批准修复后验证纯 MIT 力矩编码。
extern "C" int run_tests() {
  using namespace hello_world::motor;
  OptionalParams opt;opt.input_type=InputType::kTorq;
  DM_J4310 motor;motor.init(1,opt,false,false);
  CHECK(motor.current_ctrl_mode()==DaMiao::CtrlMode::kMIT);
  CHECK(!motor.is_enabled());uint8_t data[8]{};
  CHECK(motor.setInput(0)==Status::kOk && motor.encode(data));
  CHECK(motor.tx_id()==1 && data[3]%16==0 && data[4]==0 && data[5]==0 && (data[6]&0xF0)==0);
  CHECK((data[6]&15)==7 && data[7]==255); // 12 位对称范围中的零力矩量化值。
  CHECK(motor.setInput(0.5f)==Status::kOk && motor.encode(data));
  CHECK((((data[6]&15)<<8)|data[7])>2047 && !motor.is_enabled());
  DM_J4310 switching;switching.init(2,opt,false,true);
  CHECK(switching.current_ctrl_mode()==DaMiao::CtrlMode::kUnknown && !switching.is_enabled());
  DM_J4310 constructed(3,opt,false,false);
  CHECK(constructed.current_ctrl_mode()==DaMiao::CtrlMode::kMIT && !constructed.is_enabled());
  return 0;
}
