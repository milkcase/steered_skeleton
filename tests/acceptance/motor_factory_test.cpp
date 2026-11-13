#include "ins_motor.hpp"
#include "vehicle_motor_config.hpp"
#include "support.hpp"
// 检查当前真实工厂的默认保护、唯一责任轴和文档参数；不是将测试值写入生产配置。
extern "C" int run_tests() {
  using namespace hello_world::motor;
  CHECK(vehicle_config::kWheelRadiusM==0.066f);
#if defined(TEST_CHASSIS)
  CHECK(GetMotorWheel(-1)==nullptr && GetMotorWheel(4)==nullptr && GetMotorSteer(-1)==nullptr && GetMotorSteer(4)==nullptr);
  for(int i=0;i<4;++i) {
    const auto &w=vehicle_config::kWheels[i];const auto &s=vehicle_config::kSteers[i];
    auto *wm=GetMotorWheel(i);auto *sm=GetMotorSteer(i);
    if(w.configured)CHECK(wm!=nullptr && wm->motor_info().id==w.id && wm->getInput()==0);
    else CHECK(wm==nullptr);
    if(s.configured)CHECK(sm!=nullptr && sm->motor_info().id==s.id && sm->getInput()==0);
    else CHECK(sm==nullptr);
  }
  auto *base=GetMotorYaw();CHECK(base!=nullptr && base==GetMotorYaw());
  const auto &cfg=vehicle_config::kYaw;
#else
  CHECK(GetMotorYaw()==nullptr);auto *base=GetMotorPitch();CHECK(base!=nullptr && base==GetMotorPitch());
  const auto &cfg=vehicle_config::kPitch;
#endif
  CHECK(base->motor_info().id==cfg.id && base->motor_info().dir==(cfg.reversed?kDirRev:kDirFwd) && base->motor_info().angle_offset==cfg.offset_rad);
  auto *axis=static_cast<DaMiao *>(base);uint8_t data[8]{};
  CHECK(axis->getInput()==0 && !axis->is_enabled() && axis->get_input_type()==InputType::kTorq);
  CHECK(axis->encode(data) && axis->tx_id()==axis->motor_info().id);
  CHECK((data[3]&15)==0 && data[4]==0 && data[5]==0 && (data[6]&0xF0)==0);
  CHECK(axis->motor_info().angle_range==AngleRange::kNegPiToPosPi);
  return 0;
}
