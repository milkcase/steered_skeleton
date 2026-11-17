#include "chassis_iksolver.hpp"
#include "gimbal_chassis_comm.hpp"
#include "vehicle_motor_config.hpp"
#include "test_support.hpp"
#include <cmath>
uint32_t now_ms=0;
// 软件测试时钟，协议解码仍执行真实CRC/分页/时效检查。
uint32_t hello_world::tick::GetTickMs() {return now_ms;}
// 验证未对准仍有连续投影轮速、坐标旋转、180度等效解，以及方向0与回中不混淆。
extern "C" int run_tests() {
  using S=hello_world::chassis_ik_solver::ChassisIkSolver;
  S::Limits p;p.max_wheel_speed_rad_s=100;p.max_steer_rate_rad_s=1000;p.max_dt_s=0.003f;p.project_drive_speed=true;
  S solver(p);float angles[4]{};const float radius=vehicle_config::kWheelRadiusM;
  CHECK(solver.solve({0.2f,0.2f,0},0,angles));
  CHECK(!solver.result().drive_aligned);
  for(auto &w:solver.result().wheels)CHECK(std::fabs(w.speed_rad_s-0.2f/radius)<0.0001f);
  CHECK(solver.solve({-0.2f,0,0},0,angles));
  CHECK(solver.result().wheels[0].speed_rad_s<0);
  for(float &a:angles)a=0.7853981634f;
  CHECK(solver.solve({0.2f,0,0},1.5707963268f,angles));
  CHECK(std::fabs(solver.result().wheels[0].speed_rad_s-0.2f/radius*0.70710678f)<0.0001f);
  using C=robot::GimbalChassisComm;
  C g(C::CodePart::Gimbal,0x211,0x210),c(C::CodePart::Chassis,0x210,0x211);
  auto &d=g.gimbal_data();d.flags.debug_preview=d.flags.debug_power=true;
  d.flags.remote_valid=d.flags.emergency_released=true;
  d.preview={0,1,1,false,true,true,0.5f};
  for(int i=0;i<3;++i) {uint8_t data[8];g.encode(data);c.decode(data);++now_ms;}
  C::GimbalData got;CHECK(c.readPeer(got));
  CHECK(got.preview.steer==0 && !got.preview.neutral && got.preview.magnitude==0.5f);
  d.preview.magnitude=1.1f;uint8_t invalid[8];g.encode(invalid);CHECK(!g.lastEncodeValid());
  return 0;
}
