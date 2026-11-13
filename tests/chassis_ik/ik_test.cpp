#include "chassis_iksolver.hpp"
#include "chassis.hpp"
#include "ins_chassis_iksolver.hpp"
#include "ins_robot.hpp"
#include <cmath>
#include <limits>
#include <cstddef>
#include <initializer_list>

using Solver=hello_world::chassis_ik_solver::ChassisIkSolver;
constexpr float pi=3.14159265358979323846f,radius=0.066f;
// 裸 ARM 测试的内存填充实现，不访问外设。
extern "C" void *memset(void *p,int v,std::size_t n) {
  auto *out=static_cast<unsigned char *>(p);for(std::size_t i=0;i<n;++i)out[i]=v;return p;
}
// 裸 ARM 测试的非重叠内存复制实现，返回目标地址。
extern "C" void *memcpy(void *p,const void *source,std::size_t n) {
  auto *out=static_cast<unsigned char *>(p);const auto *in=static_cast<const unsigned char *>(source);
  for(std::size_t i=0;i<n;++i)out[i]=in[i];return p;
}
// 用绝对容差比较浮点结果，不依赖浮点精确相等。
bool near(float a,float b,float tolerance=0.0001f) { return std::fabs(a-b)<tolerance; }
// 生成理想反馈角，仅供测试把速率/对齐影响与运动学结果分开验证。
void feedbackFor(float vx,float vy,float w,float angles[4]) {
  const float x[4]={0.167f,-0.167f,-0.167f,0.167f};
  const float y[4]={0.167f,0.167f,-0.167f,-0.167f};
  for(int i=0;i<4;++i)angles[i]=std::atan2(vy+w*x[i],vx-w*y[i]);
}
// 检查所有驱动轮速度均为零，用于未对齐、停用和故障测试。
bool stopped(const Solver::Result &result) {
  for(const auto &wheel:result.wheels)if(wheel.speed_rad_s!=0)return false;
  return true;
}
#define CHECK(value) do { if(!(value))return __LINE__; } while(0)

// 验证逆解、限幅、速率历史、失效与门控；只运行计算和 getter，不链接电机驱动。
extern "C" int run_tests() {
  Solver solver;
  Solver::Limits limits{1000,10000,0.01f,0.0001f,0.01f};
  float angles[4]{};
  CHECK(solver.solve({1,0,0},0,angles) && stopped(solver.result()));
  CHECK(!solver.result().drive_aligned);
  CHECK(solver.configure(limits));
  CHECK(solver.solve({1,0,0},0,angles));
  for(const auto &wheel:solver.result().wheels)
    CHECK(near(wheel.speed_rad_s,1/radius) && near(wheel.steer_rad,0));

  // 文档旋转式独立核对 0、±90°、180°、整圈回绕以及任意方向的长度保持。
  Solver::BodyMotion body;
  CHECK(Solver::gimbalToBody({1,0,2},pi/2,body));
  CHECK(near(body.vx,0) && near(body.vy,-1) && body.w==2);
  CHECK(Solver::gimbalToBody({1,0,0},-pi/2,body) && near(body.vy,1));
  CHECK(Solver::gimbalToBody({1,0,0},pi,body) && near(body.vx,-1));
  CHECK(Solver::gimbalToBody({1,2,3},2*pi,body));
  CHECK(near(body.vx,1) && near(body.vy,2) && body.w==3);
  CHECK(Solver::gimbalToBody({0.4f,-0.7f,0},1.123f,body));
  CHECK(near(std::hypot(body.vx,body.vy),std::hypot(0.4f,0.7f)));
  for(int i=0;i<4;++i)angles[i]=-pi/2;
  solver.reset();CHECK(solver.solve({1,0,0},pi/2,angles));
  for(const auto &wheel:solver.result().wheels)
    CHECK(near(wheel.steer_rad,-pi/2) && near(wheel.speed_rad_s,1/radius));

  // 纯旋转的四轮力臂和方向，以及平移+旋转的矢量合成。
  const float x[4]={0.167f,-0.167f,-0.167f,0.167f},y[4]={0.167f,0.167f,-0.167f,-0.167f};
  feedbackFor(0,0,1,angles);solver.reset();CHECK(solver.solve({0,0,1},0,angles));
  for(int i=0;i<4;++i) {
    const auto &wheel=solver.result().wheels[i];
    CHECK(near(wheel.speed_rad_s*radius*std::cos(wheel.steer_rad),-y[i]));
    CHECK(near(wheel.speed_rad_s*radius*std::sin(wheel.steer_rad),x[i]));
  }
  feedbackFor(0.3f,0.2f,1,angles);solver.reset();CHECK(solver.solve({0.3f,0.2f,1},0,angles));
  for(int i=0;i<4;++i) {
    const auto &wheel=solver.result().wheels[i];
    CHECK(near(wheel.speed_rad_s*radius*std::cos(wheel.steer_rad),0.3f-y[i]));
    CHECK(near(wheel.speed_rad_s*radius*std::sin(wheel.steer_rad),0.2f+x[i]));
  }

  // 目标与实际舵角相差超过 π/2 时必须选反向轮速，而不是转大角度。
  for(float &a:angles)a=pi;
  solver.reset();CHECK(solver.solve({1,0,0},0,angles));
  for(const auto &wheel:solver.result().wheels)
    CHECK(near(wheel.steer_rad,-pi) && near(wheel.speed_rad_s,-1/radius));
  for(float &a:angles)a=pi/2;
  solver.reset();CHECK(solver.solve({1,0,0},0,angles));
  CHECK(near(solver.result().wheels[0].steer_rad,0));
  for(float &a:angles)a=pi/2+0.001f;
  solver.reset();CHECK(solver.solve({1,0,0},0,angles));
  CHECK(near(Solver::wrapAngle(solver.result().wheels[0].steer_rad+pi),0));

  // 限制最大轮速后，应保持全部轮组的速度比例而非逐轮独立截断。
  Solver::Result unlimited;
  feedbackFor(0.3f,0.2f,1,angles);solver.reset();solver.solve({0.3f,0.2f,1},0,angles);
  unlimited=solver.result();float peak=0;
  for(const auto &wheel:unlimited.wheels)if(std::fabs(wheel.speed_rad_s)>peak)peak=std::fabs(wheel.speed_rad_s);
  limits.max_wheel_speed_rad_s=3;CHECK(solver.configure(limits));CHECK(solver.solve({0.3f,0.2f,1},0,angles));
  for(int i=0;i<4;++i) {
    CHECK(std::fabs(solver.result().wheels[i].speed_rad_s)<=3.00001f);
    CHECK(near(solver.result().wheels[i].speed_rad_s,unlimited.wheels[i].speed_rad_s*3/peak));
  }

  // 持续转向时逐周期核对变化率，零速保持最后舵角而不突然回中。
  limits.max_steer_rate_rad_s=1;solver.configure(limits);
  for(float &a:angles)a=0;
  float previous=0;
  for(int n=0;n<100;++n) {
    CHECK(solver.solve({0,1,0},0,angles));
    const float angle=solver.result().wheels[0].steer_rad;
    CHECK(std::fabs(Solver::wrapAngle(angle-previous))<=0.00101f);
    CHECK(stopped(solver.result()));previous=angle;
    for(float &a:angles)a=angle;
  }
  CHECK(solver.solve({},0,angles));CHECK(near(solver.result().wheels[0].steer_rad,previous));
  // 反馈突然跳到理想角，但限速命令尚未跟上，仍不能立即产生驱动轮速度。
  for(float &a:angles)a=pi/2;
  CHECK(solver.solve({0,1,0},0,angles));CHECK(stopped(solver.result()));
  // 重新从可靠反馈起步后，已对准的轮组可以恢复数学速度目标。
  solver.reset();CHECK(solver.solve({0,1,0},0,angles));CHECK(solver.result().drive_aligned);
  CHECK(near(solver.result().wheels[0].speed_rad_s,3));

  // +179° 到 -179° 应沿最短方向小步跨过回绕点，不能倒转约 358°。
  solver.configure(limits);for(float &a:angles)a=179*pi/180;previous=angles[0];
  for(int n=0;n<50;++n) {
    CHECK(solver.solve({std::cos(-179*pi/180),std::sin(-179*pi/180),0},0,angles));
    const float angle=solver.result().wheels[0].steer_rad;
    const float change=Solver::wrapAngle(angle-previous);
    CHECK(change>=-0.00001f && change<=0.00101f);previous=angle;
    for(float &a:angles)a=angle;
  }
  CHECK(near(Solver::wrapAngle(previous+179*pi/180),0));

  // 某轮组的合速度为零时保持其舵角，不妨碍其余已对准轮组。
  limits.max_steer_rate_rad_s=10000;limits.max_wheel_speed_rad_s=1000;solver.configure(limits);
  feedbackFor(0.167f,-0.167f,1,angles);angles[0]=0.7f;
  CHECK(solver.solve({0.167f,-0.167f,1},0,angles));
  CHECK(solver.result().wheels[0].speed_rad_s==0 && near(solver.result().wheels[0].steer_rad,0.7f));

  // 非有限输入、无效周期、空反馈和溢出均清除结果与历史。
  const float nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
  CHECK(!solver.solve({nan,0,0},0,angles) && !solver.result().valid && stopped(solver.result()));
  CHECK(!solver.solve({0,inf,0},0,angles));CHECK(!solver.solve({0,0,inf},0,angles));
  CHECK(!solver.solve({1,0,0},nan,angles));CHECK(!solver.solve({1,0,0},0,nullptr));
  for(float dt:{0.0f,-1.0f,0.011f,nan,inf})CHECK(!solver.solve({1,0,0},0,angles,dt));
  angles[2]=nan;CHECK(!solver.solve({1,0,0},0,angles));angles[2]=0;
  const float large=std::numeric_limits<float>::max();
  CHECK(!solver.solve({large,large,0},-pi/4,angles));
  CHECK(!solver.result().valid && stopped(solver.result()));
  auto invalid=limits;invalid.max_wheel_speed_rad_s=-1;CHECK(!solver.configure(invalid));
  invalid=limits;invalid.max_steer_rate_rad_s=inf;CHECK(!solver.configure(invalid));
  invalid=limits;invalid.alignment_tolerance_rad=pi/2;CHECK(!solver.configure(invalid));
  invalid=limits;invalid.zero_speed_m_s=-1;CHECK(!solver.configure(invalid));
  invalid=limits;invalid.max_dt_s=0;CHECK(!solver.configure(invalid));
  limits.max_steer_rate_rad_s=0;CHECK(solver.configure(limits));
  CHECK(solver.solve({1,0,1},0,angles) && stopped(solver.result()));

  // 底盘模块只在明确有效的遥控 Working 模式接收目标，其余情形清目标。
  limits.max_steer_rate_rad_s=10000;solver.configure(limits);for(float &a:angles)a=0;
  robot::Chassis chassis(&solver);robot::ModuleState state;
  CHECK(!chassis.updateTargets({1,0,0},0,angles,state,true) && !chassis.targets().valid);
  state={robot::LifeState::Working,robot::ControlSource::RemoteControl,robot::ChassisMode::Separate,robot::GimbalMode::Disabled};
  CHECK(chassis.updateTargets({1,0,0},0,angles,state,true));CHECK(chassis.targets().valid);
  CHECK(!chassis.updateTargets({1,0,0},0,angles,state,false) && stopped(chassis.targets()));
  state.life=robot::LifeState::Resurrection;CHECK(!chassis.updateTargets({1,0,0},0,angles,state,true));
  state.life=robot::LifeState::Dead;CHECK(!chassis.updateTargets({1,0,0},0,angles,state,true));
  state.life=robot::LifeState::Working;state.source=robot::ControlSource::None;
  CHECK(!chassis.updateTargets({1,0,0},0,angles,state,true));
  state.source=robot::ControlSource::RemoteControl;state.chassis_mode=robot::ChassisMode::Disabled;
  CHECK(!chassis.updateTargets({1,0,0},0,angles,state,true));
  state.chassis_mode=static_cast<robot::ChassisMode>(255);
  CHECK(!chassis.updateTargets({1,0,0},0,angles,state,true));
  robot::Chassis no_solver;CHECK(!no_solver.updateTargets({1,0,0},0,angles,state,true));
  CHECK(GetChassisIkSolver()!=nullptr && GetChassisIkSolver()==GetChassisIkSolver());
  CHECK(GetChassis()!=nullptr && GetChassis()==GetChassis());
  CHECK(GetChassisIkSolver()->limits().max_wheel_speed_rad_s==0);
  return 0;
}
