#include "robot_supervisor.hpp"
#include "motor_feedback.hpp"
#include "control_clock.hpp"
#include "test_support.hpp"
#include <limits>
using namespace robot_control;
using namespace robot;
// 仅测试用的正动力参数，不写入生产配置。
Params parameters() {
  Params p;p.remote_mapping_confirmed=true;p.max_vx_m_s=p.max_vy_m_s=1;p.max_w_rad_s=2;
  p.yaw_rate_rad_s=p.pitch_rate_rad_s=0.5f;p.follow.kp=1;p.follow.ki=0.1f;
  p.follow.output_limit=1;p.follow.integral_limit=0.2f;return p;
}
// 构造已完成本地自检和完整对端状态，遥控默认处于右中回中。
ControlInput input() {
  ControlInput in;in.local_ready=in.imu_ready=in.axis_valid=in.peer_valid=true;
  in.peer_state={LifeState::Resurrection,ControlSource::RemoteControl,ChassisMode::Separate,GimbalMode::JointPosition};
  in.peer_flags={true,true,true,true,true,true,true,false};
  in.remote.valid=in.remote.released=in.remote.arm=in.remote.neutral=true;
  in.remote.mode=ChassisMode::Separate;return in;
}
// 为测试明确执行一次右中回中后右上的恢复序列，不允许直接构造 Working。
void enterWorking(Supervisor &s,ControlInput &in) {
  in=input();s.update(in);in.now_ms=201;s.update(in);
  in.remote.arm=false;in.remote.run=true;in.peer_state.life=LifeState::Working;
  in.now_ms++;s.update(in);in.now_ms++;s.update(in);
}
// 验证三态、每个必要门控、两板恢复协商、三模式和相对航向保持。
extern "C" int run_tests() {
  Supervisor defaults(Board::Gimbal,Params{});auto in=input();
  CHECK(!defaults.configured() && defaults.update(in).state.life==LifeState::Dead);
  Supervisor g(Board::Gimbal,parameters());CHECK(g.configured());
  in.remote.arm=false;in.remote.run=true;
  CHECK(!g.update(in).control && g.output().axis_action==AxisAction::Disable && !g.output().command_valid);
  in=input();CHECK(g.update(in).state.life==LifeState::Resurrection && g.output().axis_action==AxisAction::Disable);
  in.now_ms=199;CHECK(g.update(in).axis_action==AxisAction::Disable);
  in.now_ms=200;CHECK(g.update(in).axis_action==AxisAction::Zero && !g.output().control);
  in.remote.run=true;in.remote.arm=false;in.peer_state.life=LifeState::Working;
  in.now_ms++;CHECK(g.update(in).state.life==LifeState::Working && !g.output().control);
  in.now_ms++;CHECK(g.update(in).control);
  for(int gate=0;gate<13;++gate) {
    Supervisor test(Board::Gimbal,parameters());enterWorking(test,in);CHECK(test.output().control);
    if(gate==0)in.remote.valid=false;
    if(gate==1)in.remote.released=false;
    if(gate==2)in.imu_ready=false;
    if(gate==3)in.local_ready=false;
    if(gate==4)in.peer_valid=false;
    if(gate==5)in.peer_flags.ready=false;
    if(gate==6)in.peer_flags.fault=true;
    if(gate==7)in.axis_valid=false;
    if(gate==8)in.peer_flags.yaw_feedback_valid=false;
    if(gate==9)in.dt_s=0.02f;
    if(gate==10)in.remote.motion.vx=std::numeric_limits<float>::quiet_NaN();
    if(gate==11)in.peer_state.source=ControlSource::None;
    if(gate==12)in.peer_state.gimbal_mode=GimbalMode::Disabled;
    CHECK(!test.update(in).control && test.output().motion.vx==0 && test.output().target.yaw==0);
    in=input();in.remote.arm=false;in.remote.run=true;in.peer_state.life=LifeState::Working;
    CHECK(!test.update(in).control); // 有效遥控恢复后仍不能跳过回中。
  }
  Supervisor timeout(Board::Gimbal,parameters());in=input();in.axis_valid=false;timeout.update(in);
  in.now_ms=200;CHECK(timeout.update(in).axis_action==AxisAction::ArmZero);
  in.now_ms=1200;CHECK(timeout.update(in).axis_action==AxisAction::Disable);
  in.axis_valid=true;in.remote.arm=false;in.remote.run=true;in.peer_state.life=LifeState::Working;
  CHECK(!timeout.update(in).control);
  Supervisor nonneutral(Board::Gimbal,parameters());in=input();in.remote.neutral=false;
  nonneutral.update(in);in.now_ms=1000;CHECK(nonneutral.update(in).axis_action==AxisAction::Disable);

  Supervisor c(Board::Chassis,parameters());in=input();
  CHECK(c.update(in).axis_action==AxisAction::Zero);
  in.peer_state.life=LifeState::Working;c.update(in);in.now_ms++;
  in.remote.motion={0.2f,-0.1f,0.4f};CHECK(c.update(in).control && c.output().motion.w==0.4f);
  in.remote.mode=ChassisMode::Follow;in.peer_state.chassis_mode=ChassisMode::Follow;
  CHECK(!c.update(in).control);in.now_ms++;in.axis_angle=0.3f;
  CHECK(c.update(in).control && c.output().motion.w>0.29f && c.output().motion.w<0.31f);
  in.remote.mode=ChassisMode::Spin;in.peer_state.chassis_mode=ChassisMode::Spin;c.update(in);in.now_ms++;in.remote.motion.w=-0.6f;
  CHECK(c.update(in).control && c.output().motion.w==-0.6f);
  in.remote.motion.w=0;CHECK(c.update(in).motion.w==0); // 小陀螺不硬编码常转速度。
  in.remote.motion.vx=2;CHECK(!c.update(in).control);
  Supervisor no_mid(Board::Chassis,parameters());in=input();in.peer_state.life=LifeState::Working;
  CHECK(!no_mid.update(in).control && no_mid.output().axis_action==AxisAction::Disable);
  Supervisor before_hold(Board::Chassis,parameters());in=input();in.peer_flags.command_valid=false;
  CHECK(before_hold.update(in).axis_action==AxisAction::Disable && !before_hold.output().command_valid);
  Supervisor new_hold(Board::Gimbal,parameters());enterWorking(new_hold,in);
  in.remote.run=false;in.remote.arm=true;in.remote.neutral=true;
  CHECK(new_hold.update(in).axis_action==AxisAction::Disable && !new_hold.output().command_valid);
  in.now_ms+=200;CHECK(new_hold.update(in).axis_action==AxisAction::Zero);
  in.now_ms+=2000;CHECK(new_hold.update(in).state.life==LifeState::Resurrection);

  enterWorking(g,in);in.peer_chassis_yaw=0.2f;in.peer_feedback.yaw=0.4f;in.axis_angle=0.1f;
  g.stop();enterWorking(g,in);in.peer_chassis_yaw=0.2f;in.peer_feedback.yaw=0.4f;
  // 模式切换重新从当前反馈播种保持目标，再验证车头转动后的关节目标补偿。
  in.remote.mode=ChassisMode::Spin;in.peer_state.chassis_mode=ChassisMode::Spin;g.update(in);in.now_ms++;g.update(in);
  const float hold=g.output().target.yaw+in.peer_chassis_yaw;
  in.peer_chassis_yaw+=0.1f;g.update(in);
  CHECK(std::fabs(g.output().target.yaw+in.peer_chassis_yaw-hold)<0.00001f);
  in.remote.pitch_rate=0.5f;g.acceptPitchTarget(0.2f);g.update(in);
  CHECK(g.output().target.pitch>0.2f && g.output().target.pitch<0.201f);

  MotorFeedback feedback;uint8_t frame[8]{};frame[6]=30;
  CHECK(!feedback.fresh(0,0));feedback.recordDji(0,frame,0xFFFFFFF0);CHECK(feedback.fresh(0,0xFFFFFFF1));
  CHECK(!feedback.fresh(0,0x54));frame[0]=0xFF;feedback.recordDji(0,frame,0);CHECK(!feedback.fresh(0,0));
  frame[0]=0x11;frame[7]=30;feedback.recordAxis(1,frame,0);CHECK(feedback.axisEnabled(0));
  feedback.recordAxis(2,frame,1);CHECK(!feedback.axisEnabled(1));
  frame[0]=0x81;feedback.recordAxis(1,frame,2);CHECK(!feedback.fresh(8,2));
  frame[0]=1;feedback.recordAxis(1,frame,3);CHECK(feedback.fresh(8,3) && !feedback.axisEnabled(3));
  frame[0]=0x11;feedback.recordAxis(1,frame,4,true);CHECK(!feedback.axisEnabled(4));
  // 按线上字段而非共享状态模拟两板从 Dead 启动，验证自检/回中许可没有循环等待。
  Supervisor primary(Board::Gimbal,parameters()),secondary(Board::Chassis,parameters());
  ControlOutput pg,pc;bool pitch_enabled=false,yaw_enabled=false;
  for(int ms=0;ms<230;++ms) {
    auto gi=input();gi.now_ms=ms;gi.axis_valid=pitch_enabled;
    gi.remote.arm=ms<210;gi.remote.run=ms>=210;
    gi.peer_state=pc.state;gi.peer_flags={pc.ready,true,yaw_enabled,false,pc.remote_valid,pc.emergency_released,pc.command_valid,false};
    pg=primary.update(gi);if(pg.axis_action==AxisAction::ArmZero)pitch_enabled=true;
    auto ci=input();ci.now_ms=ms;ci.axis_valid=yaw_enabled;
    ci.peer_state=pg.state;ci.peer_flags={pg.ready,true,false,pitch_enabled,pg.remote_valid,pg.emergency_released,pg.command_valid,false};
    ci.remote.valid=pg.remote_valid;ci.remote.released=pg.emergency_released;ci.remote.mode=pg.state.chassis_mode;
    pc=secondary.update(ci);if(pc.axis_action==AxisAction::ArmZero)yaw_enabled=true;
    if(ms<200)CHECK(!pitch_enabled && !yaw_enabled);
    if(ms<210)CHECK(!pg.control && !pc.control);
  }
  CHECK(pg.control && pc.control && pitch_enabled && yaw_enabled);
  ControlClock clock;
  CHECK(clock.step(0,0,168000000,true)==0.001f);
  CHECK(clock.step(168000,1,168000000,true)>0);
  CHECK(clock.step(336000,1,168000000,true)>0);
  CHECK(clock.step(504000,1,168000000,true)>0);
  CHECK(clock.step(672000,1,168000000,true)==0); // HAL 时间冻结 3ms，不能让旧遥控永不过期。
  CHECK(clock.step(840000,2,168000000,true)>0);
  CHECK(clock.step(840000,2,168000000,true)==0); // CPU 周期不推进，同样拒绝。
  CHECK(clock.step(1008000,3,168000000,false)==0);
  CHECK(clock.step(1176000,4,0,true)==0);
  ControlClock wrapping_clock;wrapping_clock.step(0xFFFF0000,10,168000000,true);
  CHECK(wrapping_clock.step(0x19040,11,168000000,true)>0); // unsigned 周期差跨 32 位回绕。
  return 0;
}
