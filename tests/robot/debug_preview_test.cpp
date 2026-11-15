#include "motor_debug_preview.hpp"
#include "gimbal_chassis_comm.hpp"
#include "test_support.hpp"
#include <limits>
using namespace robot_debug;
using Comm=robot::GimbalChassisComm;
uint32_t test_ms=0;
// 模拟独立毫秒时钟，测试不连接外设或真实电机。
uint32_t hello_world::tick::GetTickMs() {return test_ms;}
// 比较预演角度的正常浮点舍入误差。
bool near(float a,float b) {return std::fabs(a-b)<0.0001f;}
// 创建全部真实条件成立的测试输入；这些许可不写入实车状态。
Input healthy() {
  Input in;in.transport=in.imu=in.peer=in.feedback=in.remote=true;
  in.neutral=in.other_neutral=true;in.angle=0.2f;return in;
}
// 明确执行右下、右中200ms、右上的准备过程，不直接写预演器内部状态。
void prepare(Preview &preview,Input &in) {
  in=healthy();preview.step(in);in.right=3;in.now_ms=1;preview.step(in);
  in.now_ms=201;preview.step(in);in.right=1;in.now_ms=202;preview.step(in);
}
// 通过实际CRC/分页协议交付一次完整快照，不共享两板运行对象。
void transfer(Comm &sender,Comm &receiver) {
  uint8_t bytes[8];for(int i=0;i<3;++i) {sender.encode(bytes);receiver.decode(bytes);}
}
// 验证真实预演状态机、锁止、回绕和协议隔离；任何路径都不能得到动力输出。
extern "C" int run_tests() {
  auto in=healthy();Preview disabled(false,true);disabled.step(in);
  CHECK(disabled.diagnostics().stage==Stage::Disabled && disabled.diagnostics().final_input==0);
  Preview preview(true,true);in.right=1;preview.step(in);
  CHECK(preview.diagnostics().stage==Stage::WaitReset);
  prepare(preview,in);CHECK(preview.diagnostics().stage==Stage::WaitStick);
  in.steer=-0.5f;in.neutral=false;in.now_ms=203;preview.step(in);
  CHECK(preview.diagnostics().stage==Stage::Previewing && preview.diagnostics().run_count==1);
  CHECK(preview.diagnostics().target_angle>in.angle && preview.diagnostics().final_input==0);
  const auto first=preview.diagnostics().target_angle;
  in.steer=0;in.neutral=true;in.now_ms++;preview.step(in);
  CHECK(preview.diagnostics().last_stop==Reason::Centered && preview.diagnostics().stop_count==1);
  CHECK(near(preview.diagnostics().recorded_target,first));
  in.steer=-1;in.neutral=false;
  for(int n=0;n<700;++n) {++in.now_ms;preview.step(in);}
  CHECK(preview.diagnostics().run_count==1 && preview.diagnostics().final_input==0);
  prepare(preview,in);in.now_ms=203;in.steer=1;in.neutral=false;preview.step(in);
  CHECK(preview.diagnostics().target_angle<in.angle);
  in.now_ms=702;preview.step(in);CHECK(preview.diagnostics().remaining_ms==1);
  in.now_ms=703;preview.step(in);CHECK(preview.diagnostics().last_stop==Reason::TimeLimit);
  const auto count=preview.diagnostics().run_count;
  for(int n=0;n<600;++n) {++in.now_ms;preview.step(in);}
  CHECK(preview.diagnostics().run_count==count);
  // 右中回中且真实条件始终成立时，Ready必须稳定保持；不能因时间经过自行回WaitReset。
  Preview stable(true,true);in=healthy();stable.step(in);in.right=3;in.now_ms=1;stable.step(in);
  in.now_ms=201;stable.step(in);CHECK(stable.diagnostics().stage==Stage::Ready);
  for(int ms=202;ms<2202;++ms) {
    in.now_ms=ms;stable.step(in);CHECK(stable.diagnostics().stage==Stage::Ready && stable.diagnostics().prepare_loss.count==0);
  }
  // 在准备完成后逐个注入短暂失效；条件恢复仍需右下复位，同时保留第一原因而非NeedReset。
  const Reason prepare_reasons[]={Reason::Transport,Reason::Imu,Reason::Peer,Reason::Feedback,
      Reason::Remote,Reason::Clock,Reason::NeedNeutral,Reason::Mode};
  for(int fault=0;fault<8;++fault) {
    Preview preparing(true,true);in=healthy();preparing.step(in);in.right=3;in.now_ms=1;preparing.step(in);
    in.now_ms=201;preparing.step(in);
    if(fault==0)in.transport=false;if(fault==1)in.imu=false;if(fault==2)in.peer=false;
    if(fault==3)in.feedback=false;if(fault==4)in.remote=false;if(fault==5)in.dt_s=0;
    if(fault==6)in.neutral=false;if(fault==7)in.left=3;
    preparing.step(in);CHECK(preparing.diagnostics().prepare_loss.reason==prepare_reasons[fault]);
    CHECK(preparing.diagnostics().prepare_loss.count==1 && preparing.diagnostics().prepare_loss.from_stage==Stage::Ready);
    in=healthy();in.right=3;
    for(int ms=202;ms<502;++ms) {in.now_ms=ms;preparing.step(in);}
    CHECK(preparing.diagnostics().stage==Stage::WaitReset && preparing.diagnostics().reason==Reason::NeedReset);
    CHECK(preparing.diagnostics().prepare_loss.reason==prepare_reasons[fault] && preparing.diagnostics().prepare_loss.count==1);
    CHECK(preparing.diagnostics().final_input==0);
  }
  // 活动中逐个丢失真实条件或改变拨杆，均记录停止，之后不能直接右上恢复。
  const Reason expected[]={Reason::Transport,Reason::Imu,Reason::Peer,Reason::Feedback,Reason::Remote,
      Reason::Mode,Reason::Clock,Reason::Sample,Reason::SwitchChanged,Reason::Emergency};
  for(int fault=0;fault<10;++fault) {
    Preview trial(true,true);prepare(trial,in);in.steer=-1;in.neutral=false;in.now_ms=203;trial.step(in);
    if(fault==0)in.transport=false;if(fault==1)in.imu=false;if(fault==2)in.peer=false;
    if(fault==3)in.feedback=false;if(fault==4)in.remote=false;if(fault==5)in.left=3;
    if(fault==6)in.dt_s=0;if(fault==7)in.angle=std::numeric_limits<float>::quiet_NaN();
    if(fault==8)in.right=3;if(fault==9)in.right=2;
    trial.step(in);CHECK(trial.diagnostics().last_stop==expected[fault]);
    CHECK(trial.diagnostics().final_input==0 && trial.diagnostics().stop_count==1);
  }
  // 目标在±π附近仍沿最短方向预演；系统毫秒回绕不延长500ms窗口。
  prepare(preview,in);in.angle=3.14158f;in.steer=-1;in.neutral=false;
  in.now_ms=0xfffffff0u;preview.step(in);CHECK(preview.diagnostics().target_angle<0);
  in.now_ms=0xfffffff0u+500u;preview.step(in);CHECK(preview.diagnostics().last_stop==Reason::TimeLimit);
  Preview observer(true,false);in=healthy();in.right=1;observer.step(in);
  CHECK(observer.diagnostics().stage==Stage::Observer && observer.diagnostics().actual_valid==0);
  in.steer=0.5f;observer.step(in);CHECK(observer.diagnostics().reason==Reason::Sample);
  publish(preview.diagnostics());CHECK(robot_motor_debug_diagnostics.final_input==0);
  CHECK(robot_motor_debug_diagnostics.sequence%2==0);
  // 对端反馈失效可只丢peer，快照内遥控失效可只丢remote，诊断不能把它们一律归作掉线。
  Preview diagnostic(false,true);in=healthy();
  in.link.snapshot_valid=in.link.preview_mode=in.link.peer_imu=in.link.peer_axis=in.link.remote_in_snapshot=1;
  diagnostic.step(in);in.peer=false;in.link.peer_axis=0;diagnostic.step(in);
  CHECK(diagnostic.diagnostics().link.peer_drops==1 && diagnostic.diagnostics().link.remote_drops==0);
  CHECK(diagnostic.diagnostics().link.last_peer_drop_mask==8);
  in.peer=true;in.link.peer_axis=1;in.remote=false;in.link.remote_in_snapshot=0;diagnostic.step(in);
  CHECK(diagnostic.diagnostics().link.remote_drops==1 && diagnostic.diagnostics().link.last_remote_snapshot_valid==1);
  CHECK(diagnostic.diagnostics().link.last_remote_source_valid==0);

  Comm chassis(Comm::CodePart::Chassis,0x210,0x211),gimbal(Comm::CodePart::Gimbal,0x211,0x210);
  auto &g=gimbal.gimbal_data();g.flags.debug_preview=true;g.flags.remote_valid=true;g.flags.emergency_released=true;
  g.preview={-0.5f,1,1,false,true,true};transfer(gimbal,chassis);
  Comm::GimbalData received;CHECK(chassis.readPeer(received));
  CHECK(received.flags.debug_preview && near(received.preview.steer,-0.5f) && received.preview.right==1);
  CHECK(received.motion.vx==0 && received.motion.vy==0 && received.motion.w==0 && !chassis.commandsUsable());
  test_ms=100;CHECK(!chassis.readPeer(received));
  test_ms=101;g.preview.right=3;g.preview.steer=0;g.preview.neutral=true;transfer(gimbal,chassis);
  CHECK(chassis.readPeer(received) && received.preview.right==3);
  // 按现场3ms分页，在每页之间读取上一份完整预演快照；正常拼包不应周期性撤销它。
  uint8_t partial[8];
  for(int page=0;page<3;++page) {
    test_ms+=3;gimbal.encode(partial);chassis.decode(partial);
    CHECK(chassis.readPeer(received) && received.flags.debug_preview && received.preview.valid);
    CHECK(!chassis.commandsUsable());
  }
  // 新第一页已报告关遥控/急停时，旧有效遥控立即不可读，未放宽控制撤销规则。
  g.flags.remote_valid=g.flags.emergency_released=false;g.preview.valid=false;g.preview.right=2;
  gimbal.encode(partial);chassis.decode(partial);CHECK(!chassis.readPeer(received));
  for(int page=1;page<3;++page) {test_ms+=3;gimbal.encode(partial);chassis.decode(partial);}
  CHECK(chassis.readPeer(received) && !received.preview.valid && !chassis.commandsUsable());
  g.flags.remote_valid=g.flags.emergency_released=true;g.preview.valid=true;g.preview.right=3;
  transfer(gimbal,chassis);CHECK(chassis.readPeer(received));
  // 坏CRC撤销快照后，收到一页好数据也不能提前恢复；必须等新快照全部通过。
  gimbal.encode(partial);partial[7]^=1;chassis.decode(partial);CHECK(!chassis.readPeer(received));
  gimbal.encode(partial);chassis.decode(partial);CHECK(!chassis.readPeer(received));
  for(int page=0;page<4;++page) {gimbal.encode(partial);chassis.decode(partial);}
  CHECK(chassis.readPeer(received) && chassis.rxDiagnostics().crc_errors==1);
  const auto completed=chassis.rxDiagnostics().completed;
  const auto age=chassis.peerAgeMs();test_ms+=100;
  CHECK(!chassis.readPeer(received) && chassis.peerAgeMs()==age+100);
  CHECK(chassis.rxDiagnostics().completed==completed);
  // 即使有人把预演快照的Working/许可位改为真，真实编码器也拒绝将其发布成控制命令。
  g.state.life=robot::LifeState::Working;g.flags.ready=g.flags.command_valid=true;
  transfer(gimbal,chassis);CHECK(!gimbal.lastEncodeValid() && !chassis.commandsUsable());
  return 0;
}
