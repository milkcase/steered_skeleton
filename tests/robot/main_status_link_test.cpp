#include "gimbal_chassis_comm.hpp"
#include "robot_supervisor.hpp"
#include "test_support.hpp"
using Comm=robot::GimbalChassisComm;
uint32_t status_now=0;
// 模拟实际毫秒推进；每3ms只交付一页，页间每1ms读取一次状态。
uint32_t hello_world::tick::GetTickMs() {return status_now;}
// 设置无动力的正常Main心跳，不使用调试预演标志，也不伪造Working许可。
void safeStatus(Comm &sender,bool chassis,robot::LifeState life,bool released) {
  robot::ModuleState state;
  if(life!=robot::LifeState::Dead)state={life,robot::ControlSource::RemoteControl,robot::ChassisMode::Separate,robot::GimbalMode::JointPosition};
  Comm::Flags flags;flags.ready=flags.imu_ready=flags.remote_valid=true;flags.emergency_released=released;
  flags.command_valid=life==robot::LifeState::Resurrection;
  if(chassis) {sender.chassis_data().state=state;sender.chassis_data().flags=flags;}
  else {sender.gimbal_data().state=state;sender.gimbal_data().flags=flags;}
}
// 对不同方向统一读取“完整状态是否可读”，不拿它代替commandsUsable运动许可。
bool statusReadable(Comm &receiver,bool chassis_sender) {
  if(chassis_sender) {Comm::ChassisData data;return receiver.readPeer(data);}
  Comm::GimbalData data;return receiver.readPeer(data);
}
// 逐毫秒复现右下安全心跳及右中恢复期间的peer，含Dead→Resurrection正常升级。
int heartbeat(bool chassis_sender) {
  Comm sender(chassis_sender?Comm::CodePart::Chassis:Comm::CodePart::Gimbal,chassis_sender?0x210:0x211,chassis_sender?0x211:0x210);
  Comm receiver(chassis_sender?Comm::CodePart::Gimbal:Comm::CodePart::Chassis,chassis_sender?0x211:0x210,chassis_sender?0x210:0x211);
  for(uint32_t elapsed=0;elapsed<90;++elapsed) {
    status_now=elapsed;
    safeStatus(sender,chassis_sender,elapsed<30?robot::LifeState::Dead:robot::LifeState::Resurrection,elapsed>=30);
    if(elapsed%3==0) {uint8_t frame[8];sender.encode(frame);receiver.decode(frame);}
    if(elapsed>=6)CHECK(statusReadable(receiver,chassis_sender));
    CHECK(!receiver.commandsUsable());
  }
  // 真实安全位下降仍在第一页立即撤销；后续完整安全快照可读但不能授权运动。
  safeStatus(sender,chassis_sender,robot::LifeState::Dead,false);
  uint8_t frame[8];sender.encode(frame);receiver.decode(frame);
  CHECK(!statusReadable(receiver,chassis_sender) && !receiver.commandsUsable());
  for(int i=0;i<2;++i) {++status_now;sender.encode(frame);receiver.decode(frame);}
  CHECK(statusReadable(receiver,chassis_sender) && !receiver.commandsUsable());
  // 坏帧不能因为“只是安全状态”而沿用缓存，必须重新收齐完整、有效的新快照。
  sender.encode(frame);frame[4]^=1;receiver.decode(frame);CHECK(!statusReadable(receiver,chassis_sender));
  sender.restartTx();for(int i=0;i<3;++i) {++status_now;sender.encode(frame);receiver.decode(frame);}
  CHECK(statusReadable(receiver,chassis_sender));status_now+=100;CHECK(!statusReadable(receiver,chassis_sender));
  return 0;
}
// 将纯状态机输出发布成真实协议字段，不直接共享对端运行状态。
void publishState(Comm &comm,bool chassis,const robot_control::ControlOutput &out,bool imu_ready=true) {
  Comm::Flags f;f.ready=out.ready;f.imu_ready=imu_ready;f.remote_valid=out.remote_valid;
  f.emergency_released=out.emergency_released;f.command_valid=out.command_valid;
  if(chassis) {f.yaw_feedback_valid=true;auto &d=comm.chassis_data();d.state=out.state;d.flags=f;d.target=out.target;}
  else {f.pitch_feedback_valid=true;auto &d=comm.gimbal_data();d.state=out.state;d.flags=f;d.motion=out.motion;d.yaw_target=out.target.yaw;}
}
// 两个真实Supervisor经3ms分页错相通信：右下→右中回中→右上，不能在页间互相撤销准备。
int coupledSupervisors() {
  robot_control::Params p;p.remote_mapping_confirmed=true;p.separate_only=true;p.start_level_pitch=true;
  p.max_vx_m_s=p.max_vy_m_s=p.max_w_rad_s=1;p.yaw_rate_rad_s=p.pitch_rate_rad_s=0.5f;
  p.follow={.kp=1,.output_limit=1,.error_period=6.283185307179586f};
  robot_control::Supervisor base(robot_control::Board::Chassis,p),cloud(robot_control::Board::Gimbal,p);
  Comm base_comm(Comm::CodePart::Chassis,0x210,0x211),cloud_comm(Comm::CodePart::Gimbal,0x211,0x210);
  for(uint32_t elapsed=0;elapsed<650;++elapsed) {
    status_now=elapsed;Comm::ChassisData from_base;Comm::GimbalData from_cloud;
    robot_control::ControlInput bi,gi;
    bi.now_ms=gi.now_ms=elapsed;bi.imu_ready=gi.imu_ready=bi.local_ready=gi.local_ready=bi.axis_valid=gi.axis_valid=true;
    if(elapsed>=600)gi.imu_ready=gi.local_ready=false; // 模拟云台IMU真实故障，不能靠peer连续性放行。
    bi.peer_valid=base_comm.readPeer(from_cloud);bi.peer_state=from_cloud.state;bi.peer_flags=from_cloud.flags;bi.peer_yaw_target=from_cloud.yaw_target;
    gi.peer_valid=cloud_comm.readPeer(from_base);gi.peer_state=from_base.state;gi.peer_flags=from_base.flags;gi.peer_feedback=from_base.feedback;
    gi.remote.valid=true;gi.remote.released=elapsed>=50;gi.remote.arm=elapsed>=50 && elapsed<450;
    gi.remote.run=elapsed>=450;gi.remote.neutral=true;gi.remote.mode=robot::ChassisMode::Separate;
    bi.remote.valid=bi.peer_valid && from_cloud.flags.remote_valid;bi.remote.released=from_cloud.flags.emergency_released;
    bi.remote.arm=from_cloud.state.life==robot::LifeState::Resurrection;bi.remote.run=from_cloud.state.life==robot::LifeState::Working;
    bi.remote.mode=from_cloud.state.chassis_mode;bi.remote.motion=from_cloud.motion;
    const auto &bo=base.update(bi);const auto &go=cloud.update(gi);publishState(base_comm,true,bo);publishState(cloud_comm,false,go,gi.imu_ready);
    uint8_t frame[8];
    if(elapsed%3==0) {base_comm.encode(frame);cloud_comm.decode(frame);}
    if(elapsed%3==1) {cloud_comm.encode(frame);base_comm.decode(frame);}
    if(elapsed>=20 && elapsed<600)CHECK(bi.peer_valid && gi.peer_valid);
    if(elapsed>=500 && elapsed<600)CHECK(bo.control && go.control);
    if(elapsed>=610)CHECK(!bo.control && !go.control);
    if(elapsed>=630)CHECK(bi.peer_valid && gi.peer_valid); // 故障状态仍在线，但两板动力保持禁止。
  }
  return 0;
}
// 恢复阶段仍可能申请零输入使能；任何真实安全位下降必须在第一页撤销旧状态。
int safetyFalls() {
  for(int condition=0;condition<8;++condition) {
    Comm sender(Comm::CodePart::Chassis,0x210,0x211),receiver(Comm::CodePart::Gimbal,0x211,0x210);
    auto &data=sender.chassis_data();data.state={robot::LifeState::Resurrection,robot::ControlSource::RemoteControl,
      robot::ChassisMode::Separate,robot::GimbalMode::JointPosition};
    data.flags={true,true,true,true,true,true,true,false};
    uint8_t frame[8];for(int i=0;i<3;++i) {++status_now;sender.encode(frame);receiver.decode(frame);}
    CHECK(statusReadable(receiver,true) && !receiver.commandsUsable());
    bool *fields[]={&data.flags.ready,&data.flags.imu_ready,&data.flags.yaw_feedback_valid,&data.flags.pitch_feedback_valid,
      &data.flags.remote_valid,&data.flags.emergency_released,&data.flags.command_valid};
    if(condition==7)data.flags.fault=true;else *fields[condition]=false;
    ++status_now;sender.encode(frame);receiver.decode(frame);
    CHECK(!statusReadable(receiver,true) && !receiver.commandsUsable());
  }
  return 0;
}
// 同时验证通信连续性与原有即时安全撤销，不连接硬件、不发送电机命令。
extern "C" int run_tests() {
  int result=heartbeat(true);if(result)return result;
  result=heartbeat(false);if(result)return result;
  result=safetyFalls();if(result)return result;
  return coupledSupervisors();
}
