#include "gimbal_chassis_comm.hpp"
#include "board_comm_task.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"
#include "can.h"
#include <cmath>
#include <limits>
#include <cstddef>
using Comm=robot::GimbalChassisComm;
uint32_t test_now=0,test_irq=0,free_mailboxes=3;
int notifications=0,sends=0;
HAL_CAN_StateTypeDef can_state=HAL_CAN_STATE_READY;
HAL_StatusTypeDef filter_result=HAL_OK,start_result=HAL_OK,send_result=HAL_OK;
HAL_StatusTypeDef notify_result=HAL_OK;
CAN_HandleTypeDef hcan1{1},hcan2{2};
CAN_HandleTypeDef *filtered_can=nullptr,*started_can=nullptr,*notified_can=nullptr,*sent_can=nullptr;
uint32_t last_notification=0;
#if defined(TEST_CHASSIS)
CAN_HandleTypeDef *const expected_comm_can=&hcan2;
constexpr uint32_t expected_bank=14,expected_fifo=CAN_FILTER_FIFO1,expected_notification=CAN_IT_RX_FIFO1_MSG_PENDING;
#else
CAN_HandleTypeDef *const expected_comm_can=&hcan1;
constexpr uint32_t expected_bank=0,expected_fifo=CAN_FILTER_FIFO0,expected_notification=CAN_IT_RX_FIFO0_MSG_PENDING;
#endif
CAN_RxHeaderTypeDef injected_header{};
CAN_TxHeaderTypeDef sent_header{};
CAN_FilterTypeDef captured_filter{};
uint8_t injected_data[8]{},sent_data[8]{};
hello_world::motor::Motor wheels[4],steers[4],yaw,pitch;
// 裸 ARM 测试内存填充，不访问外设。
extern "C" void *memset(void *p,int v,std::size_t n) {
  auto *out=static_cast<uint8_t *>(p);for(std::size_t i=0;i<n;++i)out[i]=v;return p;
}
// 裸 ARM 测试非重叠内存复制。
extern "C" void *memcpy(void *p,const void *s,std::size_t n) {
  auto *out=static_cast<uint8_t *>(p);auto *in=static_cast<const uint8_t *>(s);
  for(std::size_t i=0;i<n;++i)out[i]=in[i];return p;
}
// 返回可控毫秒数，模拟时间回绕和断线。
uint32_t hello_world::tick::GetTickMs() {return test_now;}
// 捕获滤波器配置，返回测试设置的结果。
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can,CAN_FilterTypeDef *p) {filtered_can=can;captured_filter=*p;return filter_result;}
// 返回模拟 CAN 状态。
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *) {return can_state;}
// 模拟 CAN 启动，失败时不改变监听状态。
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can) {started_can=can;if(start_result==HAL_OK)can_state=HAL_CAN_STATE_LISTENING;return start_result;}
// 记录真实接收源文件是否恢复通知。
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can,uint32_t notification) {notified_can=can;last_notification=notification;++notifications;return notify_result;}
// 返回当前可用邮箱，模拟满载。
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *) {return free_mailboxes;}
// 记录拟发送的协议页，不驱动 CAN 适配器。
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can,CAN_TxHeaderTypeDef *h,uint8_t *p,uint32_t *) {
  sent_can=can;
  ++sends;sent_header=*h;memcpy(sent_data,p,8);return send_result;
}
// 向真实接收回调提供注入的头部和数据。
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *,uint32_t,CAN_RxHeaderTypeDef *h,uint8_t *p) {
  *h=injected_header;memcpy(p,injected_data,8);return HAL_OK;
}
// 错误处理空桩；启动失败应通过新服务返回 false，而非进入此分支。
void Error_Handler() {}
// 向真实板间接口注入，或刻意注入另一总线，验证正确页也不能从错总线续命。
void receiveComm(bool wrong_bus=false) {
  auto *can=expected_comm_can;
  if(wrong_bus)can=can==&hcan1?&hcan2:&hcan1;
  if(can==&hcan1)HAL_CAN_RxFifo0MsgPendingCallback(can);
  else HAL_CAN_RxFifo1MsgPendingCallback(can);
}
// 提供本板测试驱动反馈对象。
hello_world::motor::Motor *GetMotorWheel(int i) {return &wheels[i];}
// 提供本板测试转向反馈对象。
hello_world::motor::Motor *GetMotorSteer(int i) {return &steers[i];}
// 提供本板测试 yaw 反馈对象。
hello_world::motor::Motor *GetMotorYaw() {return &yaw;}
// 提供本板测试 pitch 反馈对象。
hello_world::motor::Motor *GetMotorPitch() {return &pitch;}
// 独立逐位 CRC 参考实现，与生产使用的查表实现相互校验。
uint8_t crc(const uint8_t *p) {
  uint8_t value=0;for(int i=0;i<7;++i) {value^=p[i];for(int b=0;b<8;++b)value=(value&128)?(value<<1)^7:value<<1;}
  return value;
}
// 仅为测试构造完整许可，实际发布端必须由真实遥控和硬件状态生成。
robot::ModuleState working() {return {robot::LifeState::Working,robot::ControlSource::RemoteControl,robot::ChassisMode::Separate,robot::GimbalMode::JointPosition};}
// 仅为测试构造各项就绪位，不写入生产默认值。
Comm::Flags flags() {return {true,true,true,true,true,true,true,false};}
// 设置 C2G 测试目标与反馈，字段包含正负和非整数值。
void activateC(Comm &comm) {
  auto &d=comm.chassis_data();d.state=working();d.flags=flags();
  d.target={1.23456f,-0.22226f};d.feedback={-1.23456f,0.12345f};d.chassis_yaw=0.87654f;
}
// 设置 G2C 测试速度、yaw 目标及反馈。
void activateG(Comm &comm) {
  auto &d=comm.gimbal_data();d.state=working();d.flags=flags();
  d.motion={1.234f,-0.875f,2.25f};d.yaw_target=-1.11116f;d.feedback={1.99996f,-0.34565f};
}
// 调用真实 encode 三次，收集同一个快照的三页。
void pages(Comm &sender,uint8_t out[3][8]) {for(int i=0;i<3;++i)sender.encode(out[i]);}
// 向真实解码器依序送入快照，每页间隔 1 ms。
void deliver(Comm &receiver,const uint8_t in[3][8]) {for(int i=0;i<3;++i) {receiver.decode(in[i]);++test_now;}}
// 比较比例编码后的浮点数，容许半个量化步长和舍入误差。
bool near(float a,float b,float tolerance=0.000051f) {return std::fabs(a-b)<tolerance;}
#define CHECK(value) do {if(!(value))return __LINE__;}while(0)

// 验证真实编解码、时间/序号、CAN2 服务和接收路由；失败返回行号，不访问真实总线。
extern "C" int run_tests() {
  Comm c(Comm::CodePart::Chassis,0x210,0x211),g(Comm::CodePart::Gimbal,0x211,0x210);
  CHECK(c.isOffline() && g.isOffline() && !c.commandsUsable());
  activateC(c);activateG(g);uint8_t frame[3][8]{};
  pages(c,frame);
  CHECK(frame[0][0]==0xA0 && frame[1][0]==0xA1 && frame[2][0]==0xA2);
  CHECK(frame[0][1]==1 && frame[1][1]==1 && frame[2][1]==1);
  CHECK(frame[0][2]==0x36 && frame[0][3]==0x7F && frame[0][4]==0x30 && frame[0][5]==0x3A);
  for(auto &p:frame)CHECK(p[7]==crc(p));
  g.decode(frame[0]);g.decode(frame[1]);CHECK(g.isOffline());g.decode(frame[2]);
  Comm::ChassisData cd;CHECK(g.readPeer(cd) && g.commandsUsable());
  CHECK(near(cd.target.yaw,1.23456f) && near(cd.target.pitch,-0.22226f));
  CHECK(near(cd.feedback.yaw,-1.23456f) && near(cd.feedback.pitch,0.12345f) && near(cd.chassis_yaw,0.87654f));
  pages(g,frame);deliver(c,frame);Comm::GimbalData gd;
  CHECK(c.readPeer(gd) && c.commandsUsable());
  CHECK(near(gd.motion.vx,1.234f) && near(gd.motion.vy,-0.875f) && near(gd.motion.w,2.25f));
  CHECK(near(gd.yaw_target,-1.11116f) && near(gd.feedback.yaw,1.99996f));
  CHECK(!c.readPeer(cd) && cd.target.yaw==0);CHECK(!g.readPeer(gd) && gd.motion.vx==0);

  // 发送期间字段变化不混入本轮；下一轮才发布新值。
  uint8_t frozen[3][8];c.encode(frozen[0]);c.chassis_data().target.pitch=0.8f;
  c.encode(frozen[1]);c.encode(frozen[2]);deliver(g,frozen);g.readPeer(cd);CHECK(near(cd.target.pitch,-0.22226f));
  pages(c,frame);deliver(g,frame);g.readPeer(cd);CHECK(near(cd.target.pitch,0.8f));
  // 不完整急停页立即撤销旧指令，不等第三页；发送端会抢占未完成的活动快照。
  c.encode(frozen[0]);c.chassis_data().flags.emergency_released=false;c.encode(frame[0]);
  CHECK(frame[0][0]==0xA0 && frame[0][1]!=frozen[0][1]);g.decode(frame[0]);
  CHECK(!g.commandsUsable() && !g.readPeer(cd));
  c.encode(frame[1]);c.encode(frame[2]);deliver(g,frame);CHECK(!g.isOffline() && !g.commandsUsable());
  activateC(c);pages(c,frame);deliver(g,frame);CHECK(g.commandsUsable());

  // CRC 前后所有位的单比特错误均被拒绝，且不能沿用旧有效指令。
  for(int bit=0;bit<64;++bit) {
    Comm receiver(Comm::CodePart::Gimbal,0x211,0x210);
    pages(c,frame);deliver(receiver,frame);CHECK(receiver.commandsUsable());
    uint8_t bad[8];memcpy(bad,frame[0],8);bad[bit/8]^=1<<(bit%8);receiver.decode(bad);
    CHECK(!receiver.commandsUsable() && !receiver.readPeer(cd));
  }
  // 丢页、乱序、混序号、超时拼包、错误方向、保留字段和非法整数编码均不提交。
  for(int scenario=0;scenario<7;++scenario) {
    Comm receiver(Comm::CodePart::Gimbal,0x211,0x210);pages(c,frame);
    if(scenario==0) {receiver.decode(frame[0]);receiver.decode(frame[2]);}
    if(scenario==1) {receiver.decode(frame[1]);receiver.decode(frame[0]);receiver.decode(frame[2]);}
    if(scenario==2) {frame[1][1]++;frame[1][7]=crc(frame[1]);deliver(receiver,frame);}
    if(scenario==3) {receiver.decode(frame[0]);test_now+=10;receiver.decode(frame[1]);receiver.decode(frame[2]);}
    if(scenario==4) {frame[0][0]=0xB0;frame[0][7]=crc(frame[0]);deliver(receiver,frame);}
    if(scenario==5) {frame[2][4]=1;frame[2][7]=crc(frame[2]);deliver(receiver,frame);}
    if(scenario==6) {frame[0][4]=0x80;frame[0][5]=0;frame[0][7]=crc(frame[0]);deliver(receiver,frame);}
    CHECK(receiver.isOffline() && !receiver.commandsUsable());
  }
  Comm invalid_state(Comm::CodePart::Gimbal,0x211,0x210);pages(c,frame);
  frame[0][2]|=3;frame[0][7]=crc(frame[0]);deliver(invalid_state,frame);CHECK(invalid_state.isOffline());
  // 非有限/越范围发布值被替换成故障快照，而不是截断后伪装成合法指令。
  c.chassis_data().target.yaw=std::numeric_limits<float>::quiet_NaN();pages(c,frame);
  CHECK(!c.lastEncodeValid());deliver(g,frame);CHECK(g.readPeer(cd) && cd.flags.fault && !g.commandsUsable());
  activateG(g);g.gimbal_data().motion.vx=40;pages(g,frame);CHECK(!g.lastEncodeValid());deliver(c,frame);
  CHECK(c.readPeer(gd) && gd.flags.fault && !c.commandsUsable());
  activateC(c);activateG(g);

  // 连续快照的 8 位序号和 32 位毫秒时间均可自然回绕。
  test_now=0xFFFFFE00;
  Comm wrapping(Comm::CodePart::Gimbal,0x211,0x210);
  for(int i=0;i<260;++i) {pages(c,frame);deliver(wrapping,frame);CHECK(wrapping.commandsUsable());}
  const uint32_t last=test_now-1;test_now=last+99;CHECK(!wrapping.isOffline());
  test_now=last+100;CHECK(wrapping.isOffline() && !wrapping.readPeer(cd) && cd.target.yaw==0);
  deliver(wrapping,frame);CHECK(wrapping.isOffline());  // 重复最后一包不得续命。

  // 发送板重启后旧序号先以安全状态重同步，再接收新序号指令。
  Comm restart_rx(Comm::CodePart::Gimbal,0x211,0x210),old_sender(Comm::CodePart::Chassis,0x210,0x211);
  activateC(old_sender);for(int i=0;i<5;++i) {pages(old_sender,frame);deliver(restart_rx,frame);}
  test_now+=100;Comm restarted(Comm::CodePart::Chassis,0x210,0x211);
  pages(restarted,frame);deliver(restart_rx,frame);CHECK(!restart_rx.isOffline() && !restart_rx.commandsUsable());
  activateC(restarted);pages(restarted,frame);deliver(restart_rx,frame);CHECK(restart_rx.commandsUsable());

  // 两板实际 service/getter/路由分别编译，协议帧不会被分发给电机。
  CHECK(GetGimbalChassisComm()!=nullptr && GetGimbalChassisComm()==GetGimbalChassisComm());
  CHECK(!SendBoardCommunication());filter_result=HAL_ERROR;CHECK(!StartBoardCommunication());filter_result=HAL_OK;
  start_result=HAL_ERROR;CHECK(!StartBoardCommunication());start_result=HAL_OK;
  CHECK(StartBoardCommunication());CHECK(captured_filter.FilterBank==expected_bank && captured_filter.SlaveStartFilterBank==14);
  CHECK(captured_filter.FilterFIFOAssignment==expected_fifo && captured_filter.FilterMaskIdLow==0);
  CHECK(filtered_can==expected_comm_can && started_can==expected_comm_can && notified_can==expected_comm_can);
  CHECK(last_notification==expected_notification);
  notify_result=HAL_ERROR;CHECK(!StartBoardCommunication());notify_result=HAL_OK;
  free_mailboxes=0;CHECK(!SendBoardCommunication());free_mailboxes=3;
  CHECK(SendBoardCommunication());CHECK(sent_header.DLC==8 && sent_header.IDE==CAN_ID_STD && sent_header.RTR==CAN_RTR_DATA);
  CHECK(sent_can==expected_comm_can);
#if defined(TEST_CHASSIS)
  CHECK(sent_header.StdId==0x210);Comm peer(Comm::CodePart::Gimbal,0x211,0x210);activateG(peer);
  Comm::GimbalData peer_data;
#else
  CHECK(sent_header.StdId==0x211);Comm peer(Comm::CodePart::Chassis,0x210,0x211);activateC(peer);
  Comm::ChassisData peer_data;
#endif
  pages(peer,frame);CHECK(!ReadPeerBoard(peer_data));
  // 先给错总线完整三页，不能建立板间在线状态，更不能被分发成电机反馈。
  for(int i=0;i<3;++i) {
    injected_header={GetGimbalChassisComm()->rxId(),CAN_ID_STD,CAN_RTR_DATA,8};
    memcpy(injected_data,frame[i],8);receiveComm(true);
  }
  CHECK(!ReadPeerBoard(peer_data));
  for(int i=0;i<3;++i) {
    injected_header={GetGimbalChassisComm()->rxId(),CAN_ID_STD,CAN_RTR_DATA,8};memcpy(injected_data,frame[i],8);
    receiveComm();
  }
  CHECK(ReadPeerBoard(peer_data) && ReadPeerCommand(peer_data) && PeerCommandsUsable());
  CHECK(notified_can==expected_comm_can && last_notification==expected_notification);
  CHECK(yaw.decodes==0 && pitch.decodes==0 && wheels[0].decodes==0 && steers[0].decodes==0);
  CHECK(notifications>=3);
  test_irq=1;CHECK(ReadPeerCommand(peer_data) && test_irq==1);test_irq=0;
  // 未 Working 的自检状态仍可交换，但控制读取必须失败并清空目标，避免互等及误动作。
#if defined(TEST_CHASSIS)
  peer.gimbal_data().state.life=robot::LifeState::Resurrection;
#else
  peer.chassis_data().state.life=robot::LifeState::Resurrection;
#endif
  pages(peer,frame);
  for(int i=0;i<3;++i) {
    memcpy(injected_data,frame[i],8);receiveComm();
  }
  CHECK(ReadPeerBoard(peer_data) && peer_data.flags.ready && !PeerCommandsUsable());
  CHECK(!ReadPeerCommand(peer_data) && peer_data.state.life==robot::LifeState::Dead && !peer_data.flags.ready);
  // 错总线、帧长、扩展/远程格式和未知 ID 都不刷新协议在线时间。
  test_now+=100;
  injected_header.DLC=7;receiveComm();CHECK(!PeerCommandsUsable());
  injected_header.DLC=8;injected_header.IDE=CAN_ID_EXT;receiveComm();CHECK(!ReadPeerBoard(peer_data));
  injected_header.IDE=CAN_ID_STD;injected_header.RTR=CAN_RTR_REMOTE;receiveComm();CHECK(!ReadPeerBoard(peer_data));
  injected_header.RTR=CAN_RTR_DATA;injected_header.StdId=0x333;receiveComm();CHECK(!ReadPeerBoard(peer_data));
  injected_header.StdId=GetGimbalChassisComm()->rxId();receiveComm(true);CHECK(!ReadPeerBoard(peer_data));
  send_result=HAL_ERROR;CHECK(!SendBoardCommunication());
  return 0;
}
