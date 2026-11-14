#include "robot_can_service.hpp"
#include "motor_M3508.hpp"
#include "motor_GM6020.hpp"
#include "tim.h"
#include "main_task.hpp"
#if defined(TEST_CHASSIS)
#include "drive_controller.hpp"
#endif
#include "test_support.hpp"
using namespace robot_control;
using namespace robot;
using namespace hello_world::motor;
uint32_t test_now=0,free_mailboxes=3,aborts=0,frame_count=0;
CAN_HandleTypeDef hcan1{},hcan2{};
bool can1_listening=false,can2_listening=false;
int can_init_failure=0;
uint32_t can_abort_mask=0;
CAN_HandleTypeDef *motor_filter_can=nullptr;
CAN_FilterTypeDef motor_filter{};
uint32_t motor_notification=0;
TIM_HandleTypeDef htim6{},htim2{};
hello_world::remote_control::DT7 remote;
bool remote_valid=true;
GimbalChassisComm *local_comm=nullptr;
int init_calls=0,comm_calls=0,main_calls=0;
bool order_ok=true;
struct Frame {CAN_HandleTypeDef *can;uint32_t id;uint8_t data[8];};
Frame frames[4096];
// 提供可控时间源，所有生产有效期逻辑仍使用真实无符号差值。
uint32_t hello_world::tick::GetTickMs() {return test_now;}
// 提供 HAL 毫秒接口，真实 DBUS 临界区不调用外设。
extern "C" uint32_t HAL_GetTick() {return test_now;}
// 返回测试遥控快照，保持 DT7 的真实 decode 和 getter。
bool ReadRemoteControl(hello_world::remote_control::DT7 &out) {out=remote;return remote_valid;}
// 模拟所有 CAN 控制器处于已启动状态。
// 本轮进一步分别模拟两个控制器，不能以一条已启动总线掩盖另一条未启动。
HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *can) {
  return (can==&hcan1?can1_listening:can2_listening)?HAL_CAN_STATE_LISTENING:HAL_CAN_STATE_READY;
}
// 返回邮箱余量，用于制造发送拥塞。
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *) {return free_mailboxes;}
// 记录实际序列化帧，保留总线和 ID，不操作 CAN 适配器。
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can,const CAN_TxHeaderTypeDef *h,const uint8_t *p,uint32_t *) {
  if(frame_count<4096) {auto &f=frames[frame_count++];f.can=can;f.id=h->StdId;memcpy(f.data,p,8);}
  return HAL_OK;
}
// 记录撤销旧邮箱动作，模拟硬件取消待发命令。
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can,uint32_t) {can_abort_mask|=can==&hcan1?1:2;++aborts;return HAL_OK;}
// 本测试不检查滤波器，实际配置已经第八项及生产编译覆盖。
// 拓扑适配后补查电机侧滤波器和配置失败；原来的不检查说明仅为早期测试范围。
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can,const CAN_FilterTypeDef *filter) {
  motor_filter_can=can;motor_filter=*filter;return can_init_failure==1?HAL_ERROR:HAL_OK;
}
// 模拟启动成功。
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can) {
  if(can_init_failure==2)return HAL_ERROR;
  if(can==&hcan1)can1_listening=true;else can2_listening=true;return HAL_OK;
}
// 模拟通知使能成功。
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *,uint32_t notification) {
  motor_notification=notification;return can_init_failure==3?HAL_ERROR:HAL_OK;
}
// 单页接口使用真实线上编码，帮助检查零状态/Working 发布是否正确。
bool SendBoardCommunication() {
  if(free_mailboxes==0)return false;
  uint8_t data[8];local_comm->encode(data);CAN_TxHeaderTypeDef h{};uint32_t box;
  h.StdId=local_comm->txId();h.DLC=8;
#if defined(TEST_CHASSIS)
  return HAL_CAN_AddTxMessage(&hcan2,&h,data,&box)==HAL_OK;
#else
  return HAL_CAN_AddTxMessage(&hcan1,&h,data,&box)==HAL_OK;
#endif
}
// 模拟板间接口启动，不涉及物理总线。
bool StartBoardCommunication() {
  if(can_init_failure==4)return false;
#if defined(TEST_CHASSIS)
  can2_listening=true;
#else
  can1_listening=true;
#endif
  return true;
}
// 记录真实任务薄层调用初始化的次数。
void InitializeRobotRuntime() {++init_calls;}
// 记录真实任务薄层将控制交给 Robot 的次数。
void RunRobotControl() {++main_calls;}
// 记录真实 TIM6 回调将通信放在控制之后。
extern "C" void CommTask() {order_ok=order_ok && main_calls==comm_calls;++comm_calls;}
// 记录 MainTaskInit 的通信初始化接线。
extern "C" void CommTaskInit() {++comm_calls;}
// 声明真实定时器回调，测试只注入句柄，不操作定时器寄存器。
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);

// 构造明确的拨杆与五通道 DBUS 字节，再交给提供的 DT7 解码器。
void setRemote(uint8_t right,uint16_t pitch=1024) {
  uint16_t c[5]={1024,pitch,1024,1024,1024};uint8_t d[18]{};
  d[0]=c[0];d[1]=(c[0]>>8)|(c[1]<<3);d[2]=(c[1]>>5)|(c[2]<<6);
  d[3]=c[2]>>2;d[4]=(c[2]>>10)|(c[3]<<1);d[5]=(c[3]>>7)|(right<<4)|(1<<6);
  d[16]=c[4];d[17]=c[4]>>8;remote.decode(d);
}
// 正参数仅用于测试，不改写未确认的实车配置。
Params params() {
  Params p;p.remote_mapping_confirmed=true;p.max_vx_m_s=p.max_vy_m_s=1;p.max_w_rad_s=1;
  p.yaw_rate_rad_s=p.pitch_rate_rad_s=0.5f;p.follow.kp=1;p.follow.output_limit=1;return p;
}
// 发布完整安全/运行快照，不直接共享两个板的运行变量。
void peerFrame(GimbalChassisComm &peer,GimbalChassisComm &local,LifeState life,bool released=true) {
  GimbalChassisComm::Flags flags{true,true,true,true,true,released,true,false};
  ModuleState state{life,ControlSource::RemoteControl,ChassisMode::Separate,GimbalMode::JointPosition};
#if defined(TEST_CHASSIS)
  auto &d=peer.gimbal_data();d.state=state;d.flags=flags;d.motion={0.2f,0,0};d.yaw_target=0.2f;
#else
  // 模拟底盘稍晚收到云台回中许可；云台的零输入使能不能等待底盘先发布控制许可。
  flags.command_valid=test_now>=202;
  auto &d=peer.chassis_data();d.state=state;d.flags=flags;d.feedback.yaw=0.2f;d.chassis_yaw=0.1f;
#endif
  for(int i=0;i<3;++i) {uint8_t data[8];peer.encode(data);local.decode(data);}
}
// 注入真实达妙状态反馈，记录本地接收时间；不把使能发送等同于已经使能。
void axisFeedback(DaMiao &axis) {
  uint8_t f[8]={static_cast<uint8_t>(0x10|axis.motor_info().id),0x80,0,0x80,0,0,30,30};
  axis.decode(f);MotorFeedbacks().recordAxis(axis.motor_info().id,f,test_now);
}
// 验证真实 Runtime、PID、驱动序列化、任务分发及失效后的零帧/失能帧。
extern "C" int run_tests() {
  MainTaskInit();CHECK(init_calls==1 && comm_calls==1);
  HAL_TIM_PeriodElapsedCallback(&htim2);CHECK(main_calls==0);
  HAL_TIM_PeriodElapsedCallback(&htim6);CHECK(main_calls==1 && comm_calls==2 && order_ok);
  OptionalParams opt;opt.input_type=InputType::kTorq;opt.angle_range=AngleRange::kNegPiToPosPi;
  // 使用与生产工厂相同的默认构造加 init 路径，关闭自动使能和自动模式切换。
  DM_J4310 axis;axis.init(1,opt,false,false);
  hello_world::pid::Pid::Params ap{.kp=2,.output_limit=1};
  Gimbal::Params gp;gp.max_torque_nm=1;gp.max_target_rate_rad_s=3;
#if defined(TEST_CHASSIS)
  ap.error_period=6.283185307179586f;
  Gimbal axis_loop(Gimbal::Axis::Yaw,&axis,nullptr,gp);
  RobotRuntime robot(Board::Chassis,params());
  GimbalChassisComm local(GimbalChassisComm::CodePart::Chassis,0x210,0x211),peer(GimbalChassisComm::CodePart::Gimbal,0x211,0x210);
#else
  gp.pitch_limits_confirmed=true;gp.pitch_min_rad=-0.5f;gp.pitch_max_rad=0.5f;
  Gimbal axis_loop(Gimbal::Axis::Pitch,&axis,nullptr,gp);
  RobotRuntime robot(Board::Gimbal,params());
  GimbalChassisComm local(GimbalChassisComm::CodePart::Gimbal,0x211,0x210),peer(GimbalChassisComm::CodePart::Chassis,0x210,0x211);
#endif
  hello_world::pid::Pid pid(ap);
  Gimbal loop(axis_loop.axis(),&axis,&pid,gp);
  robot.registerAxis(&axis,&loop,&pid);robot.registerCommunication(&local);local_comm=&local;
  hello_world::imu::Imu imu;robot.registerImu(&imu);
  hello_world::imu::BMI088HWConfig hw;const float rotation[9]={1,0,0,0,1,0,0,0,1};
  CHECK(!robot.initializeImu(hw,rotation,false));CHECK(robot.initializeImu(hw,rotation,true));
#if defined(TEST_CHASSIS)
  M3508 wheels[4]={M3508(1),M3508(2),M3508(3),M3508(4)};
  GM6020 steers[4]={GM6020(1),GM6020(2),GM6020(3),GM6020(4)};
  hello_world::chassis_ik_solver::ChassisIkSolver::Limits limits;
  limits.max_wheel_speed_rad_s=10;limits.max_steer_rate_rad_s=5;
  Chassis::Solver solver(limits);Chassis chassis(&solver);
  Motor *wp[4],*sp[4];hello_world::pid::Pid wpid[4],spid[4];hello_world::pid::Pid *wpp[4],*spp[4];
  for(int i=0;i<4;++i) {
    wp[i]=&wheels[i];sp[i]=&steers[i];wpp[i]=&wpid[i];spp[i]=&spid[i];
    wpid[i].configure({.kp=10,.output_limit=100});
    spid[i].configure({.kp=10,.output_limit=100,.error_period=6.283185307179586f});
  }
  DriveController drive(&chassis,wp,sp,wpp,spp,&solver);robot.registerDrive(&drive);
  RobotCanService service(&robot,true);service.registerDrive(wp,sp);
#else
  RobotCanService service(&robot,false);
#endif
  // 任一电机接口初始化阶段失败，或板间接口失败，都不能把通信启动报告为成功。
  for(int failure=1;failure<=4;++failure) {
    can1_listening=can2_listening=false;can_init_failure=failure;
    CHECK(!service.init() && !robot.output().control);
  }
  can_init_failure=0;can1_listening=can2_listening=false;
  CHECK(service.init() && can1_listening && can2_listening);
#if defined(TEST_CHASSIS)
  CHECK(motor_filter_can==&hcan1 && motor_filter.FilterBank==0 && motor_filter.FilterFIFOAssignment==CAN_FILTER_FIFO0);
  CHECK(motor_notification==CAN_IT_RX_FIFO0_MSG_PENDING);
#else
  CHECK(motor_filter_can==&hcan2 && motor_filter.FilterBank==14 && motor_filter.FilterFIFOAssignment==CAN_FILTER_FIFO1);
  CHECK(motor_notification==CAN_IT_RX_FIFO1_MSG_PENDING);
#endif
  robot.setTransportReady(true);setRemote(3);
  for(int step=0;step<240;++step) {
    test_now=step;robot.pollSensors();
    const bool work=step>=210;
    if(step==210)setRemote(1,1156);
    if(step>=205)axisFeedback(axis);
#if defined(TEST_CHASSIS)
    uint8_t raw[8]{};raw[6]=30;
    for(int i=0;i<4;++i) {wheels[i].decode(raw);steers[i].decode(raw);MotorFeedbacks().recordDji(i,raw,test_now);MotorFeedbacks().recordDji(i+4,raw,test_now);}
#endif
    peerFrame(peer,local,work?LifeState::Working:LifeState::Resurrection);
    robot.update();service.tick();
    if(step<205)CHECK(axis.getInput()==0);
    if(step==199)CHECK(robot.output().ready);
    if(step==200)CHECK(robot.output().axis_action==AxisAction::ArmZero);
    if(step==211)CHECK(robot.output().control);
  }
  CHECK(robot.output().control);
  CHECK(std::fabs(axis.getInput())>0.00001f);
  bool enabled=false,wheel_nonzero=false,steer_seen=false;
  for(uint32_t i=0;i<frame_count;++i) {
    const auto &f=frames[i];
    if(f.id==1)CHECK(f.can==&hcan2); // 本地关节始终 CAN2，云台板间接口改动不能带走 pitch。
#if defined(TEST_CHASSIS)
    if(f.id==0x210)CHECK(f.can==&hcan2);
#else
    if(f.id==0x211)CHECK(f.can==&hcan1);
#endif
    if(f.id==1 && f.data[0]==255 && f.data[7]==0xFC)enabled=true;
    if(f.id==1 && f.data[0]!=255)CHECK((f.data[3]&15)==0 && f.data[4]==0 && f.data[5]==0 && (f.data[6]&0xF0)==0);
    if(f.id==GM6020::GetTxId(1,InputType::kRaw) && f.can==&hcan1)steer_seen=true;
    if(f.id==M3508::GetTxId(1) && f.can==&hcan2 && f.data[1]!=0 && f.data[3]!=0 && f.data[5]!=0 && f.data[7]!=0)wheel_nonzero=true;
  }
  CHECK(enabled);
#if defined(TEST_CHASSIS)
  CHECK(wheel_nonzero && steer_seen);
#else
  CHECK(!wheel_nonzero && !steer_seen);
#endif
  const uint32_t before=frame_count,old_abort=aborts;
  can_abort_mask=0;
  test_now++;setRemote(2);peerFrame(peer,local,LifeState::Dead,false);robot.update();service.tick();
  CHECK(axis.getInput()==0 && !robot.output().control && aborts>old_abort);
  CHECK(can_abort_mask==3); // 急停同时取消本地两条总线的旧邮箱。
  bool disabled=false;
  for(uint32_t i=before;i<frame_count;++i) {
    const auto &f=frames[i];if(f.id==1 && f.data[7]==0xFD)disabled=true;
    if(f.id==M3508::GetTxId(1) || f.id==GM6020::GetTxId(1,InputType::kRaw))for(uint8_t byte:f.data)CHECK(byte==0);
  }
  CHECK(disabled);
#if defined(TEST_CHASSIS)
  for(int i=0;i<4;++i)CHECK(wheels[i].getInput()==0 && steers[i].getInput()==0);
#endif
  // 再次明确回中恢复，然后制造 CAN 邮箱持续忙；即使注入的反馈仍新鲜也必须归零。
  // 推进一个测试周期，可独立停掉前台 IMU 或对端报文，不停掉真实控制/通信代码。
  auto advance=[&](bool working,bool sample=true,bool receive_peer=true) {
    ++test_now;if(sample)robot.pollSensors();axisFeedback(axis);
#if defined(TEST_CHASSIS)
    uint8_t raw[8]{};raw[6]=30;
    for(int k=0;k<4;++k) {wheels[k].decode(raw);steers[k].decode(raw);MotorFeedbacks().recordDji(k,raw,test_now);MotorFeedbacks().recordDji(k+4,raw,test_now);}
#endif
    if(receive_peer)peerFrame(peer,local,working?LifeState::Working:LifeState::Resurrection);
    robot.update();service.tick();
  };
  setRemote(3);
  for(int i=0;i<240;++i) {
    if(i==210)setRemote(1,1156);
    advance(i>=210);
  }
  CHECK(robot.output().control);
  free_mailboxes=0;
  for(int i=0;i<25;++i)advance(true);
  CHECK(!robot.output().control && axis.getInput()==0);
  free_mailboxes=3;robot.setTransportReady(true);setRemote(3);
  for(int i=0;i<240;++i) {if(i==210)setRemote(1,1156);advance(i>=210);}
  CHECK(robot.output().control);
  for(int i=0;i<5;++i)advance(true,false);
  CHECK(!robot.output().control && axis.getInput()==0); // 前台停采 5ms，控制中断仍在运行也不能继续。
  setRemote(3);
  for(int i=0;i<240;++i) {if(i==210)setRemote(1,1156);advance(i>=210);}
  CHECK(robot.output().control);
  for(int i=0;i<100;++i)advance(true,true,false);
  CHECK(local.isOffline() && !robot.output().control && axis.getInput()==0);
  return 0;
}
