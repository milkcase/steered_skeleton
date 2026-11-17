#include "motor_debug_power.hpp"
#include "test_support.hpp"
#include <limits>
#include <initializer_list>
using namespace robot_debug;
using Comm=robot::GimbalChassisComm;
uint32_t test_ms=0;
uint32_t test_clock_offset=0;
// 提供双板纯软件测试时间，不连接真实电机。
uint32_t hello_world::tick::GetTickMs() {return test_ms-test_clock_offset;}
struct Bench {
  Power chassis{true,true},gimbal{true,false};
  Comm c{Comm::CodePart::Chassis,0x210,0x211},g{Comm::CodePart::Gimbal,0x211,0x210};
  Input ci{},gi{};Session cs{},gs{};
  float angle=0,velocity=0;bool radio=true,imu=true,feedback=true,other=true,deliver=true;
  bool chassis_imu=true;
  uint32_t chassis_tx_phase=0,gimbal_tx_phase=0;
  uint32_t chassis_clock_offset=0;
  // 以真实三页CAN协议、1ms控制和3ms分页交换挑战及左竖输入，不直接赋值授权状态。
  void tick(uint8_t right,float stick) {
    gi={};gi.now_ms=test_ms;gi.transport=true;gi.imu=imu;gi.feedback=true;gi.remote=radio;
    gi.right=right;gi.steer=std::fabs(stick)>0.05f?stick:0;gi.neutral=std::fabs(stick)<=0.05f && other;gi.other_neutral=other;
    ci=gi;ci.angle=angle;ci.velocity=velocity;ci.feedback=feedback;
    ci.now_ms=test_ms-chassis_clock_offset;
    ci.imu=imu && chassis_imu;
    ci.feedback_current_valid=feedback;ci.feedback_current_a=0.125f;
    Comm::GimbalData gd;Comm::ChassisData cd;
    test_clock_offset=chassis_clock_offset;
    ci.peer=c.readPeer(gd) && gd.flags.imu_ready && gd.flags.pitch_feedback_valid;
    ci.remote=ci.peer && gd.preview.valid;ci.right=gd.preview.right;ci.left=gd.preview.left;
    ci.steer=gd.preview.steer;ci.neutral=gd.preview.neutral;ci.other_neutral=gd.preview.other_neutral;gs=gd.session;
    if(drive::kVector) {ci.magnitude=gd.preview.magnitude;ci.vector_wheel_target=ci.magnitude*drive::kMaxTargetSpeed;}
    test_clock_offset=0;
    gi.peer=g.readPeer(cd) && cd.flags.imu_ready && cd.flags.yaw_feedback_valid;cs=cd.session;
    const auto cr=c.rxDiagnostics(),gr=g.rxDiagnostics();
    ci.link.rx_event=cr.last_rejection;gi.link.rx_event=gr.last_rejection;
    ci.link.crc_errors=cr.crc_errors;ci.link.format_errors=cr.format_errors;ci.link.order_errors=cr.order_errors;
    ci.link.assembly_timeouts=cr.assembly_timeouts;ci.link.replays=cr.replays;
    chassis.step(ci,gs);gimbal.step(gi,cs);
    auto &a=c.chassis_data();a={};a.state=chassis.output().state;a.session=chassis.session();
    a.flags.debug_preview=a.flags.debug_power=true;a.flags.ready=chassis.output().ready;a.flags.command_valid=chassis.output().command_valid;
    a.flags.imu_ready=ci.imu;a.flags.yaw_feedback_valid=true;a.flags.remote_valid=ci.remote;a.flags.emergency_released=ci.remote && ci.right!=2;
    auto &b=g.gimbal_data();b={};b.state=gimbal.output().state;b.session=gimbal.session();
    b.flags.debug_preview=b.flags.debug_power=true;b.flags.ready=gimbal.output().ready;b.flags.command_valid=gimbal.output().command_valid;
    b.flags.imu_ready=imu;b.flags.pitch_feedback_valid=true;b.flags.remote_valid=radio;b.flags.emergency_released=radio && right!=2;
    b.preview={gi.steer,right,1,gi.neutral,other,radio};
    if(drive::kVector)b.preview.magnitude=gi.neutral?0:std::fabs(stick);
    if(deliver && test_ms%3==chassis_tx_phase) {uint8_t data[8];c.encode(data);test_clock_offset=0;g.decode(data);}
    if(deliver && test_ms%3==gimbal_tx_phase) {uint8_t data[8];g.encode(data);test_clock_offset=chassis_clock_offset;c.decode(data);}
    test_clock_offset=0;
    ++test_ms;
  }
  // 真实拨杆序列先复位，再回中准备，最后等待右上授权；不允许直接上档起动。
  void prepare() {
    for(int i=0;i<80;++i)tick(2,0);
    for(int i=0;i<350;++i)tick(3,0);
    for(int i=0;i<70;++i)tick(1,0);
  }
};
// 验证驱动单速度环、本地整定编码、限流包络隔离、截止及各项失效归零。
#if !ROBOT_DRIVE_TUNING_ENABLED
extern "C" int run_tests() {
  bool profiles[4096]{};
  for(unsigned p=1;p<=16;++p)for(unsigned i=0;i<8;++i)for(unsigned d=0;d<4;++d)for(unsigned a=1;a<=8;++a) {
    const auto key=drive::profileFor(p,i,d,a);CHECK(key>=0x4000 && key<0x5000 && !profiles[key-0x4000]);profiles[key-0x4000]=true;
  }
  CHECK(drive::profileFor(0,0,0,1)==0 && drive::profileFor(1,8,0,1)==0);
  Bench b;b.prepare();float previous=0;bool moved=false;
  for(int i=0;i<350;++i) {
    b.tick(1,0.5f);const auto d=b.chassis.diagnostics();
    if(b.chassis.output().control) {
      moved=true;CHECK(d.position_loop_enabled==0 && d.drive_mode==1);
      CHECK(d.loops.speed_target_rad_s>=0 && d.loops.speed_target_rad_s<=0.25f+0.00001f);
      CHECK(d.loops.speed_target_rad_s-previous<=drive::kMaxAcceleration*0.00101f);
      CHECK(std::fabs(d.loops.current_command_a)<=drive::kCurrentA && b.gimbal.command()==0);
      previous=d.loops.speed_target_rad_s;
    }
  }
  CHECK(moved && b.chassis.command()>0);
  // 实际轮速高于目标时必须计算反向制动电流；不是常值或递增电流测试。
  b.velocity=0.8f;for(int i=0;i<3;++i)b.tick(1,0.5f);
  CHECK(b.chassis.command()<0 && b.chassis.diagnostics().loops.speed_error_rad_s<0);
  auto bad=b.gs;bad.target=33;CHECK(!b.chassis.stillSafe(b.ci,bad));
  for(unsigned bit:{1u,8u,32u,256u}) {bad=b.gs;bad.profile^=bit;CHECK(!b.chassis.stillSafe(b.ci,bad));}
  b.velocity=0.1f;for(int i=0;i<220;++i)b.tick(1,0.5f);
  CHECK(!b.chassis.output().control && b.chassis.command()==0 && b.chassis.diagnostics().last_stop==Reason::TimeLimit);
  CHECK(b.chassis.diagnostics().recorded_peak_speed_rad_s>=0.8f && b.chassis.diagnostics().recorded_feedback_samples>400);
  for(int i=0;i<100;++i)b.tick(1,0.5f);
  CHECK(b.chassis.diagnostics().run_count==1);
  // 反向速度目标与电流方向正确；两端零位附近小噪声按死区处理，不误起动。
  test_ms=0;Bench reverse;reverse.prepare();for(int i=0;i<30;++i)reverse.tick(1,0.03f);
  CHECK(reverse.chassis.diagnostics().run_count==0);
  for(int i=0;i<100;++i)reverse.tick(1,-0.5f);
  CHECK(reverse.chassis.command()<0 && reverse.chassis.diagnostics().loops.speed_target_rad_s<0);
  // 分别注入反馈/遥控/IMU/对端失效、其他通道、超速、位移、回中和急停。
  for(int fault=0;fault<10;++fault) {
    test_ms=0;Bench f;f.prepare();for(int i=0;i<100;++i)f.tick(1,0.5f);CHECK(f.chassis.output().control);
    if(fault==0)f.radio=false;if(fault==1)f.feedback=false;if(fault==2)f.imu=false;
    if(fault==3)f.deliver=false;if(fault==4)f.other=false;if(fault==5)f.velocity=1.01f;
    if(fault==6)f.angle=0.36f;if(fault==7)f.angle=6.283185307f;
    for(int i=0;i<120;++i)f.tick(fault==9?2:1,fault==8?0:0.5f);
    CHECK(f.chassis.command()==0 && !f.chassis.output().control);
    if(fault==5 || fault==6 || fault==7)CHECK(f.chassis.diagnostics().last_stop==Reason::MotionLimit);
  }
  return 0;
}
#endif
