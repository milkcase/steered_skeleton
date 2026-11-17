#include "motor_debug_power.hpp"
#include "test_support.hpp"
#include <limits>
using namespace robot_debug;
using Comm=robot::GimbalChassisComm;
uint32_t test_ms=0;
// 模拟两板毫秒时钟；全部试验在ARM模拟器运行，不连接电机。
uint32_t hello_world::tick::GetTickMs() {return test_ms;}
// 比较方向、目标速度与电流计算的正常浮点误差。
bool near(float a,float b) {return std::fabs(a-b)<0.0001f;}
struct Bench {
  Power c{true,true},g{true,false};
  Comm cc{Comm::CodePart::Chassis,0x210,0x211},gc{Comm::CodePart::Gimbal,0x211,0x210};
  Input ci{},gi{};Session cs{},gs{};
  float velocity=0,position=0,yaw=0;bool radio=true,imu=true,feedback=true,deliver=true,other=true;
  // 用真实3页协议传方向/回中位/双环参数编号；向前角0和回中角0必须能可靠区分。
  void tick(uint8_t right,float forward,float horizontal) {
    const auto stick=direction::solve(forward,horizontal);
    gi={};gi.now_ms=test_ms;gi.transport=true;gi.imu=imu;gi.feedback=true;gi.remote=radio && stick.valid;
    gi.right=right;gi.steer=stick.angle/direction::kPi;gi.neutral=!stick.active && other;gi.other_neutral=other;
    ci=gi;ci.angle=position;ci.velocity=velocity;ci.yaw_delta=yaw;ci.feedback=feedback;
    Comm::GimbalData gd;Comm::ChassisData cd;
    ci.peer=cc.readPeer(gd) && gd.flags.imu_ready && gd.flags.pitch_feedback_valid;
    ci.remote=ci.peer && gd.preview.valid;ci.right=gd.preview.right;ci.left=gd.preview.left;
    ci.steer=gd.preview.steer;ci.neutral=gd.preview.neutral;ci.other_neutral=gd.preview.other_neutral;gs=gd.session;
    const auto rx=cc.rxDiagnostics();ci.link.rx_event=rx.last_rejection;
    ci.link.crc_errors=rx.crc_errors;ci.link.format_errors=rx.format_errors;ci.link.order_errors=rx.order_errors;
    ci.link.assembly_timeouts=rx.assembly_timeouts;ci.link.replays=rx.replays;
    gi.peer=gc.readPeer(cd) && cd.flags.imu_ready && cd.flags.yaw_feedback_valid;cs=cd.session;
    c.step(ci,gs);g.step(gi,cs);
    auto &a=cc.chassis_data();a={};a.state=c.output().state;a.session=c.session();
    a.flags.debug_preview=a.flags.debug_power=true;a.flags.ready=c.output().ready;a.flags.command_valid=c.output().command_valid;
    a.flags.imu_ready=imu;a.flags.yaw_feedback_valid=true;a.flags.remote_valid=ci.remote;a.flags.emergency_released=ci.remote && ci.right!=2;
    auto &b=gc.gimbal_data();b={};b.state=g.output().state;b.session=g.session();
    b.flags.debug_preview=b.flags.debug_power=true;b.flags.ready=g.output().ready;b.flags.command_valid=g.output().command_valid;
    b.flags.imu_ready=imu;b.flags.pitch_feedback_valid=true;b.flags.remote_valid=gi.remote;b.flags.emergency_released=gi.remote && right!=2;
    b.preview={gi.steer,right,1,gi.neutral,other,gi.remote};
    if(deliver && test_ms%3==0) {uint8_t frame[8];cc.encode(frame);gc.decode(frame);gc.encode(frame);cc.decode(frame);}
    ++test_ms;
  }
  // 执行真实右下、右中回中200ms、右上等待，不能写内部准备标志。
  void prepare() {
    for(int i=0;i<80;++i)tick(2,0,0);
    for(int i=0;i<350;++i)tick(3,0,0);
    for(int i=0;i<70;++i)tick(1,0,0);
  }
};
// 检查二维解算、内外环串联、无500ms截止、回中/联锁及新旧协议隔离。
extern "C" int run_tests() {
  // 位图遍历95000种包络的target/profile对，不让测试栈随包络数量线性占用字节。
  uint8_t safety_profiles[(steer_safety::kSafetyProfiles+7)/8]{};
  for(uint32_t speed=0;speed<steer_safety::kSpeedCombinations;++speed)for(uint32_t current=1;current<=steer_safety::kCurrentSlots;++current) {
    const auto key=steer_safety::safetyProfile(speed,current);
    const auto tag=steer_safety::safetyTarget(speed,current);
    CHECK(key>=steer_safety::kProfileBase && key<steer_safety::kProfileBase+steer_safety::kProfileSpan && tag>=steer_safety::kTargetBase);
    const uint32_t code=(tag-steer_safety::kTargetBase)*steer_safety::kProfileSpan+key-steer_safety::kProfileBase;
    CHECK(code==speed*steer_safety::kCurrentSlots+current-1 && code<steer_safety::kSafetyProfiles);
    CHECK((safety_profiles[code/8]&(1u<<(code%8)))==0);safety_profiles[code/8]|=1u<<(code%8);
  }
  CHECK(steer_safety::safetyProfile(steer_safety::kSpeedCombinations,1)==0 && steer_safety::safetyProfile(0,0)==0);
  CHECK(direction::solve(0,0).valid && !direction::solve(0,0).active);
  CHECK(direction::solve(0.5f,0).active && near(direction::solve(0.5f,0).angle,0));
  CHECK(near(direction::solve(0,-1).angle,direction::kPi/2));
  CHECK(near(direction::solve(0,1).angle,-direction::kPi/2));
  CHECK(!direction::solve(std::numeric_limits<float>::quiet_NaN(),0).valid);
  Bench b;b.prepare();
  // 向前虽方向0，仍须启动并持续超过500ms；当前位置偏左时输出右转闭环电流。
  b.position=0.02f;
  for(int i=0;i<1300;++i)b.tick(1,0.5f,0);
  CHECK(b.c.output().control && b.c.command()<0 && b.g.command()==0);
  CHECK(b.c.diagnostics().run_count==1 && b.c.diagnostics().stop_count==0 && b.c.diagnostics().time_limit_enabled==0);
  CHECK(b.c.diagnostics().direction_mode==1 && near(b.c.diagnostics().requested_angle,0));
  const auto d=b.c.diagnostics();
  CHECK(d.loops.active && near(d.loops.position_target_rad,0) && near(d.loops.position_feedback_rad,b.position));
  CHECK(near(d.loops.speed_target_rad_s,-direction::kPositionKp*b.position));
  // 用真实非零反馈速度验证内环误差；不能只把位置输出换个名字。
  b.velocity=0.3f;b.tick(1,0.5f,0);
  const auto running=b.c.diagnostics();
  CHECK(near(running.loops.speed_feedback_rad_s,0.3f));
  CHECK(near(running.loops.speed_error_rad_s,running.loops.speed_target_rad_s-0.3f));
  CHECK(std::fabs(running.loops.current_command_a)<=direction::kCurrentA);
  auto mismatch=b.gs;mismatch.profile^=1;CHECK(!b.c.stillSafe(b.ci,mismatch));
  mismatch=b.gs;mismatch.target=1;CHECK(!b.c.stillSafe(b.ci,mismatch));
  // 新方向穿过0不等于回中；真正回中归零，再偏杆可继续，且不会输出非Working动力。
  b.velocity=0;
  for(int i=0;i<30;++i)b.tick(1,0.5f,-0.01f);
  for(int i=0;i<30;++i)b.tick(1,0.5f,0.01f);
  CHECK(b.c.output().control);
  for(int i=0;i<20;++i)b.tick(1,0,0);
  CHECK(!b.c.output().control && b.c.command()==0);
  CHECK(b.c.diagnostics().last_stop==Reason::Centered);
  CHECK(b.c.diagnostics().recorded_loops.active==1 && b.c.diagnostics().loops.active==0);
  for(int i=0;i<30;++i)b.tick(1,0.5f,0);
  CHECK(b.c.diagnostics().run_count==2 && b.c.output().control);
  for(int repeat=0;repeat<3;++repeat) {
    for(int i=0;i<30;++i)b.tick(1,0,0);
    CHECK(!b.c.output().control && b.c.command()==0 && b.c.diagnostics().last_stop==Reason::Centered);
    for(int i=0;i<30;++i)b.tick(1,0.5f,0);
    CHECK(b.c.output().control);
  }
  // 回中等待期间遇到坏CRC仍必须故障锁止；链路恢复和再次偏杆不能自动解除。
  for(int i=0;i<30;++i)b.tick(1,0,0);
  uint8_t corrupt[8]{};corrupt[7]=1;b.cc.decode(corrupt);b.tick(1,0,0);
  for(int i=0;i<50;++i)b.tick(1,0.5f,0);
  CHECK(!b.c.output().control && b.c.command()==0);
  // 每项保护分别模拟，不能用“还会500ms截止”掩盖保护缺失。
  for(int fault=0;fault<7;++fault) {
    test_ms=0;Bench f;f.prepare();
    for(int i=0;i<600;++i)f.tick(1,0.5f,-0.03f);
    CHECK(f.c.output().control);
    if(fault==0)f.radio=false;if(fault==1)f.imu=false;if(fault==2)f.feedback=false;
    if(fault==3)f.other=false;if(fault==4)f.deliver=false;
    if(fault==5)f.velocity=steer_safety::kActualSpeedLimit+0.1f;
    for(int i=0;i<120;++i)f.tick(fault==6?2:1,0.5f,-0.03f);
    CHECK(!f.c.output().control && f.c.command()==0 && f.g.command()==0);
    if(fault==5) {
      const auto saved=f.c.diagnostics().stop_check;
      CHECK(f.c.diagnostics().last_stop==Reason::MotionLimit && saved.mask==CheckMotion && saved.source==1);
      CHECK(near(saved.actual_velocity,steer_safety::kActualSpeedLimit+0.1f) && near(saved.speed_limit,steer_safety::kActualSpeedLimit));
    }
    if(fault==6)CHECK(f.c.diagnostics().last_stop==Reason::Emergency);
  }
  // yaw坐标变换和>90°等效解仍生效；本轮只计算转向，不允许驱动轮滚动。
  test_ms=0;Bench rotated;rotated.prepare();rotated.yaw=direction::kPi;
  for(int i=0;i<100;++i)rotated.tick(1,0.5f,0);
  CHECK(rotated.c.output().control && near(rotated.c.diagnostics().equivalent_angle,0));
  CHECK(rotated.c.diagnostics().equivalent_drive_sign==-1);
  // 左、右、后方摇杆方向均由二维atan2给出；后方允许等效前向+驱动反转，不能丢失符号。
  test_ms=0;Bench backward;backward.prepare();
  for(int i=0;i<100;++i)backward.tick(1,-0.5f,0);
  CHECK(near(std::fabs(backward.c.diagnostics().requested_angle),direction::kPi));
  CHECK(near(backward.c.diagnostics().equivalent_angle,0) && backward.c.diagnostics().equivalent_drive_sign==-1);
  // 接收撤销提示只改变停止名称：坏CRC不能伪造“正常回中”，也不能因此恢复许可。
  auto hint=b.ci;hint.peer=false;hint.now_ms=100;
  hint.link.rx_event={};auto &event=hint.link.rx_event;
  event.reason=Comm::RxFailure::SafetyPage;event.time_ms=100;event.flags_byte=123;
  event.control_signal_valid=1;event.right=1;event.neutral=1;
  CHECK(b.c.failureReason(hint,CheckPeer|CheckSession)==Reason::Centered);
  event.reason=Comm::RxFailure::Crc;
  CHECK(b.c.failureReason(hint,CheckPeer|CheckSession)==Reason::PeerConsent);
  CHECK(b.c.failureReason(hint,CheckMotion)==Reason::MotionLimit);
  // 实际和目标均可超过旧的5°起点限制；这里只运行纯计算，不使电机长时间空转。
  test_ms=0;Bench free_angle;free_angle.prepare();
  for(int i=0;i<1600;++i)free_angle.tick(1,0.5f,-0.2f);
  free_angle.position=0.1f;free_angle.tick(1,0.5f,-0.2f);
  CHECK(free_angle.c.output().control && free_angle.c.diagnostics().target_angle>0.08727f);
  CHECK(free_angle.c.diagnostics().range_limit_enabled==0);
  return 0;
}
