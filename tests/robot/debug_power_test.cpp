#include "motor_debug_power.hpp"
#include "test_support.hpp"
using namespace robot_debug;
using Comm=robot::GimbalChassisComm;
uint32_t test_ms=0;
// 模拟两板各自的毫秒接口，测试只在ARM模拟器运行。
uint32_t hello_world::tick::GetTickMs() {return test_ms;}
// 明确的首轮测试参数，与生产独立动力配置相同，不写入普通固件。
PowerConfig config() {return {tuning::kPositionKp,tuning::kCurrentLimitA,0.0872664626f,0.5f};}
// 提供本地已验证条件；对端条件和会话仍必须通过真实协议收取。
Input local(uint8_t right,float stick) {
  Input in;in.now_ms=test_ms;in.transport=in.imu=in.feedback=in.remote=true;
  in.channels_valid=true;in.channels[1]=stick;
  in.right=right;in.steer=stick;in.neutral=std::fabs(stick)<=0.05f;in.other_neutral=true;return in;
}
// 云台撤销原因单元测试显式提供已收齐的对端快照；双板分页的完整验证仍由Bench承担。
Input cloudInput(uint8_t right,float stick) {
  auto in=local(right,stick);in.peer=true;return in;
}
struct Bench {
  Power chassis{true,true,config()},gimbal{true,false,config()};
  Comm c{Comm::CodePart::Chassis,0x210,0x211},g{Comm::CodePart::Gimbal,0x211,0x210};
  Input ci{},gi{};Session cs{},gs{};
  float position=0,speed=0;bool remote=true,imu=true,feedback=true,deliver=true;
  // 以1ms控制、3ms一页更新真实双板协议，允许注入关遥控/断反馈和运动超限。
  void tick(uint8_t right,float stick) {
    ci=local(right,stick);gi=local(right,stick);ci.angle=position;ci.velocity=speed;
    ci.remote=gi.remote=remote;ci.imu=gi.imu=imu;ci.feedback=feedback;
    ci.feedback_current_valid=feedback;ci.feedback_current_a=0.012f;
    Comm::GimbalData gd;Comm::ChassisData cd;
    ci.peer=c.readPeer(gd) && gd.flags.debug_power && gd.flags.imu_ready && gd.flags.pitch_feedback_valid;
    ci.remote=ci.peer && gd.preview.valid;ci.right=gd.preview.right;ci.left=gd.preview.left;
    ci.steer=gd.preview.steer;ci.neutral=gd.preview.neutral;ci.other_neutral=gd.preview.other_neutral;gs=gd.session;
    const bool cloud_received=g.readPeer(cd);
    gi.peer=cloud_received && cd.flags.debug_power && cd.flags.imu_ready && cd.flags.yaw_feedback_valid;cs=cd.session;
    gi.link.snapshot_valid=cloud_received;gi.link.age_ms=g.peerAgeMs();gi.link.rx_event=g.rxDiagnostics().last_rejection;
    chassis.step(ci,gs);gimbal.step(gi,cs);
    auto &outc=c.chassis_data();outc={};outc.state=chassis.output().state;
    outc.flags.ready=chassis.output().ready;outc.flags.command_valid=chassis.output().command_valid;
    outc.flags.debug_preview=outc.flags.debug_power=true;outc.flags.imu_ready=imu;outc.flags.yaw_feedback_valid=true;
    outc.flags.remote_valid=ci.remote;outc.flags.emergency_released=ci.remote && ci.right!=2;outc.session=chassis.session();
    auto &outg=g.gimbal_data();outg={};outg.state=gimbal.output().state;
    outg.flags.ready=gimbal.output().ready;outg.flags.command_valid=gimbal.output().command_valid;
    outg.flags.debug_preview=outg.flags.debug_power=true;outg.flags.imu_ready=imu;outg.flags.pitch_feedback_valid=true;
    outg.flags.remote_valid=remote;outg.flags.emergency_released=remote && right!=2;
    outg.preview={stick,right,1,std::fabs(stick)<=0.05f,true,remote};outg.session=gimbal.session();
    if(deliver && test_ms%3==0) {
      uint8_t data[8];c.encode(data);g.decode(data);g.encode(data);c.decode(data);
    }
    ++test_ms;
  }
};
// 验证双板挑战回显、Working红线、单次截止、停止后不续期与私有控制上限。
extern "C" int run_tests() {
  Power disabled(false,true);CHECK(!disabled.configured());
  Power bad(true,true,PowerConfig{});CHECK(!bad.configured());
  auto mismatched=config();mismatched.kp+=0.5f;Power wrong_config(true,true,mismatched);CHECK(!wrong_config.configured());
  // 128种可调组合必须编号唯一、装入现有一个字节；非法档位不生成有效编号。
  bool seen_profiles[256]{};
  for(uint32_t p=1;p<=16;++p)for(uint32_t c=1;c<=8;++c) {
    const auto profile=tuning::profileFor(p,c);CHECK(profile>=128 && profile<=255 && !seen_profiles[profile]);seen_profiles[profile]=true;
  }
  CHECK(tuning::profileFor(0,1)==0 && tuning::profileFor(1,9)==0);
  CHECK(tuning::stepIndex(0.6f,0.5f,16)==0 && tuning::stepIndex(0.3f,0.025f,8)==0);
  Bench b;bool moving=false;float peak=0;
  for(int ms=0;ms<1100;++ms) {
    const uint8_t right=ms<60?2:(ms<360?3:1);const float stick=ms>=400?-0.5f:0;
    b.tick(right,stick);
    CHECK(std::fabs(b.chassis.command())<=tuning::kCurrentLimitA && b.gimbal.command()==0);
    if(b.chassis.command()!=0) {
      CHECK(b.chassis.output().state.life==robot::LifeState::Working && b.chassis.output().control);
      CHECK(b.chassis.session().token==b.gs.token && b.gs.phase==2 && b.gs.target==1 && b.gs.profile==Comm::kPowerProfile);
      moving=true;if(b.chassis.command()>peak)peak=b.chassis.command();
    }
  }
  CHECK(moving && peak>0 && peak<=tuning::kCurrentLimitA && b.chassis.command()==0);
  // 低P/高限流组合本来就可能到不了上限；只在已知本脉冲可饱和的范围检查确实达到限流。
  if(tuning::kPositionKp>=5.0f && tuning::kCurrentLimitA<=0.100f)
    CHECK(std::fabs(peak-tuning::kCurrentLimitA)<0.000001f);
  CHECK(std::fabs(b.chassis.diagnostics().recorded_peak_feedback_current_a-0.012f)<0.000001f);
  CHECK(b.chassis.diagnostics().recorded_feedback_samples>400);
  CHECK(b.chassis.diagnostics().run_count==1 && b.chassis.diagnostics().stop_count==1);
  CHECK(b.chassis.diagnostics().last_stop==Reason::TimeLimit);
  const auto token=b.chassis.session().token;
  for(int i=0;i<200;++i)b.tick(1,-0.5f);
  CHECK(b.chassis.diagnostics().run_count==1 && b.chassis.command()==0);
  // 新一轮必须右下产生新挑战，再回中准备；相同的旧echo不得得到动力。
  for(int i=0;i<70;++i)b.tick(2,0);
  CHECK(b.chassis.session().token!=token);
  for(int i=0;i<320;++i)b.tick(3,0);
  for(int i=0;i<100;++i)b.tick(1,-0.5f);
  CHECK(b.chassis.command()>0 && b.chassis.stillSafe(b.ci,b.gs));
  auto invalid=b.gs;invalid.token=token;CHECK(!b.chassis.stillSafe(b.ci,invalid));
  invalid=b.gs;invalid.target=2;CHECK(!b.chassis.stillSafe(b.ci,invalid));
  invalid=b.gs;invalid.profile=1;CHECK(!b.chassis.stillSafe(b.ci,invalid)); // 旧电压配置不得用于本次电流输出。
  invalid=b.gs;invalid.profile=2;CHECK(!b.chassis.stillSafe(b.ci,invalid)); // 旧P=0.5电流版也不能授权新P参数。
  invalid=b.gs;invalid.profile=3;CHECK(!b.chassis.stillSafe(b.ci,invalid));
  invalid=b.gs;invalid.profile^=8;CHECK(!b.chassis.stillSafe(b.ci,invalid)); // 另一个P档位也必须拒绝。
  auto late=b.ci;late.now_ms=b.chassis.diagnostics().started_ms+500;
  CHECK(!b.chassis.stillSafe(late,b.gs));
  auto bad_sample=b.ci;bad_sample.steer=0;CHECK(!b.chassis.stillSafe(bad_sample,b.gs));
  // 细分复查与原有门控一致；多条件同时失效均保留位标志，不因增加诊断而放宽许可。
  CHECK(b.chassis.safetyFailureMask(b.ci,b.gs)==0);
  CHECK(b.chassis.safetyFailureMask(bad_sample,b.gs)==CheckStick);
  bad_sample=b.ci;bad_sample.imu=false;bad_sample.feedback=false;
  CHECK(b.chassis.safetyFailureMask(bad_sample,b.gs)==(CheckImu|CheckFeedback));
  CHECK(b.chassis.safetyFailureMask(b.ci,invalid)==CheckSession);
  CHECK(b.chassis.safetyFailureMask(late,b.gs)==CheckTime);
  // 模拟正常控制之后、最终发送之前的IMU失效；停止清状态及下一次右下不能覆盖首次采样。
  b.chassis.recordStopCheck(bad_sample,b.gs,b.chassis.safetyFailureMask(bad_sample,b.gs),2,1);
  const auto captured=b.chassis.diagnostics().stop_check;
  CHECK(captured.count==1 && captured.mask==(CheckImu|CheckFeedback) && captured.source==2);
  CHECK(captured.imu==0 && captured.feedback==0 && captured.peer_token==captured.local_token);
  b.chassis.recordStopCheck(b.ci,invalid,CheckSession,2,2);
  CHECK(b.chassis.diagnostics().stop_check.mask==captured.mask);
  // 实际反馈已越过本轮角度或速度上限，不能靠位置环自动拉回。
  b.position=0.1f;b.tick(1,-0.5f);
  CHECK(b.chassis.command()==0 && b.chassis.diagnostics().last_stop==Reason::MotionLimit);
  b.position=0;
  for(int i=0;i<70;++i)b.tick(2,0);
  CHECK(b.chassis.diagnostics().stop_check.time_ms==captured.time_ms);
  for(int i=0;i<320;++i)b.tick(3,0);
  for(int i=0;i<100;++i)b.tick(1,-0.5f);
  CHECK(b.chassis.command()>0);b.speed=0.6f;b.tick(1,-0.5f);
  CHECK(b.chassis.command()==0 && b.chassis.diagnostics().last_stop==Reason::MotionLimit);
  // 活动时分别失去遥控、IMU、目标反馈、对端快照，或拨回中/下，都必须停止并禁止自动再起。
  for(int fault=0;fault<6;++fault) {
    test_ms=0;Bench trial;
    for(int ms=0;ms<550;++ms)trial.tick(ms<60?2:(ms<360?3:1),ms<400?0:-0.5f);
    CHECK(trial.chassis.command()>0);
    if(fault==0)trial.remote=false;if(fault==1)trial.imu=false;
    if(fault==2)trial.feedback=false;if(fault==3)trial.deliver=false;
    for(int ms=0;ms<130;++ms)trial.tick(fault==4?2:(fault==5?3:1),-0.5f);
    CHECK(trial.chassis.command()==0 && !trial.chassis.output().control);
    trial.remote=trial.imu=trial.feedback=trial.deliver=true;
    for(int ms=0;ms<130;++ms)trial.tick(1,-0.5f);
    CHECK(trial.chassis.command()==0);
  }
  // 云台各类授权撤销都要锁存第一次真实条件；本地五通道原值可区分轻推横杆时的斜向偏量。
  for(int fault=0;fault<8;++fault) {
    test_ms=0;Power cloud(true,false,config());
    Session peer;peer.token=100+fault;peer.phase=1;
    cloud.step(cloudInput(2,0),peer);test_ms=1;cloud.step(cloudInput(3,0),peer);
    test_ms=201;cloud.step(cloudInput(3,0),peer);CHECK(cloud.session().phase==1);
    test_ms=202;peer.phase=2;cloud.step(cloudInput(1,-0.5f),peer);
    CHECK(cloud.session().phase==2 && cloud.output().state.life==robot::LifeState::Working);
    test_ms=203;auto in=cloudInput(1,-0.5f);uint32_t expected=0;
    in.channels_valid=true;in.channels[1]=-0.5f;
    if(fault==0) {in.other_neutral=false;in.channels[0]=0.08f;expected=CheckOtherChannel;}
    if(fault==1) {in.dt_s=0.004f;expected=CheckSample;}
    if(fault==2) {in.peer=false;expected=CheckPeer;}
    if(fault==3) {in.transport=false;expected=CheckTransport;}
    if(fault==4) {in.left=3;expected=CheckSwitch;}
    if(fault==5) {in.feedback=false;expected=CheckFeedback;}
    if(fault==6) {in.imu=false;expected=CheckImu;}
    if(fault==7) {peer.phase=3;expected=CheckSession;}
    cloud.step(in,peer);
    const auto saved=cloud.diagnostics().stop_check;
    CHECK(saved.count==1 && saved.source==4 && saved.mask==expected && saved.run_count==0);
    CHECK(saved.local_phase==2 && saved.local_token!=0 && cloud.session().phase==0);
    CHECK(saved.channels_valid==1 && saved.channels[1]==-0.5f);
    CHECK(cloud.command()==0 && cloud.output().state.life==robot::LifeState::Dead);
    if(fault==0) {
      // 复现现场58：IMU、pitch反馈、遥控、急停释放均好，但其他通道偏离导致云台清就绪/指令位。
      Comm sender(Comm::CodePart::Gimbal,0x211,0x210);auto &wire=sender.gimbal_data();wire.state=cloud.output().state;
      wire.flags.debug_preview=wire.flags.debug_power=wire.flags.imu_ready=wire.flags.pitch_feedback_valid=true;
      wire.flags.remote_valid=wire.flags.emergency_released=true;
      wire.preview={-0.5f,1,1,false,false,true};wire.session=cloud.session();
      uint8_t frame[8];sender.encode(frame);
      CHECK((frame[0]&15)==0 && frame[2]==192 && frame[3]==58);
      CHECK(saved.channels[0]==0.08f && saved.other_neutral==0);
    }
    // 条件恢复、右下、重复stop及再次发布都不覆盖这次首因；旧授权不能自行恢复。
    auto recovered=cloudInput(2,0);peer.phase=1;
    for(int ms=0;ms<10;++ms) {++test_ms;recovered.now_ms=test_ms;cloud.step(recovered,peer);}
    cloud.stop(Reason::PeerConsent);
    const auto kept=cloud.diagnostics().stop_check;
    CHECK(kept.count==1 && kept.mask==saved.mask && kept.time_ms==saved.time_ms);
    CHECK(kept.channels[0]==saved.channels[0] && kept.local_token==saved.local_token);
    if(fault==0) {
      // 新挑战完成新的右下/回中握手后，下一次真实撤销要替换旧记录，不能永久停留在第一次会话。
      peer.token+=1000;++test_ms;cloud.step(cloudInput(2,0),peer);
      ++test_ms;cloud.step(cloudInput(3,0),peer);test_ms+=200;cloud.step(cloudInput(3,0),peer);
      ++test_ms;peer.phase=2;cloud.step(cloudInput(1,-0.5f),peer);CHECK(cloud.session().phase==2);
      ++test_ms;auto late_sample=cloudInput(1,-0.5f);late_sample.dt_s=0.004f;cloud.step(late_sample,peer);
      const auto newer=cloud.diagnostics().stop_check;
      CHECK(newer.count==2 && newer.local_token==peer.token && newer.mask==CheckSample);
    }
  }
  // 复现现场链路：左横杆从有效偏量回到-0.04697，已在0.05死区内；第一页立即撤销而记为18。
  // 云台随后收到flags=6的底盘安全页，锁存2560；这不等同于云台真实遥控失联。
  test_ms=0;Bench deadzone;
  for(int ms=0;ms<680;++ms)deadzone.tick(ms<60?2:(ms<360?3:1),ms<400?0:-0.5f);
  CHECK(deadzone.chassis.command()>0);
  for(int ms=0;ms<35;++ms)deadzone.tick(1,-0.0469696969f);
  CHECK(deadzone.chassis.command()==0 && deadzone.chassis.diagnostics().last_stop==Reason::PeerConsent);
  const auto centered=deadzone.gimbal.diagnostics().stop_check;
  CHECK(centered.source==4 && centered.mask==(CheckPeer|CheckSession));
  CHECK(centered.remote==1 && centered.right==1 && centered.left==1 && centered.other_neutral==1);
  CHECK(centered.imu==1 && centered.feedback==1 && centered.transport==1 && centered.elapsed_ms<500);
  CHECK(centered.channels_valid==1 && std::fabs(centered.channels[1]+0.0469696969f)<0.000001f);
  CHECK(centered.rx_event.reason==Comm::RxFailure::SafetyPage && centered.rx_event.state_byte==192 && centered.rx_event.flags_byte==6);
  CHECK(deadzone.c.rxDiagnostics().order_errors==0 && deadzone.g.rxDiagnostics().order_errors==0);
  // 对照：同一准备条件下持续有效左偏，不进入死区，应由原500ms截止停止，而非提前18。
  test_ms=0;Bench held;
  for(int ms=0;ms<1000;++ms)held.tick(ms<60?2:(ms<360?3:1),ms<400?0:-0.3f);
  CHECK(held.chassis.command()==0 && held.chassis.diagnostics().last_stop==Reason::TimeLimit);
  CHECK(held.chassis.diagnostics().run_count==1 && held.chassis.diagnostics().stop_count==1);
  return 0;
}
