#ifndef STEERED_MOTOR_DEBUG_POWER_HPP_
#define STEERED_MOTOR_DEBUG_POWER_HPP_
#include "motor_debug_preview.hpp"
#include "robot_supervisor.hpp"
#include "pid.hpp"

namespace robot_debug {
using Session=robot::GimbalChassisComm::DebugSession;
struct PowerConfig {
  float kp=0,current_limit_a=0,max_offset_rad=0,max_speed_rad_s=0;
};
// 只有独立动力构建选用这组首轮参数；普通/预演构建保留零值，未改整车PID配置。
inline constexpr PowerConfig kPowerConfig = [] {
  if(kPowerEnabled && drive::kEnabled)return PowerConfig{drive::kKp,drive::kCurrentA,drive::kMaxDisplacement,drive::kMaxActualSpeed};
  // 两个调试旋钮仅在独立动力构建生效；硬边界和双板唯一配置编号由同一头文件校验。
  if(kPowerEnabled)return PowerConfig{direction::kEnabled?direction::kPositionKp:tuning::kPositionKp,
      direction::kEnabled?direction::kCurrentA:tuning::kCurrentLimitA,direction::kEnabled?0:0.0872664626f,
      direction::kEnabled?steer_safety::kActualSpeedLimit:0.5f};
  return PowerConfig{};
}();

// 管理左前转向闭环和双板会话；方向版为位置/速度双环，发送层仍需独立重查许可。
class Power {
 public:
  // 保存角色与参数；历史模式保留500ms，方向版取消试转时限。云台只授权，不输出电机动力。
  Power(bool enabled,bool chassis,const PowerConfig &config=kPowerConfig)
      : enabled_(enabled),chassis_(chassis),config_(config),schedule_(enabled,chassis) {
    hello_world::pid::Pid::Params p;
    p.kp=config.kp;p.output_limit=config.current_limit_a;p.max_dt_s=0.003f;p.error_period=6.283185307179586f;
    pid_.configure(drive::kEnabled?drive::params():(direction::kEnabled?steer_safety::positionParams():p));
    speed_pid_.configure(direction::speedParams());
  }
  // 检查本轮不可通过Watch放宽的上限；未填或非法参数不允许建立动力许可。
  bool configured() const {
    if(drive::kEnabled)return enabled_ && config_.kp==drive::kKp && config_.current_limit_a==drive::kCurrentA &&
        config_.max_offset_rad==drive::kMaxDisplacement && config_.max_speed_rad_s==drive::kMaxActualSpeed && pid_.configured();
    return enabled_ && std::isfinite(config_.kp) && (direction::kFour?config_.kp>=0:config_.kp>0) &&
        config_.kp<=(direction::kEnabled?direction::kHardMaxPositionKp:tuning::kHardMaxKp) &&
        config_.kp==(direction::kEnabled?direction::kPositionKp:tuning::kPositionKp) &&
        config_.current_limit_a==(direction::kEnabled?direction::kCurrentA:tuning::kCurrentLimitA) &&
        std::isfinite(config_.current_limit_a) && config_.current_limit_a>0 &&
        config_.current_limit_a<=(direction::kEnabled?direction::kHardMaxCurrentA:tuning::kHardMaxCurrentA) &&
        (direction::kEnabled?config_.max_offset_rad==0:
            (std::isfinite(config_.max_offset_rad) && config_.max_offset_rad>0 && config_.max_offset_rad<=0.087267f)) &&
        std::isfinite(config_.max_speed_rad_s) && config_.max_speed_rad_s>0 &&
        (direction::kEnabled?config_.max_speed_rad_s==steer_safety::kActualSpeedLimit:config_.max_speed_rad_s<=0.5f) &&
        pid_.configured() && (!direction::kEnabled || speed_pid_.configured());
  }
  // 返回专用诊断状态；普通Supervisor仍停用，不能伪造其configured或ready。
  const robot_control::ControlOutput &output() const {return out_;}
  // 返回本板会话：底盘发布挑战，云台只回显已实际收到的挑战与遥控准备阶段。
  Session session() const {return local_;}
  // 返回本电机私有控制值，唯一发送层复查后才编码到对应槽，诊断镜像不参与此决定。
  float command() const {return command_;}
  // 只生成九个双环实时读数，避免四轮每毫秒为取loops而复制完整故障历史与通信诊断。
  LoopDiagnostics loopDiagnostics() const {
    const auto &d=schedule_.diagnostics();
    return {out_.control?1u:0u,d.run_count,d.target_angle,last_input_.angle,d.error,
        target_velocity_,last_input_.velocity,speed_error_,command_};
  }
  // 以真实条件更新闭环；正常转向回中清PID后可继续，其他停止锁住旧会话、需要新右下准备。
  void step(const Input &input,const Session &peer) {
    const bool new_link_error=!sameLinkErrors(input.link,last_input_.link);
    // 记录仍有控制许可时的真实反馈，包含截止/失效这一周期；不把计算出的指令当反馈。
    if(chassis_ && out_.control)observeCurrent(input);
    last_input_=input;peer_=peer;command_=0;out_={};
    if(!configured()) {stop(Reason::Configuration);return;}
    if(!chassis_) {observeCloud(input,peer);return;}
    const bool down=input.remote && input.right==2;
    if(down && !was_down_) {
      ++generation_;local_.token=(input.now_ms*2654435761u)^generation_;
      if(local_.token==0)local_.token=generation_;
      local_.phase=1;center_paused_=false;resetLoops();peak_=0;
    }
    was_down_=down;
    Input gated=input;
    const bool matching=peer.token!=0 && peer.token==local_.token && peer.target==robot::GimbalChassisComm::kDebugTarget && peer.profile==robot::GimbalChassisComm::kPowerProfile;
    gated.peer=input.peer && matching;
    // 本轮迁移只需旧阶段，避免每1ms复制整份含锁存记录的诊断结构。
    const auto previous_stage=schedule_.diagnostics().stage;
    if(mode::kResumable && previous_stage==Stage::Previewing && !new_link_error && localPauseSafe(input)) {
      const bool centered=input.peer && matching && peer.phase==2 && healthy(input) && input.right==1 && input.neutral;
      const bool center_page=!input.peer && failureReason(input,CheckPeer|CheckSession)==Reason::Centered;
      if(centered || center_page) {pauseForCenter(input,peer);return;}
    }
    if(mode::kResumable && center_paused_ && !input.peer) {
      // 安全第一页已立即停机，最多等原10ms拼包窗口；等待期间绝不输出，坏帧或超时仍锁止。
      if(!new_link_error && localPauseSafe(input) && input.now_ms-center_pause_ms_<robot::GimbalChassisComm::kAssemblyMs) {
        waitAtCenter();return;
      }
      recordStopCheck(input,peer,CheckPeer|CheckSession,1);stop(Reason::PeerConsent);return;
    }
    if(mode::kResumable && center_paused_ && input.peer && input.right==1) {
      const uint32_t failed=healthFailureMask(input)|(!matching || peer.phase!=2?static_cast<uint32_t>(CheckSession):0u);
      if(new_link_error || failed!=0) {
        recordStopCheck(input,peer,failed?failed:static_cast<uint32_t>(CheckPeer),1);
        stop(new_link_error?Reason::PeerConsent:failureReason(input,failed));return;
      }
      if(!input.neutral)center_paused_=false;
    }
    // 云台回显“右上”需要一个完整快照；未开始输出时等待它，不把等待误当成准许动力。
    if(input.right==1 && previous_stage!=Stage::Previewing && peer.phase!=2 && gated.peer) {
      gated.steer=0;gated.neutral=true;
    }
    if(previous_stage==Stage::Previewing && (!matching || peer.phase!=2)) {
      recordStopCheck(input,peer,CheckSession|(!input.peer?static_cast<uint32_t>(CheckPeer):0u),1);
      stop(failureReason(input,CheckSession|(!input.peer?static_cast<uint32_t>(CheckPeer):0u)));return;
    }
    schedule_.step(gated);
    const auto &d=schedule_.diagnostics();
    if(previous_stage==Stage::Previewing && d.stage!=Stage::Previewing) {
      // 样本拒绝也必须先保存现场，避免直接清PID后只剩数字9而丢失原始输入。
      if(d.reason==Reason::Sample)recordStopCheck(gated,peer,CheckSample,1);
      local_.phase=3;recorded_peak_=peak_;saveCurrentFeedback();resetLoops();return;
    }
    const bool prepared=d.stage==Stage::Ready || d.stage==Stage::WaitStick;
    if((previous_stage==Stage::Ready || previous_stage==Stage::WaitStick) &&
        !prepared && d.stage!=Stage::Previewing && !down) {local_.phase=3;resetLoops();return;}
    out_.ready=healthy(input) && input.peer;
    if(prepared)out_.state={robot::LifeState::Resurrection,robot::ControlSource::RemoteControl,
        robot::ChassisMode::Separate,robot::GimbalMode::Disabled};
    if(d.stage!=Stage::Previewing)return;
    if(!gated.peer || peer.phase!=2) {
      recordStopCheck(input,peer,CheckSession|(!input.peer?static_cast<uint32_t>(CheckPeer):0u),1);
      stop(failureReason(input,CheckSession|(!input.peer?static_cast<uint32_t>(CheckPeer):0u)));return;
    }
    if(!withinMotion(input)) {recordStopCheck(input,peer,CheckMotion,1);stop(Reason::MotionLimit);return;}
    if(d.run_count!=last_run_) {
      last_run_=d.run_count;peak_=feedback_peak_=velocity_peak_=0;feedback_samples_=0;observeCurrent(input);resetLoops();
    }
    // 方向双环：外环(rad -> rad/s)再接内环(rad/s -> A)，内环必须使用实际速度反馈。
    float value=0;
    if(drive::kEnabled) {
      // M3508仅用速度环：提供驱动已经把减速比计入vel()，不可再除一次减速比。
      target_velocity_=d.target_velocity;speed_error_=target_velocity_-input.velocity;
      value=pid_.update(target_velocity_,input.velocity,input.dt_s);
    } else {
      value=pid_.updateError(d.error,input.dt_s);
      target_velocity_=direction::kEnabled?value:0;
      speed_error_=direction::kEnabled?target_velocity_-input.velocity:0;
      if(direction::kEnabled)value=speed_pid_.update(target_velocity_,input.velocity,input.dt_s);
    }
    if(!pid_.lastUpdateOk() || (direction::kEnabled && !speed_pid_.lastUpdateOk()) || !std::isfinite(value) || std::fabs(value)>config_.current_limit_a) {
      stop(Reason::Configuration);return;
    }
    command_=value;if(std::fabs(value)>peak_)peak_=std::fabs(value);
    recorded_loops_={1,d.run_count,d.target_angle,input.angle,d.error,target_velocity_,input.velocity,speed_error_,command_};
    if(drive::kEnabled)recorded_displacement_=input.angle-d.start_angle;
    local_.phase=2;out_.ready=true;out_.control=out_.command_valid=true;
    out_.state={robot::LifeState::Working,robot::ControlSource::RemoteControl,
        robot::ChassisMode::Separate,robot::GimbalMode::Disabled};
  }
  // 发送前检查最新真实条件和会话；历史版另核对500ms，方向版不设试转时限。
  bool stillSafe(const Input &input,const Session &peer) const {
    return safetyFailureMask(input,peer)==0;
  }
  // 对原有最终许可逐项列出失败位，仍要求全部通过；此函数与stillSafe共用，避免诊断和实际门控不一致。
  uint32_t safetyFailureMask(const Input &in,const Session &peer) const {
    uint32_t mask=healthFailureMask(in);
    if(!configured())mask|=CheckConfiguration;
    if(!chassis_ || !out_.control || out_.state.life!=robot::LifeState::Working || local_.phase!=2)mask|=CheckLife;
    if(in.right!=1)mask|=CheckSwitch;
    if(!in.peer)mask|=CheckPeer;
    if((direction::kEnabled || drive::kVector)?in.neutral:!(std::fabs(in.steer)>0.05f))mask|=CheckStick;
    if(peer.token!=local_.token || peer.token==0 || peer.phase!=2 || peer.target!=robot::GimbalChassisComm::kDebugTarget ||
        peer.profile!=robot::GimbalChassisComm::kPowerProfile)mask|=CheckSession;
    if(!mode::kResumable && in.now_ms-schedule_.diagnostics().started_ms>=500)mask|=CheckTime;
    if(!withinMotion(in))mask|=CheckMotion;
    if(!std::isfinite(command_) || std::fabs(command_)>config_.current_limit_a)mask|=CheckOutput;
    return mask;
  }
  // 只决定停止后的原因名称，不提供运行许可；运动/时钟保护不能一律误标成18。
  Reason failureReason(const Input &in,uint32_t mask) const {
    if(mask&CheckConfiguration)return Reason::Configuration;
    if(mask&(CheckControlAge|CheckSample))return Reason::Clock;
    if(mask&CheckMotion)return Reason::MotionLimit;
    if(mask&CheckTime)return Reason::TimeLimit;
    if(mode::kWideSession) {
      // readPeer被安全第一页撤销后会清默认数据；应读取已校验的撤销记录，不能拿默认right=2猜急停。
      const auto &e=in.link.rx_event;
      if(!in.peer && e.reason==robot::GimbalChassisComm::RxFailure::SafetyPage && e.control_signal_valid &&
          in.now_ms-e.time_ms<=robot::GimbalChassisComm::kAssemblyMs) {
        if((e.flags_byte&16)==0)return Reason::Remote;
        if(e.right==2 || (e.flags_byte&32)==0)return Reason::Emergency;
        if((e.flags_byte&2)==0)return Reason::Imu;
        if((e.flags_byte&8)==0)return Reason::Feedback;
        if((e.flags_byte&128)!=0)return Reason::PeerConsent;
        if(e.right!=1)return Reason::SwitchChanged;
        if(e.neutral)return Reason::Centered;
      }
      if(in.peer && in.remote) {
        if(in.right==2)return Reason::Emergency;
        if(in.right!=1)return Reason::SwitchChanged;
        if((direction::kEnabled || drive::kVector)?in.neutral:std::fabs(in.steer)<=0.05f)return Reason::Centered;
      }
    }
    if(mask&(CheckPeer|CheckSession))return Reason::PeerConsent;
    if(mask&CheckTransport)return Reason::Transport;
    if(mask&CheckImu)return Reason::Imu;
    if(mask&CheckFeedback)return Reason::Feedback;
    if(mask&CheckRemote)return Reason::Remote;
    if(mask&CheckOtherChannel)return Reason::Mode;
    if(mask&CheckSwitch)return in.right==2?Reason::Emergency:Reason::SwitchChanged;
    if(mask&CheckStick)return Reason::Centered;
    return Reason::Configuration;
  }
  // 在停止清状态之前保存真实采样与会话；source=1控制、2最终复查、3外部停止、4云台撤销授权。
  // 底盘每次实际运行记一次；云台无电机运行计数，按已授权的会话号锁存一次，不读取全局镜像。
  void recordStopCheck(const Input &in,const Session &peer,uint32_t mask,uint32_t source,uint32_t control_age=0) {
    const auto &d=schedule_.diagnostics();
    if(mask==0)return;
    if(chassis_) {
      if(d.run_count==0 || (stop_check_.count!=0 && stop_check_.run_count==d.run_count))return;
    } else if(local_.phase!=2 || local_.token==0 ||
        (stop_check_.count!=0 && stop_check_.local_token==local_.token))return;
    StopCheck saved;saved.count=stop_check_.count+1;saved.run_count=d.run_count;
    saved.neutral=in.neutral;
    saved.sample_failures=(!std::isfinite(in.steer)?1u:0u)|
        (std::isfinite(in.steer) && std::fabs(in.steer)>1.001f?2u:0u)|
        (!std::isfinite(in.angle)?4u:0u)|(!std::isfinite(in.velocity)?8u:0u)|
        (in.neutral && (!in.other_neutral || std::fabs(in.steer)>(direction::kEnabled?0.0001f:0.051f))?16u:0u);
    saved.time_ms=in.now_ms;saved.source=source;saved.mask=mask;saved.control_age_ms=control_age;
    saved.elapsed_ms=chassis_?in.now_ms-d.started_ms:(cloud_seen_active_?in.now_ms-cloud_active_ms_:0);
    saved.transport=in.transport;saved.imu=in.imu;saved.peer=in.peer;saved.feedback=in.feedback;
    saved.remote=in.remote;saved.right=in.right;saved.left=in.left;saved.other_neutral=in.other_neutral;
    saved.local_token=local_.token;saved.peer_token=peer.token;saved.local_phase=local_.phase;saved.peer_phase=peer.phase;
    saved.peer_target=peer.target;saved.peer_profile=peer.profile;saved.snapshot_valid=in.link.snapshot_valid;
    saved.peer_age_ms=in.link.age_ms;saved.peer_fault=in.link.peer_fault;saved.stick=in.steer;saved.dt_s=in.dt_s;
    saved.rx_event=in.link.rx_event;
    saved.channels_valid=in.channels_valid;
    saved.actual_angle=in.angle;saved.start_angle=d.start_angle;saved.actual_velocity=in.velocity;
    saved.target_velocity=target_velocity_;saved.speed_limit=config_.max_speed_rad_s;saved.offset_limit=config_.max_offset_rad;
    for(int i=0;i<5;++i)saved.channels[i]=in.channels[i];
    stop_check_=saved;
  }
  // 为外部保护请求保存当前最近采样，仅在本轮仍有控制许可时调用，不覆盖已锁存的首次失效。
  void recordStopRequest() {
    recordStopCheck(last_input_,peer_,CheckStopRequest,3);
  }
  // 强制停止保留本轮记录，清除实际控制许可和PID；不把“暂时归零”当作可自动恢复。
  void stop(Reason reason) {
    if(!chassis_)recordStopCheck(last_input_,peer_,CheckStopRequest,4);
    if(chassis_)schedule_.cancel(reason);
    local_.phase=3;command_=0;out_={};recorded_peak_=peak_;saveCurrentFeedback();resetLoops();
    cloud_prepared_=cloud_holding_=cloud_reset_=false;
    center_paused_=false;
  }
  // 生成只读诊断副本，明确区分动力构建与预演；源数据仍由私有状态机持有。
  Diagnostics diagnostics() const {
    auto d=schedule_.diagnostics();d.board=chassis_?0:1;d.power_enabled=enabled_;d.enabled=enabled_;d.zero_output_only=0;
    d.session_token=local_.token;d.peer_token=peer_.token;d.local_phase=local_.phase;d.peer_phase=peer_.phase;
    d.life_state=static_cast<uint32_t>(out_.state.life);
    d.session_ready=local_.phase>=1 && local_.phase<=2 && local_.token!=0 && peer_.token==local_.token &&
        peer_.profile==robot::GimbalChassisComm::kPowerProfile && peer_.target==robot::GimbalChassisComm::kDebugTarget && peer_.phase>=1 && peer_.phase<=2;
    d.pid_output=command_;d.final_input=command_;d.peak_output=peak_;d.recorded_peak_output=recorded_peak_;
    d.stop_check=stop_check_;
    d.feedback_current_valid=last_input_.feedback_current_valid;
    d.feedback_current_a=last_input_.feedback_current_valid?last_input_.feedback_current_a:0;
    d.recorded_peak_feedback_current_a=recorded_feedback_peak_;d.recorded_feedback_samples=recorded_feedback_samples_;
    d.configured_kp=config_.kp;d.configured_current_limit_a=config_.current_limit_a;d.configured_profile=robot::GimbalChassisComm::kPowerProfile;
    d.direction_mode=direction::kEnabled;d.target_velocity=target_velocity_;d.speed_error=speed_error_;
    d.time_limit_enabled=!mode::kResumable;
    d.range_limit_enabled=!mode::kResumable;
    d.loops={out_.control?1u:0u,d.run_count,d.target_angle,last_input_.angle,d.error,
        target_velocity_,last_input_.velocity,speed_error_,command_};
    d.recorded_loops=recorded_loops_;
    d.recorded_peak_speed_rad_s=recorded_velocity_peak_;
    d.drive_mode=drive::kEnabled;d.position_loop_enabled=!drive::kEnabled;
    d.configured_actual_speed_limit_rad_s=config_.max_speed_rad_s;
    d.algorithm_version=direction::kEnabled?steer_safety::kAlgorithmVersion:robot::GimbalChassisComm::kDebugTarget;
    if(direction::kEnabled) {
      d.configured_target_speed_rpm=steer_safety::kTargetRpm;d.configured_target_rate_rpm=steer_safety::kRateRpm;
      d.configured_trip_speed_rpm=steer_safety::kTripRpm;
    }
    if(drive::kEnabled) {
      // 只读镜像按实际生效rad/s反算rpm，兼容旧宏覆盖时仍显示真实速度上限。
      d.configured_target_speed_rpm=drive::kMaxTargetSpeed*9.54929658551372f;
      d.configured_trip_speed_rpm=drive::kMaxActualSpeed*9.54929658551372f;
      d.configured_drive_acceleration_rpm_s=drive::kMaxAcceleration*9.54929658551372f;
      d.configured_speed_kp=drive::kKp;d.configured_speed_ki=drive::kKi;d.configured_speed_kd=drive::kKd;
      d.displacement_rad=last_input_.angle-d.start_angle;d.recorded_displacement_rad=recorded_displacement_;
    }
    if(direction::kEnabled) {d.configured_speed_kp=direction::kSpeedKp;d.configured_speed_ki=direction::kSpeedKi;d.configured_speed_kd=direction::kSpeedKd;}
    d.right=last_input_.right;d.left=last_input_.left;d.stick=last_input_.steer;
    if(out_.state.life==robot::LifeState::Working)d.stage=Stage::PowerRunning;
    return d;
  }
 private:
  // 错帧计数只用于把“真正链路故障”与“正常中心安全页”区分，绝不借此授予动力。
  static bool sameLinkErrors(const LinkDiagnostics &a,const LinkDiagnostics &b) {
    return a.crc_errors==b.crc_errors && a.format_errors==b.format_errors && a.order_errors==b.order_errors &&
        a.assembly_timeouts==b.assembly_timeouts && a.replays==b.replays;
  }
  // 回中暂停只保留本地仍健康的准备历史；对端快照恢复前始终是非Working和零输入。
  bool localPauseSafe(const Input &in) const {
    return configured() && in.transport && in.imu && in.feedback && std::isfinite(in.angle) &&
        std::isfinite(in.velocity) && std::isfinite(in.dt_s) && in.dt_s>0 && in.dt_s<=0.003f && withinMotion(in);
  }
  // 发布零输出待杆状态；保留会话挑战，不把它作为电机控制许可。
  void waitAtCenter() {
    command_=0;out_={};out_.ready=true;
    out_.state={robot::LifeState::Resurrection,robot::ControlSource::RemoteControl,robot::ChassisMode::Separate,robot::GimbalMode::Disabled};
    local_.phase=1;
  }
  // 正常回中立即停机、清两环和记录现场；只有此路径可保留准备历史，所有故障仍走stop锁止。
  void pauseForCenter(const Input &in,const Session &peer) {
    recordStopCheck(in,peer,CheckStick,1);schedule_.pauseForCenter(in);
    recorded_peak_=peak_;saveCurrentFeedback();resetLoops();center_paused_=true;center_pause_ms_=in.now_ms;
    waitAtCenter();
  }
  // 只对本轮控制期间有效的电机反馈取绝对峰值；观察次数不是独立CAN帧数，不参与输出许可。
  void observeCurrent(const Input &in) {
    if(in.feedback && std::isfinite(in.velocity) && std::fabs(in.velocity)>velocity_peak_)velocity_peak_=std::fabs(in.velocity);
    if(!in.feedback_current_valid || !std::isfinite(in.feedback_current_a))return;
    ++feedback_samples_;
    const float value=std::fabs(in.feedback_current_a);
    if(value>feedback_peak_)feedback_peak_=value;
  }
  // 停止时保留本轮反馈峰值和有效观察次数，回中后仍能读取；新一轮在真正开始时重置活跃统计。
  void saveCurrentFeedback() {
    recorded_feedback_peak_=feedback_peak_;recorded_feedback_samples_=feedback_samples_;
    recorded_velocity_peak_=velocity_peak_;
    if(drive::kEnabled && last_run_!=0 && last_input_.feedback && std::isfinite(last_input_.angle))
      recorded_displacement_=last_input_.angle-schedule_.diagnostics().start_angle;
  }
  // 把原healthy条件逐项展开为失败位，底盘和云台共用同一判据；诊断不增加或放宽任何门槛。
  uint32_t healthFailureMask(const Input &in) const {
    uint32_t mask=0;
    if(!in.transport)mask|=CheckTransport;
    if(!in.imu)mask|=CheckImu;
    if(!in.feedback)mask|=CheckFeedback;
    if(!in.remote)mask|=CheckRemote;
    if(in.left!=1)mask|=CheckSwitch;
    if(!in.other_neutral)mask|=CheckOtherChannel;
    if(!std::isfinite(in.dt_s) || in.dt_s<=0 || in.dt_s>0.003f || !std::isfinite(in.steer) ||
        std::fabs(in.steer)>1.001f || !std::isfinite(in.angle) || !std::isfinite(in.velocity) ||
        (direction::kEnabled && (!std::isfinite(in.yaw_delta) || (in.neutral && std::fabs(in.steer)>0.0001f))))mask|=CheckSample;
    return mask;
  }
  // 检查实际输入与控制周期，云台和底盘使用同一标准，绝不从全局镜像读取布尔许可。
  bool healthy(const Input &in) const {
    return healthFailureMask(in)==0;
  }
  // 两模式都检查实际速度；仅历史模式另查起点附近的位置范围，越界不自动大幅拉回。
  bool withinMotion(const Input &in) const {
    // M3508采用连续轮轴角，直接计算相对起点位移；禁止用fmod掩盖多转一整圈。
    if(drive::kEnabled)return std::isfinite(in.angle) &&
        (drive::kTuning || std::fabs(in.angle-schedule_.diagnostics().start_angle)<=config_.max_offset_rad) &&
        std::isfinite(in.velocity) && std::fabs(in.velocity)<=config_.max_speed_rad_s;
    // 方向调参已按用户要求取消起点±5°停机；仍检查实际速度，不能改成无限速开环。
    if(direction::kEnabled)return std::isfinite(in.velocity) && std::fabs(in.velocity)<=config_.max_speed_rad_s;
    constexpr float pi=3.14159265358979323846f;
    float delta=std::fmod(in.angle-schedule_.diagnostics().start_angle,2*pi);
    if(delta>pi)delta-=2*pi;
    if(delta< -pi)delta+=2*pi;
    return std::fabs(delta)<=config_.max_offset_rad && std::fabs(in.velocity)<=config_.max_speed_rad_s;
  }
  // 云台只根据本地真实遥控完成回中计时并回显挑战，绝不生成电机输出或替底盘伪造反馈。
  void observeCloud(const Input &in,const Session &peer) {
    schedule_.step(in);
    uint32_t mask=healthFailureMask(in);
    if(!in.peer)mask|=CheckPeer;
    if(peer.token==0 || peer.target!=robot::GimbalChassisComm::kDebugTarget || peer.profile!=robot::GimbalChassisComm::kPowerProfile || peer.phase==0 || peer.phase==3)mask|=CheckSession;
    if(mask!=0) {
      recordStopCheck(in,peer,mask,4);
      local_.phase=0;cloud_prepared_=cloud_holding_=cloud_reset_=false;return;
    }
    if(local_.token!=peer.token) {
      recordStopCheck(in,peer,CheckSession,4);
      local_.token=peer.token;cloud_prepared_=cloud_holding_=cloud_reset_=false;cloud_seen_active_=false;
    }
    if(in.right==2) {
      recordStopCheck(in,peer,CheckSwitch,4);
      local_.phase=0;cloud_reset_=true;cloud_prepared_=cloud_holding_=false;cloud_seen_active_=false;return;
    }
    if(!cloud_reset_) {recordStopCheck(in,peer,CheckLife,4);local_.phase=0;return;}
    if(in.right==3) {
      if(cloud_seen_active_ || !in.neutral) {
        recordStopCheck(in,peer,CheckSwitch,4);cloud_reset_=false;local_.phase=0;return;
      }
      if(!cloud_holding_) {cloud_holding_=true;cloud_hold_ms_=in.now_ms;}
      cloud_prepared_=in.now_ms-cloud_hold_ms_>=200;local_.phase=cloud_prepared_?1:0;
    } else if(in.right==1 && cloud_prepared_) {
      local_.phase=2;
      if(peer.phase==2 && !cloud_seen_active_) {cloud_seen_active_=true;cloud_active_ms_=in.now_ms;}
      if(!mode::kResumable && cloud_seen_active_ && in.now_ms-cloud_active_ms_>=500) {
        recordStopCheck(in,peer,CheckTime,4);
        cloud_reset_=false;cloud_prepared_=false;local_.phase=0;return;
      }
    } else {recordStopCheck(in,peer,CheckSwitch,4);local_.phase=0;cloud_reset_=false;return;}
    out_.ready=true;
    out_.state={local_.phase==2 && peer.phase==2?robot::LifeState::Working:robot::LifeState::Resurrection,
        robot::ControlSource::RemoteControl,robot::ChassisMode::Separate,robot::GimbalMode::Disabled};
    out_.command_valid=out_.state.life==robot::LifeState::Working;
  }
  // 任一停止或新会话都同时清两环；下次不得沿用此前的速度积分或目标速度。
  void resetLoops() {pid_.reset();speed_pid_.reset();target_velocity_=speed_error_=0;}
  bool enabled_,chassis_;PowerConfig config_;Preview schedule_;hello_world::pid::Pid pid_,speed_pid_;
  robot_control::ControlOutput out_{};Session local_{},peer_{};Input last_input_{};
  uint32_t generation_=0,last_run_=0,cloud_hold_ms_=0,cloud_active_ms_=0;
  bool was_down_=false,cloud_reset_=false,cloud_holding_=false,cloud_prepared_=false,cloud_seen_active_=false;
  bool center_paused_=false;uint32_t center_pause_ms_=0;
  float command_=0,peak_=0,recorded_peak_=0;
  float target_velocity_=0,speed_error_=0;
  float recorded_displacement_=0;
  float velocity_peak_=0,recorded_velocity_peak_=0;
  LoopDiagnostics recorded_loops_{};
  float feedback_peak_=0,recorded_feedback_peak_=0;
  uint32_t feedback_samples_=0,recorded_feedback_samples_=0;
  StopCheck stop_check_{};
};
}
#endif
