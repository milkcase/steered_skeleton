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
  // 下一档只提高位置环P，使已有50mA上限可实际到达；不提高电流、时限或运动范围。
  if(kPowerEnabled)return PowerConfig{5.0f,0.05f,0.0872664626f,0.5f};
  return PowerConfig{};
}();

// 只管理左前转向的有限位置环和双板会话，不访问HAL或直接发CAN；发送层还必须重查许可。
class Power {
 public:
  // 保存角色及显式参数，复用已验收的500ms调度器；云台仅观察和授权，不生成电机动力。
  Power(bool enabled,bool chassis,const PowerConfig &config=kPowerConfig)
      : enabled_(enabled),chassis_(chassis),config_(config),schedule_(enabled,chassis) {
    hello_world::pid::Pid::Params p;
    p.kp=config.kp;p.output_limit=config.current_limit_a;p.max_dt_s=0.003f;p.error_period=6.283185307179586f;
    pid_.configure(p);
  }
  // 检查本轮不可通过Watch放宽的上限；未填或非法参数不允许建立动力许可。
  bool configured() const {
    return enabled_ && std::isfinite(config_.kp) && config_.kp>0 && config_.kp<=5.0f &&
        std::isfinite(config_.current_limit_a) && config_.current_limit_a>0 && config_.current_limit_a<=0.05f &&
        std::isfinite(config_.max_offset_rad) && config_.max_offset_rad>0 && config_.max_offset_rad<=0.087267f &&
        std::isfinite(config_.max_speed_rad_s) && config_.max_speed_rad_s>0 && config_.max_speed_rad_s<=0.5f && pid_.configured();
  }
  // 返回专用诊断状态；普通Supervisor仍停用，不能伪造其configured或ready。
  const robot_control::ControlOutput &output() const {return out_;}
  // 返回本板会话：底盘发布挑战，云台只回显已实际收到的挑战与遥控准备阶段。
  Session session() const {return local_;}
  // 返回私有控制值，唯一发送层复查后才可把它编码到ID1槽，诊断镜像不参与此决定。
  float command() const {return command_;}
  // 以真实条件更新握手或闭环；所有停止都清PID并锁住旧会话，只有新右下可再次准备。
  void step(const Input &input,const Session &peer) {
    // 记录仍有控制许可时的真实反馈，包含截止/失效这一周期；不把计算出的指令当反馈。
    if(chassis_ && out_.control)observeCurrent(input);
    last_input_=input;peer_=peer;command_=0;out_={};
    if(!configured()) {stop(Reason::Configuration);return;}
    if(!chassis_) {observeCloud(input,peer);return;}
    const bool down=input.remote && input.right==2;
    if(down && !was_down_) {
      ++generation_;local_.token=(input.now_ms*2654435761u)^generation_;
      if(local_.token==0)local_.token=generation_;
      local_.phase=1;pid_.reset();peak_=0;
    }
    was_down_=down;
    Input gated=input;
    const bool matching=peer.token!=0 && peer.token==local_.token && peer.target==1 && peer.profile==robot::GimbalChassisComm::kPowerProfile;
    gated.peer=input.peer && matching;
    // 本轮迁移只需旧阶段，避免每1ms复制整份含锁存记录的诊断结构。
    const auto previous_stage=schedule_.diagnostics().stage;
    // 云台回显“右上”需要一个完整快照；未开始输出时等待它，不把等待误当成准许动力。
    if(input.right==1 && previous_stage!=Stage::Previewing && peer.phase!=2 && gated.peer) {
      gated.steer=0;gated.neutral=true;
    }
    if(previous_stage==Stage::Previewing && (!matching || peer.phase!=2)) {
      recordStopCheck(input,peer,CheckSession|(!input.peer?static_cast<uint32_t>(CheckPeer):0u),1);
      stop(Reason::PeerConsent);return;
    }
    schedule_.step(gated);
    const auto &d=schedule_.diagnostics();
    if(previous_stage==Stage::Previewing && d.stage!=Stage::Previewing) {
      local_.phase=3;recorded_peak_=peak_;saveCurrentFeedback();pid_.reset();return;
    }
    const bool prepared=d.stage==Stage::Ready || d.stage==Stage::WaitStick;
    if((previous_stage==Stage::Ready || previous_stage==Stage::WaitStick) &&
        !prepared && d.stage!=Stage::Previewing && !down) {local_.phase=3;pid_.reset();return;}
    out_.ready=healthy(input) && input.peer;
    if(prepared)out_.state={robot::LifeState::Resurrection,robot::ControlSource::RemoteControl,
        robot::ChassisMode::Separate,robot::GimbalMode::Disabled};
    if(d.stage!=Stage::Previewing)return;
    if(!gated.peer || peer.phase!=2) {
      recordStopCheck(input,peer,CheckSession|(!input.peer?static_cast<uint32_t>(CheckPeer):0u),1);
      stop(Reason::PeerConsent);return;
    }
    if(!withinMotion(input)) {stop(Reason::MotionLimit);return;}
    if(d.run_count!=last_run_) {
      last_run_=d.run_count;peak_=feedback_peak_=0;feedback_samples_=0;observeCurrent(input);pid_.reset();
    }
    const float value=pid_.updateError(d.error,input.dt_s);
    if(!pid_.lastUpdateOk() || !std::isfinite(value) || std::fabs(value)>config_.current_limit_a) {
      stop(Reason::Configuration);return;
    }
    command_=value;if(std::fabs(value)>peak_)peak_=std::fabs(value);
    local_.phase=2;out_.ready=true;out_.control=out_.command_valid=true;
    out_.state={robot::LifeState::Working,robot::ControlSource::RemoteControl,
        robot::ChassisMode::Separate,robot::GimbalMode::Disabled};
  }
  // 发送前检查最新真实条件和会话，直接核对500ms硬截止，不依赖上一控制周期的诊断记录。
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
    if(!(std::fabs(in.steer)>0.05f))mask|=CheckStick;
    if(peer.token!=local_.token || peer.token==0 || peer.phase!=2 || peer.target!=1 ||
        peer.profile!=robot::GimbalChassisComm::kPowerProfile)mask|=CheckSession;
    if(in.now_ms-schedule_.diagnostics().started_ms>=500)mask|=CheckTime;
    if(!withinMotion(in))mask|=CheckMotion;
    if(!std::isfinite(command_) || std::fabs(command_)>config_.current_limit_a)mask|=CheckOutput;
    return mask;
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
    saved.time_ms=in.now_ms;saved.source=source;saved.mask=mask;saved.control_age_ms=control_age;
    saved.elapsed_ms=chassis_?in.now_ms-d.started_ms:(cloud_seen_active_?in.now_ms-cloud_active_ms_:0);
    saved.transport=in.transport;saved.imu=in.imu;saved.peer=in.peer;saved.feedback=in.feedback;
    saved.remote=in.remote;saved.right=in.right;saved.left=in.left;saved.other_neutral=in.other_neutral;
    saved.local_token=local_.token;saved.peer_token=peer.token;saved.local_phase=local_.phase;saved.peer_phase=peer.phase;
    saved.peer_target=peer.target;saved.peer_profile=peer.profile;saved.snapshot_valid=in.link.snapshot_valid;
    saved.peer_age_ms=in.link.age_ms;saved.peer_fault=in.link.peer_fault;saved.stick=in.steer;saved.dt_s=in.dt_s;
    saved.rx_event=in.link.rx_event;
    saved.channels_valid=in.channels_valid;
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
    local_.phase=3;command_=0;out_={};recorded_peak_=peak_;saveCurrentFeedback();pid_.reset();
    cloud_prepared_=cloud_holding_=cloud_reset_=false;
  }
  // 生成只读诊断副本，明确区分动力构建与预演；源数据仍由私有状态机持有。
  Diagnostics diagnostics() const {
    auto d=schedule_.diagnostics();d.board=chassis_?0:1;d.power_enabled=enabled_;d.enabled=enabled_;d.zero_output_only=0;
    d.session_token=local_.token;d.peer_token=peer_.token;d.local_phase=local_.phase;d.peer_phase=peer_.phase;
    d.life_state=static_cast<uint32_t>(out_.state.life);
    d.session_ready=local_.phase>=1 && local_.phase<=2 && local_.token!=0 && peer_.token==local_.token &&
        peer_.profile==robot::GimbalChassisComm::kPowerProfile && peer_.target==1 && peer_.phase>=1 && peer_.phase<=2;
    d.pid_output=command_;d.final_input=command_;d.peak_output=peak_;d.recorded_peak_output=recorded_peak_;
    d.stop_check=stop_check_;
    d.feedback_current_valid=last_input_.feedback_current_valid;
    d.feedback_current_a=last_input_.feedback_current_valid?last_input_.feedback_current_a:0;
    d.recorded_peak_feedback_current_a=recorded_feedback_peak_;d.recorded_feedback_samples=recorded_feedback_samples_;
    d.right=last_input_.right;d.left=last_input_.left;d.stick=last_input_.steer;
    if(out_.state.life==robot::LifeState::Working)d.stage=Stage::PowerRunning;
    return d;
  }
 private:
  // 只对本轮控制期间有效的电机反馈取绝对峰值；观察次数不是独立CAN帧数，不参与输出许可。
  void observeCurrent(const Input &in) {
    if(!in.feedback_current_valid || !std::isfinite(in.feedback_current_a))return;
    ++feedback_samples_;
    const float value=std::fabs(in.feedback_current_a);
    if(value>feedback_peak_)feedback_peak_=value;
  }
  // 停止时保留本轮反馈峰值和有效观察次数，回中后仍能读取；新一轮在真正开始时重置活跃统计。
  void saveCurrentFeedback() {
    recorded_feedback_peak_=feedback_peak_;recorded_feedback_samples_=feedback_samples_;
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
        std::fabs(in.steer)>1.001f || !std::isfinite(in.angle) || !std::isfinite(in.velocity))mask|=CheckSample;
    return mask;
  }
  // 检查实际输入与控制周期，云台和底盘使用同一标准，绝不从全局镜像读取布尔许可。
  bool healthy(const Input &in) const {
    return healthFailureMask(in)==0;
  }
  // 将实际位置和速度限制在本轮小范围；反馈越界直接停止，不自动大幅拉回。
  bool withinMotion(const Input &in) const {
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
    if(peer.token==0 || peer.target!=1 || peer.profile!=robot::GimbalChassisComm::kPowerProfile || peer.phase==0 || peer.phase==3)mask|=CheckSession;
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
      if(cloud_seen_active_ && in.now_ms-cloud_active_ms_>=500) {
        recordStopCheck(in,peer,CheckTime,4);
        cloud_reset_=false;cloud_prepared_=false;local_.phase=0;return;
      }
    } else {recordStopCheck(in,peer,CheckSwitch,4);local_.phase=0;cloud_reset_=false;return;}
    out_.ready=true;
    out_.state={local_.phase==2 && peer.phase==2?robot::LifeState::Working:robot::LifeState::Resurrection,
        robot::ControlSource::RemoteControl,robot::ChassisMode::Separate,robot::GimbalMode::Disabled};
    out_.command_valid=out_.state.life==robot::LifeState::Working;
  }
  bool enabled_,chassis_;PowerConfig config_;Preview schedule_;hello_world::pid::Pid pid_;
  robot_control::ControlOutput out_{};Session local_{},peer_{};Input last_input_{};
  uint32_t generation_=0,last_run_=0,cloud_hold_ms_=0,cloud_active_ms_=0;
  bool was_down_=false,cloud_reset_=false,cloud_holding_=false,cloud_prepared_=false,cloud_seen_active_=false;
  float command_=0,peak_=0,recorded_peak_=0;
  float feedback_peak_=0,recorded_feedback_peak_=0;
  uint32_t feedback_samples_=0,recorded_feedback_samples_=0;
  StopCheck stop_check_{};
};
}
#endif
