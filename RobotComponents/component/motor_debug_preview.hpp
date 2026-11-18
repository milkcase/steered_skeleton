#ifndef STEERED_MOTOR_DEBUG_PREVIEW_HPP_
#define STEERED_MOTOR_DEBUG_PREVIEW_HPP_
#include <cmath>
#include <cstdint>
#include "gimbal_chassis_comm.hpp"

// 只打开零输出预演，绝不打开动力通道；普通固件未定义此宏时保持关闭。
#ifndef ROBOT_MOTOR_DEBUG_PREVIEW_ENABLED
#define ROBOT_MOTOR_DEBUG_PREVIEW_ENABLED 0
#endif
#ifndef ROBOT_MOTOR_DEBUG_POWER_ENABLED
#define ROBOT_MOTOR_DEBUG_POWER_ENABLED 0
#endif
static_assert(ROBOT_MOTOR_DEBUG_PREVIEW_ENABLED==0 || ROBOT_MOTOR_DEBUG_PREVIEW_ENABLED==1);
static_assert(ROBOT_MOTOR_DEBUG_POWER_ENABLED==0 || ROBOT_MOTOR_DEBUG_POWER_ENABLED==1);
static_assert(!(ROBOT_MOTOR_DEBUG_PREVIEW_ENABLED && ROBOT_MOTOR_DEBUG_POWER_ENABLED));
static_assert(!robot_debug::direction::kEnabled || ROBOT_MOTOR_DEBUG_POWER_ENABLED==1,"方向双环只能用于独立受限动力构建");
static_assert(!robot_debug::drive::kEnabled || ROBOT_MOTOR_DEBUG_POWER_ENABLED==1,"驱动速度环只能用于独立动力构建");

namespace robot_debug {
inline constexpr bool kPreviewEnabled=ROBOT_MOTOR_DEBUG_PREVIEW_ENABLED==1;
inline constexpr bool kPowerEnabled=ROBOT_MOTOR_DEBUG_POWER_ENABLED==1;
inline constexpr bool kDebugEnabled=kPreviewEnabled || kPowerEnabled;
enum class Stage : uint32_t {Disabled,WaitReset,WaitNeutral,Ready,WaitStick,Previewing,Latched,Rejected,Observer,PowerRunning};
enum class Reason : uint32_t {None,Disabled,Transport,Imu,Peer,Feedback,Remote,Emergency,
    Mode,Sample,Clock,NeedReset,NeedNeutral,Centered,TimeLimit,SwitchChanged,MotionLimit,Configuration,PeerConsent,Transmit};
// 拆分“快照读不到”和“对端条件不满足”，计数仅用于定位问题，不参与动作许可。
struct LinkDiagnostics {
  uint32_t snapshot_valid=0,preview_mode=0,peer_imu=0,peer_axis=0,peer_fault=0,remote_in_snapshot=0;
  uint32_t age_ms=0xFFFFFFFFu,complete_count=0,crc_errors=0,format_errors=0,order_errors=0,assembly_timeouts=0,replays=0;
  uint32_t peer_drops=0,remote_drops=0,last_peer_drop_mask=0,last_remote_snapshot_valid=0,last_remote_source_valid=0,last_drop_ms=0;
  robot::GimbalChassisComm::RxEvent rx_event{};
};
struct Input {
  uint32_t now_ms=0;
  float dt_s=0.001f,steer=0,angle=0,velocity=0,yaw_delta=0;
  float magnitude=0,vector_wheel_target=0;
  uint8_t right=2,left=1;
  bool transport=false,imu=false,peer=false,feedback=false,remote=false;
  bool neutral=false,other_neutral=false;
  LinkDiagnostics link{};
  // 仅本地云台采样提供五通道原值：左竖、左横、右竖、右横、拨轮；不通过此数组授权。
  bool channels_valid=false;
  float channels[5]{};
  bool feedback_current_valid=false;
  float feedback_current_a=0;
};
// 锁存准备阶段第一次丢失条件的现场，避免下一周期NeedReset覆盖原始原因。
struct PrepareLoss {
  uint32_t count=0,time_ms=0,right=2,left=1,neutral=0,other_neutral=0;
  Stage from_stage=Stage::Disabled;
  Reason reason=Reason::None;
  float dt_s=0,stick=0;
};
// 动力输出复查的失败位；可同时有多项，保留真实条件，不把诊断位当作动作许可。
enum StopCheckBits : uint32_t {
  CheckConfiguration=1u,CheckLife=2u,CheckTransport=4u,CheckImu=8u,CheckFeedback=16u,
  CheckRemote=32u,CheckSwitch=64u,CheckOtherChannel=128u,CheckSample=256u,CheckPeer=512u,
  CheckStick=1024u,CheckSession=2048u,CheckTime=4096u,CheckMotion=8192u,CheckOutput=16384u,
  CheckControlAge=32768u,CheckStopRequest=65536u
};
// 底盘按实际运行、云台按已授权会话锁存首次失败；恢复和右下均不覆盖，下一轮真实失败才替换。
struct StopCheck {
  // 样本失败位：1摇杆非有限、2摇杆越界、4角度非有限、8速度非有限、16回中标记矛盾。
  uint32_t sample_failures=0,neutral=0;
  uint32_t count=0,run_count=0,time_ms=0,source=0,mask=0,control_age_ms=0,elapsed_ms=0;
  uint32_t transport=0,imu=0,peer=0,feedback=0,remote=0,right=0,left=0,other_neutral=0;
  uint32_t local_token=0,peer_token=0,local_phase=0,peer_phase=0,peer_target=0,peer_profile=0;
  uint32_t snapshot_valid=0,peer_age_ms=0,peer_fault=0;
  float stick=0,dt_s=0;
  float actual_angle=0,start_angle=0,actual_velocity=0,target_velocity=0,speed_limit=0,offset_limit=0;
  robot::GimbalChassisComm::RxEvent rx_event{};
  uint32_t channels_valid=0;
  float channels[5]{};
};
// Ozone可直接观察的双环读数；active=1才表示当前有控制许可，recorded_loops是最后有效周期快照。
struct LoopDiagnostics {
  uint32_t active=0,run_count=0;
  float position_target_rad=0,position_feedback_rad=0,position_error_rad=0;
  float speed_target_rad_s=0,speed_feedback_rad_s=0,speed_error_rad_s=0;
  float current_command_a=0;
};
// 四舵启动/运行首次失败快照；右下恢复不覆盖，下一会话真实失败才更新，不用于控制许可。
struct FourSteerFault {
  uint32_t count=0,time_ms=0,wheel_id=0,source=0,failure_mask=0;
  Stage stage=Stage::Disabled;Reason reason=Reason::None;
  uint32_t local_token=0,peer_token=0,local_phase=0,peer_phase=0;
  uint32_t transport=0,imu=0,feedback=0,remote=0,peer=0,right=0,left=0;
  uint32_t snapshot_valid=0,peer_age_ms=0,control_age_ms=0,control_mask=0;
  float dt_s=0,angle_rad=0,velocity_rad_s=0;
  robot::GimbalChassisComm::RxEvent rx_event{};
};
// IMU缓存和前台采样节奏的只读观测，区分采样断档、底层未就绪与对端传播失效。
struct RuntimeHealth {
  uint32_t board=0,poll_count=0,ready_updates=0,imu_cache_ready=0,imu_age_ms=0;
  uint32_t max_poll_gap_ms=0,imu_drops=0,last_imu_drop_age_ms=0,last_imu_drop_cache_ready=0;
  uint32_t control_us=0,max_control_us=0;
  uint32_t imu_state=0,imu_last_interval_us=0,imu_max_interval_us=0,imu_timing_fault=0;
  uint32_t imu_recalibrations=0;
};
struct Diagnostics {
  uint32_t sequence=0,board=0,target_id=1,enabled=0,zero_output_only=1;
  Stage stage=Stage::Disabled;
  Reason reason=Reason::Disabled,last_stop=Reason::None;
  uint32_t now_ms=0,started_ms=0,remaining_ms=0,run_count=0,stop_count=0;
  uint32_t transport=0,imu=0,peer=0,feedback=0,remote=0,right=2,left=1,actual_valid=0;
  float stick=0,actual_angle=0,actual_velocity=0,start_angle=0,target_angle=0,error=0;
  float recorded_target=0,end_angle=0,final_input=0;
  LinkDiagnostics link{};
  float dt_s=0;
  uint32_t neutral=0,other_neutral=0;
  PrepareLoss prepare_loss{};
  uint32_t power_enabled=0,life_state=0,session_ready=0,session_token=0,peer_token=0,local_phase=0,peer_phase=0;
  float pid_output=0,peak_output=0,recorded_peak_output=0;
  StopCheck stop_check{};
  uint32_t feedback_current_valid=0,recorded_feedback_samples=0;
  float feedback_current_a=0,recorded_peak_feedback_current_a=0;
  uint32_t configured_profile=0;
  float configured_kp=0,configured_current_limit_a=0;
  uint32_t direction_mode=0,time_limit_enabled=1,range_limit_enabled=1;
  float requested_angle=0,equivalent_angle=0,target_velocity=0,speed_error=0;
  float configured_speed_kp=0,configured_speed_ki=0,configured_speed_kd=0;
  LoopDiagnostics loops{},recorded_loops{};
  uint32_t drive_mode=0,position_loop_enabled=1;
  float requested_speed_rad_s=0,displacement_rad=0,recorded_displacement_rad=0;
  float recorded_peak_speed_rad_s=0;
  float configured_actual_speed_limit_rad_s=0,equivalent_drive_sign=1;
  uint32_t algorithm_version=0;
  float configured_target_speed_rpm=0,configured_target_rate_rpm=0,configured_trip_speed_rpm=0;
  float configured_drive_acceleration_rpm_s=0;
};

// 纯内存的左前转向预演器：只计算目标，不持有电机或CAN指针，也不计算可发送的动力。
class Preview {
 public:
  // 保存编译模式及板角色；云台仅观察，目标角度记录由底盘独立生成。
  Preview(bool enabled,bool chassis) : enabled_(enabled),chassis_(chassis) {d_.board=chassis?0:1;}
  // 返回只读记录，停止后保留上一轮目标、结束角及原因，方便Ozone低频观察。
  const Diagnostics &diagnostics() const {return d_;}
  // 供有真实失效依据的动力包装层撤销本轮，仍执行原有锁止，不自动重新准备。
  void cancel(Reason reason) {reject(reason,Stage::Latched);}
  // 仅正常回中使用：归零并记录一次停止，保留已经完成的右下/回中准备，允许下一次有效偏杆重启。
  void pauseForCenter(const Input &in) {
    if(running_) {++d_.stop_count;d_.recorded_target=d_.target_angle;d_.end_angle=in.angle;d_.last_stop=Reason::Centered;}
    running_=holding_=false;armed_=reset_seen_=true;offset_=0;
    d_.now_ms=in.now_ms;d_.actual_angle=in.angle;d_.actual_velocity=in.velocity;d_.remaining_ms=0;
    d_.stage=Stage::WaitStick;d_.reason=Reason::Centered;d_.final_input=0;
  }
  // 更新真实输入、检查安全条件并执行一次预演；输出字段始终固定为零。
  void step(const Input &in) {
    observeLink(in);
    d_.enabled=enabled_;d_.now_ms=in.now_ms;d_.transport=in.transport;d_.imu=in.imu;
    d_.peer=in.peer;d_.feedback=in.feedback;d_.remote=in.remote;d_.right=in.right;d_.left=in.left;
    d_.stick=in.steer;d_.actual_angle=in.angle;d_.actual_velocity=in.velocity;d_.final_input=0;
    d_.dt_s=in.dt_s;d_.neutral=in.neutral;d_.other_neutral=in.other_neutral;
    d_.actual_valid=chassis_ && in.feedback && std::isfinite(in.angle) && std::isfinite(in.velocity);
    if(!enabled_) {reject(Reason::Disabled,Stage::Disabled);return;}
    if(!in.transport) {reject(Reason::Transport);return;}
    if(!in.imu) {reject(Reason::Imu);return;}
    if(!in.peer) {reject(Reason::Peer);return;}
    if(!in.feedback) {reject(Reason::Feedback);return;}
    if(!in.remote) {reject(Reason::Remote);return;}
    if(!std::isfinite(in.dt_s) || in.dt_s<=0 || in.dt_s>0.003f) {reject(Reason::Clock);return;}
    if(!std::isfinite(in.steer) || std::fabs(in.steer)>1.001f ||
        !std::isfinite(in.angle) || !std::isfinite(in.velocity)) {reject(Reason::Sample);return;}
    // 预演协议的摇杆按0.001量化；中位标记与明显偏杆冲突时拒绝，不信任伪造的回中标记。
    if(in.neutral && (!in.other_neutral || std::fabs(in.steer)>(direction::kEnabled?0.0001f:0.051f))) {reject(Reason::Sample);return;}
    if(in.left!=1 || (in.right!=1 && in.right!=2 && in.right!=3) || !in.other_neutral) {
      reject(Reason::Mode);return;
    }
    if(in.right==2) {
      reject(Reason::Emergency,Stage::WaitNeutral);reset_seen_=true;return;
    }
    if(!chassis_) {reject(Reason::None,Stage::Observer);return;}
    if(!reset_seen_) {reject(Reason::NeedReset,Stage::WaitReset);return;}
    if(in.right==3) {
      if(running_ || (armed_ && d_.stage==Stage::WaitStick)) {reject(Reason::SwitchChanged);return;}
      if(!in.neutral) {reject(Reason::NeedNeutral);return;}
      if(!holding_) {holding_=true;hold_ms_=in.now_ms;}
      armed_=in.now_ms-hold_ms_>=200;
      d_.stage=armed_?Stage::Ready:Stage::WaitNeutral;d_.reason=armed_?Reason::None:Reason::NeedNeutral;
      return;
    }
    if(!armed_) {reject(Reason::NeedReset,Stage::WaitReset);return;}
    if(!mode::kResumable && running_ && in.now_ms-d_.started_ms>=500) {reject(Reason::TimeLimit,Stage::Latched);return;}
    if((direction::kEnabled || drive::kVector)?in.neutral:std::fabs(in.steer)<=0.05f) {
      if(running_)reject(Reason::Centered,Stage::Latched);
      else {d_.stage=Stage::WaitStick;d_.reason=Reason::None;}
      return;
    }
    if(!running_) {
      running_=true;offset_=0;d_.start_angle=in.angle;d_.started_ms=in.now_ms;++d_.run_count;
    }
    // 历史模式限起点±5°；方向版取消该范围，只保留目标每秒最多改变5°，不改普通整车配置。
    constexpr float limit=0.0872664626f;
    if(drive::kEnabled) {
      // 驱动只生成有加速度限制的轮速目标，不将角度误差送入位置环；500ms由上方共同截止。
      d_.requested_speed_rad_s=drive::kVector?in.vector_wheel_target:in.steer*drive::kMaxTargetSpeed;
      const float change=d_.requested_speed_rad_s-offset_,step=drive::kMaxAcceleration*in.dt_s;
      offset_+=change>step?step:(change< -step?-step:change);
      d_.target_velocity=offset_;d_.speed_error=offset_-in.velocity;
      d_.target_angle=d_.start_angle;d_.error=0;
      d_.remaining_ms=drive::kTuning?0:drive::kDurationMs-(in.now_ms-d_.started_ms);d_.stage=Stage::Previewing;d_.reason=Reason::None;return;
    }
    if(direction::kEnabled) {
      if(!std::isfinite(in.yaw_delta)) {reject(Reason::Sample);return;}
      // 云台系方向加本地yaw即车体系方向；目标从当前角起步，按配置限速，无起点±5°限制。
      d_.requested_angle=wrap(in.steer*direction::kPi+in.yaw_delta);
      // 与四轮逆解相同：超过90°改选等效舵角，未来驱动轮速度取反；本轮M3508仍为零。
      d_.equivalent_angle=d_.requested_angle;
      d_.equivalent_drive_sign=1;
      if(std::fabs(wrap(d_.equivalent_angle-in.angle))>direction::kPi*0.5f) {
        d_.equivalent_angle=wrap(d_.equivalent_angle+direction::kPi);
        d_.equivalent_drive_sign=-1;
      }
      const float requested_offset=wrap(d_.equivalent_angle-d_.start_angle);
      const float change=wrap(requested_offset-offset_);
      const float step=steer_safety::kTargetRateRadS*in.dt_s;
      offset_+=change>step?step:(change< -step?-step:change);
      offset_=wrap(offset_);
    } else offset_-=in.steer*limit*in.dt_s;
    if(!direction::kEnabled) {
      if(offset_>limit)offset_=limit;
      if(offset_< -limit)offset_=-limit;
    }
    d_.target_angle=wrap(d_.start_angle+offset_);d_.error=wrap(d_.target_angle-in.angle);
    d_.remaining_ms=direction::kEnabled?0:500-(in.now_ms-d_.started_ms);d_.stage=Stage::Previewing;d_.reason=Reason::None;
  }
 private:
  // 锁存1到0的跳变及其来源，即使Ozone错过短暂状态，也能在停止后定位不同步失效。
  void observeLink(const Input &in) {
    const auto old=d_.link;
    const bool peer_drop=d_.peer!=0 && !in.peer,remote_drop=d_.remote!=0 && !in.remote;
    d_.link=in.link;
    d_.link.peer_drops=old.peer_drops+(peer_drop?1:0);
    d_.link.remote_drops=old.remote_drops+(remote_drop?1:0);
    d_.link.last_peer_drop_mask=old.last_peer_drop_mask;
    d_.link.last_remote_snapshot_valid=old.last_remote_snapshot_valid;
    d_.link.last_remote_source_valid=old.last_remote_source_valid;
    d_.link.last_drop_ms=(peer_drop || remote_drop)?in.now_ms:old.last_drop_ms;
    if(peer_drop)d_.link.last_peer_drop_mask=(!in.link.snapshot_valid?1u:0u)|(!in.link.preview_mode?2u:0u)|
        (!in.link.peer_imu?4u:0u)|(!in.link.peer_axis?8u:0u)|(in.link.peer_fault?16u:0u);
    if(remote_drop) {
      d_.link.last_remote_snapshot_valid=in.link.snapshot_valid;
      d_.link.last_remote_source_valid=in.link.remote_in_snapshot;
    }
  }
  // 记录停止结果并清除准备历史；重新尝试必须先看到右下，不能持续偏杆自动续期。
  void reject(Reason reason,Stage stage=Stage::Rejected) {
    // 只在原准备记录仍存在时记一次，后续等待右下不覆盖现场；主动右下不算准备故障。
    if(!running_ && (holding_ || armed_ || reset_seen_) && reason!=Reason::Emergency &&
        reason!=Reason::NeedReset && reason!=Reason::None && reason!=Reason::Disabled) {
      auto &loss=d_.prepare_loss;++loss.count;loss.time_ms=d_.now_ms;loss.reason=reason;loss.from_stage=d_.stage;
      loss.right=d_.right;loss.left=d_.left;loss.neutral=d_.neutral;loss.other_neutral=d_.other_neutral;
      loss.dt_s=d_.dt_s;loss.stick=d_.stick;
    }
    if(running_) {++d_.stop_count;d_.last_stop=reason;d_.recorded_target=d_.target_angle;d_.end_angle=d_.actual_angle;}
    running_=holding_=armed_=reset_seen_=false;d_.remaining_ms=0;d_.stage=stage;d_.reason=reason;
  }
  // 将预演角度和误差放回最短单圈区间，不改变任何电机编码器零位。
  static float wrap(float angle) {
    constexpr float pi=3.14159265358979323846f;
    float value=std::fmod(angle,2*pi);if(value>=pi)value-=2*pi;if(value< -pi)value+=2*pi;return value;
  }
  bool enabled_,chassis_,reset_seen_=false,holding_=false,armed_=false,running_=false;
  uint32_t hold_ms_=0;float offset_=0;Diagnostics d_{};
};
}

// 每份固件都有独立全局对象；可直接在Ozone中按这个名字读取，不需要局部变量作用域。
extern "C" {inline volatile robot_debug::Diagnostics robot_motor_debug_diagnostics{};}
// 数组下标0~3依次左前、左后、右后、右前；供Ozone观察，绝不读取其值决定动力。
extern "C" {inline volatile robot_debug::LoopDiagnostics robot_four_steer_diagnostics[4]{};}
extern "C" {inline volatile robot_debug::FourSteerFault robot_four_steer_fault{};}
extern "C" {inline volatile robot_debug::RuntimeHealth robot_runtime_health{};}
// 逆解所得四个有符号轮速rad/s；本轮只有下标0允许实际驱动，其余仅供观察。
extern "C" {inline volatile float robot_vector_wheel_targets[4]{};}
// 默认主程序专用只读镜像；不通过修改诊断字段改变许可、目标或PID。
struct RobotMainDiagnostics {
  uint32_t board=0,life=0,control=0,ready=0,imu=0,peer=0,axis_valid=0;
  uint32_t axis_action=0,axis_enabled=0,axis_status=0,prepared=0,prepare_failed=0;
  uint32_t peer_life=0,peer_ready=0,peer_imu=0,peer_axis_valid=0,peer_command_valid=0,peer_fault=0;
  float yaw_target_rad=0,pitch_target_rad=0,axis_angle_rad=0,axis_speed_rad_s=0,axis_speed_target_rad_s=0,torque_nm=0;
  float axis_position_target_rad=0,axis_position_error_rad=0;
  float axis_feedback_torque_nm=0,feedforward_nm=0,pid_torque_nm=0;
  uint32_t axis_feedback_torque_valid=0,feedforward_endpoint=0,torque_limited=0;
  // 两板同名pitch只读实测镜像；底盘来自独立CAN观测帧，valid=0时三项归零。
  float pitch_feedback_angle_rad=0,pitch_feedback_speed_rad_s=0,pitch_feedback_torque_nm=0;
  uint32_t pitch_feedback_valid=0,pitch_feedback_age_ms=0xffffffffu,pitch_feedback_count=0;
  // 与上述pitch反馈来自同一组快照；目标为串级两环真正使用的值，不是未限速的摇杆积分目标。
  float pitch_position_target_rad=0,pitch_speed_target_rad_s=0,pitch_torque_nm=0;
  float pitch_feedforward_nm=0,pitch_pid_torque_nm=0,pitch_position_error_rad=0;
  uint32_t pitch_control=0,pitch_torque_limited=0,pitch_feedforward_endpoint=0;
};
extern "C" {inline volatile RobotMainDiagnostics robot_main_diagnostics{};}
// 一次准备流程的首个停止现场；右下保留，新一轮有效右中准备才允许记录下一次。
struct RobotMainStopDiagnostics {
  uint32_t count=0,time_ms=0,source=0,life=0,control=0,axis_action=0,axis_enabled=0,axis_valid=0,axis_status=0;
  uint32_t peer_valid=0,peer_life=0,peer_ready=0,peer_imu=0,peer_axis_valid=0,peer_command_valid=0,peer_fault=0,commands_usable=0;
  uint32_t imu=0,parameters=0,prepared=0,prepare_failed=0;
  uint32_t remote_valid=0,remote_released=0,remote_arm=0,remote_run=0,remote_neutral=0;
  float dt_s=0;
  uint32_t peer_age_ms=0,rx_completed=0,crc_errors=0,format_errors=0,order_errors=0,assembly_timeouts=0,replays=0;
  robot::GimbalChassisComm::RxEvent rx_event{};
  uint32_t last_peer_life=0,last_peer_ready=0,last_peer_imu=0,last_peer_axis_valid=0,last_peer_command_valid=0,last_peer_fault=0;
  float axis_angle_rad=0,axis_velocity_rad_s=0,last_peer_axis_rad=0;
};
extern "C" {inline volatile RobotMainStopDiagnostics robot_main_stop{};}
extern "C" {inline volatile robot_debug::LoopDiagnostics robot_drive_diagnostics[4]{};}
namespace robot_debug {
// 复制接收撤销事件到只读全局镜像；逐字段写入volatile，事件本身不参与实际许可判断。
inline void publishRxEvent(volatile robot::GimbalChassisComm::RxEvent &out,const robot::GimbalChassisComm::RxEvent &source) {
  out.reason=source.reason;out.time_ms=source.time_ms;out.page=source.page;out.expected_page=source.expected_page;
  out.seq=source.seq;out.expected_seq=source.expected_seq;out.state_byte=source.state_byte;out.flags_byte=source.flags_byte;
  out.control_signal_valid=source.control_signal_valid;out.right=source.right;out.neutral=source.neutral;
}
// 逐字段发布两环目标和真实反馈，调试器写镜像不会反向影响私有PID或动作许可。
inline void publishLoops(volatile LoopDiagnostics &out,const LoopDiagnostics &in) {
  out.active=in.active;out.run_count=in.run_count;
  out.position_target_rad=in.position_target_rad;out.position_feedback_rad=in.position_feedback_rad;out.position_error_rad=in.position_error_rad;
  out.speed_target_rad_s=in.speed_target_rad_s;out.speed_feedback_rad_s=in.speed_feedback_rad_s;out.speed_error_rad_s=in.speed_error_rad_s;
  out.current_command_a=in.current_command_a;
}
// 发布只读诊断；偶数序列表示一次复制完成，实际控制从不读取这个可被调试器写入的对象。
inline void publish(const Diagnostics &source) {
  auto &out=robot_motor_debug_diagnostics;
  out.sequence=out.sequence+1;
  out.board=source.board;out.target_id=source.target_id;out.enabled=source.enabled;out.zero_output_only=source.zero_output_only;
  out.stage=source.stage;out.reason=source.reason;out.last_stop=source.last_stop;
  out.now_ms=source.now_ms;out.started_ms=source.started_ms;out.remaining_ms=source.remaining_ms;
  out.run_count=source.run_count;out.stop_count=source.stop_count;
  out.transport=source.transport;out.imu=source.imu;out.peer=source.peer;out.feedback=source.feedback;
  out.remote=source.remote;out.right=source.right;out.left=source.left;out.stick=source.stick;
  out.actual_valid=source.actual_valid;
  out.actual_angle=source.actual_angle;out.actual_velocity=source.actual_velocity;
  out.start_angle=source.start_angle;out.target_angle=source.target_angle;out.error=source.error;
  out.recorded_target=source.recorded_target;out.end_angle=source.end_angle;out.final_input=source.final_input;
  out.power_enabled=source.power_enabled;out.life_state=source.life_state;
  out.session_ready=source.session_ready;out.session_token=source.session_token;
  out.peer_token=source.peer_token;out.local_phase=source.local_phase;out.peer_phase=source.peer_phase;
  out.pid_output=source.pid_output;out.peak_output=source.peak_output;out.recorded_peak_output=source.recorded_peak_output;
  out.feedback_current_valid=source.feedback_current_valid;out.feedback_current_a=source.feedback_current_a;
  out.recorded_feedback_samples=source.recorded_feedback_samples;
  out.recorded_peak_feedback_current_a=source.recorded_peak_feedback_current_a;
  out.configured_profile=source.configured_profile;out.configured_kp=source.configured_kp;
  out.configured_drive_acceleration_rpm_s=source.configured_drive_acceleration_rpm_s;
  out.configured_current_limit_a=source.configured_current_limit_a;
  out.direction_mode=source.direction_mode;out.time_limit_enabled=source.time_limit_enabled;
  out.range_limit_enabled=source.range_limit_enabled;
  out.requested_angle=source.requested_angle;out.equivalent_angle=source.equivalent_angle;
  out.target_velocity=source.target_velocity;out.speed_error=source.speed_error;
  out.configured_speed_kp=source.configured_speed_kp;out.configured_speed_ki=source.configured_speed_ki;
  out.configured_speed_kd=source.configured_speed_kd;
  publishLoops(out.loops,source.loops);publishLoops(out.recorded_loops,source.recorded_loops);
  out.drive_mode=source.drive_mode;out.position_loop_enabled=source.position_loop_enabled;
  out.requested_speed_rad_s=source.requested_speed_rad_s;out.displacement_rad=source.displacement_rad;
  out.recorded_displacement_rad=source.recorded_displacement_rad;
  out.recorded_peak_speed_rad_s=source.recorded_peak_speed_rad_s;
  out.configured_actual_speed_limit_rad_s=source.configured_actual_speed_limit_rad_s;
  out.equivalent_drive_sign=source.equivalent_drive_sign;
  out.algorithm_version=source.algorithm_version;
  out.configured_target_speed_rpm=source.configured_target_speed_rpm;out.configured_target_rate_rpm=source.configured_target_rate_rpm;
  out.configured_trip_speed_rpm=source.configured_trip_speed_rpm;
  out.stop_check.count=source.stop_check.count;out.stop_check.run_count=source.stop_check.run_count;
  out.stop_check.time_ms=source.stop_check.time_ms;out.stop_check.source=source.stop_check.source;
  out.stop_check.mask=source.stop_check.mask;out.stop_check.control_age_ms=source.stop_check.control_age_ms;
  out.stop_check.sample_failures=source.stop_check.sample_failures;out.stop_check.neutral=source.stop_check.neutral;
  out.stop_check.elapsed_ms=source.stop_check.elapsed_ms;
  out.stop_check.transport=source.stop_check.transport;out.stop_check.imu=source.stop_check.imu;
  out.stop_check.peer=source.stop_check.peer;out.stop_check.feedback=source.stop_check.feedback;
  out.stop_check.remote=source.stop_check.remote;out.stop_check.right=source.stop_check.right;
  out.stop_check.left=source.stop_check.left;out.stop_check.other_neutral=source.stop_check.other_neutral;
  out.stop_check.local_token=source.stop_check.local_token;out.stop_check.peer_token=source.stop_check.peer_token;
  out.stop_check.local_phase=source.stop_check.local_phase;out.stop_check.peer_phase=source.stop_check.peer_phase;
  out.stop_check.peer_target=source.stop_check.peer_target;out.stop_check.peer_profile=source.stop_check.peer_profile;
  out.stop_check.snapshot_valid=source.stop_check.snapshot_valid;out.stop_check.peer_age_ms=source.stop_check.peer_age_ms;
  out.stop_check.peer_fault=source.stop_check.peer_fault;out.stop_check.stick=source.stop_check.stick;
  out.stop_check.dt_s=source.stop_check.dt_s;
  out.stop_check.actual_angle=source.stop_check.actual_angle;out.stop_check.start_angle=source.stop_check.start_angle;
  out.stop_check.actual_velocity=source.stop_check.actual_velocity;out.stop_check.target_velocity=source.stop_check.target_velocity;
  out.stop_check.speed_limit=source.stop_check.speed_limit;out.stop_check.offset_limit=source.stop_check.offset_limit;
  publishRxEvent(out.stop_check.rx_event,source.stop_check.rx_event);
  out.stop_check.channels_valid=source.stop_check.channels_valid;
  for(int i=0;i<5;++i)out.stop_check.channels[i]=source.stop_check.channels[i];
  out.dt_s=source.dt_s;out.neutral=source.neutral;out.other_neutral=source.other_neutral;
  out.prepare_loss.count=source.prepare_loss.count;out.prepare_loss.time_ms=source.prepare_loss.time_ms;
  out.prepare_loss.reason=source.prepare_loss.reason;out.prepare_loss.from_stage=source.prepare_loss.from_stage;
  out.prepare_loss.right=source.prepare_loss.right;out.prepare_loss.left=source.prepare_loss.left;
  out.prepare_loss.neutral=source.prepare_loss.neutral;out.prepare_loss.other_neutral=source.prepare_loss.other_neutral;
  out.prepare_loss.dt_s=source.prepare_loss.dt_s;out.prepare_loss.stick=source.prepare_loss.stick;
  out.link.snapshot_valid=source.link.snapshot_valid;out.link.preview_mode=source.link.preview_mode;
  out.link.peer_imu=source.link.peer_imu;out.link.peer_axis=source.link.peer_axis;out.link.peer_fault=source.link.peer_fault;
  out.link.remote_in_snapshot=source.link.remote_in_snapshot;out.link.age_ms=source.link.age_ms;
  out.link.complete_count=source.link.complete_count;out.link.crc_errors=source.link.crc_errors;
  out.link.format_errors=source.link.format_errors;out.link.order_errors=source.link.order_errors;
  out.link.assembly_timeouts=source.link.assembly_timeouts;out.link.replays=source.link.replays;
  out.link.peer_drops=source.link.peer_drops;out.link.remote_drops=source.link.remote_drops;
  out.link.last_peer_drop_mask=source.link.last_peer_drop_mask;
  out.link.last_remote_snapshot_valid=source.link.last_remote_snapshot_valid;
  out.link.last_remote_source_valid=source.link.last_remote_source_valid;out.link.last_drop_ms=source.link.last_drop_ms;
  publishRxEvent(out.link.rx_event,source.link.rx_event);
  out.sequence=out.sequence+1;
}
}
#endif
