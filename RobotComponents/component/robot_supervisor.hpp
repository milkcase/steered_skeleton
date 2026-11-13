#ifndef STEERED_ROBOT_SUPERVISOR_HPP_
#define STEERED_ROBOT_SUPERVISOR_HPP_
#include "robot_control_config.hpp"
#include "gimbal_chassis_comm.hpp"
#include <cmath>
namespace robot_control {
enum class Board {Chassis,Gimbal};
enum class AxisAction {Disable,ArmZero,Zero,Control};
struct RemoteIntent {
  bool valid=false,released=false,arm=false,run=false,neutral=false;
  robot::ChassisMode mode=robot::ChassisMode::Disabled;
  robot::ChassisMotion motion{};
  float yaw_rate=0,pitch_rate=0;
};
struct ControlInput {
  uint32_t now_ms=0;
  float dt_s=0.001f;
  bool local_ready=false,imu_ready=false,axis_valid=false;
  bool peer_valid=false;
  robot::ModuleState peer_state{};
  robot::GimbalChassisComm::Flags peer_flags{};
  robot::GimbalMotion peer_feedback{};
  float peer_chassis_yaw=0,axis_angle=0,peer_yaw_target=0;
  RemoteIntent remote{};
};
struct ControlOutput {
  robot::ModuleState state{};
  bool ready=false,remote_valid=false,emergency_released=false,command_valid=false;
  bool control=false;
  AxisAction axis_action=AxisAction::Disable;
  robot::ChassisMotion motion{};
  robot::GimbalMotion target{};
};
// 纯状态机，不读 HAL、不使能电机。硬件层必须再次检查本地反馈和最新急停状态。
class Supervisor {
 public:
  // 保存板角色、参数并创建独立跟随 PID；默认参数保持停用。
  Supervisor(Board board,const Params &params) : board_(board),params_(params),follow_(params.follow) {}
  // 检查三种模式共同需要的遥控范围，不能拿合法零参数当成可运行参数。
  bool configured() const {
    return params_.remote_mapping_confirmed && positive(params_.max_vx_m_s) && positive(params_.max_vy_m_s) &&
        positive(params_.max_w_rad_s) && params_.max_w_rad_s<=32.767f &&
        params_.max_vx_m_s<=32.767f && params_.max_vy_m_s<=32.767f &&
        positive(params_.yaw_rate_rad_s) && positive(params_.pitch_rate_rad_s) &&
        std::isfinite(params_.deadzone) && params_.deadzone>0 && params_.deadzone<0.2f &&
        params_.neutral_hold_ms>=100 && params_.enable_window_ms>=params_.neutral_hold_ms &&
        params_.enable_window_ms<=5000 && positive(params_.max_dt_s) && params_.max_dt_s<=0.01f &&
        follow_.configured() && params_.follow.kp>0 && params_.follow.ki>=0 && params_.follow.kd>=0 &&
        params_.follow.output_limit>0 && params_.follow.output_limit<=params_.max_w_rad_s &&
        std::fabs(params_.follow.error_period-2*kPi)<0.00001f;
  }
  // 最外层安全出口，清除所有历史；恢复必须重新右中回中，不能带着原指令直接回 Working。
  void stop() {out_={};armed_=false;holding_=false;seeded_=false;follow_.reset();}
  // 按急停/就绪、恢复、Working 的顺序生成当前周期决策，不在恢复阶段生成回中运动。
  const ControlOutput &update(const ControlInput &in) {
    const auto previous=out_.state;
    out_={};
    out_.ready=configured() && in.local_ready && in.imu_ready;
    out_.remote_valid=in.remote.valid;
    out_.emergency_released=in.remote.valid && in.remote.released;
    const bool valid_mode=in.remote.mode==robot::ChassisMode::Separate ||
        in.remote.mode==robot::ChassisMode::Follow || in.remote.mode==robot::ChassisMode::Spin;
    if(!out_.ready || !in.remote.valid || !in.remote.released || !valid_mode ||
        !std::isfinite(in.dt_s) || in.dt_s<=0 || in.dt_s>params_.max_dt_s ||
        !in.peer_valid || !in.peer_flags.ready || !in.peer_flags.imu_ready ||
        !in.peer_flags.remote_valid || !in.peer_flags.emergency_released || in.peer_flags.fault ||
        !robot::isValid(in.peer_state) || !robot::isFinite(in.remote.motion) ||
        (in.peer_state.life!=robot::LifeState::Dead &&
            (in.peer_state.source!=robot::ControlSource::RemoteControl || in.peer_state.gimbal_mode!=robot::GimbalMode::JointPosition)) ||
        !std::isfinite(in.remote.yaw_rate) || !std::isfinite(in.remote.pitch_rate)) {
      resetHistory();return out_;
    }
    out_.state={robot::LifeState::Resurrection,robot::ControlSource::RemoteControl,in.remote.mode,robot::GimbalMode::JointPosition};
    out_.command_valid=true;
    if(board_==Board::Gimbal) {
      // 从运行拨回自检时重新计时，不能沿用上一次回中时间直接再次放行。
      if(previous.life==robot::LifeState::Working && in.remote.arm) {
        armed_=false;holding_=false;seeded_=false;follow_.reset();
      }
      if(in.remote.arm && in.remote.neutral) {
        if(!holding_) {holding_=true;hold_start_=in.now_ms;}
        if(in.now_ms-hold_start_>=params_.neutral_hold_ms && !arm_failed_) {
          if(!armed_) {armed_=true;arm_start_=in.now_ms;}
        }
      } else if(!in.remote.run) {
        resetHistory();out_.state.life=robot::LifeState::Dead;out_.command_valid=false;return out_;
      }
    } else {
      // 底盘只接受云台的恢复/运行阶段，不依赖未接线的本地 USART3。
      if(in.peer_state.life==robot::LifeState::Dead) {
        resetHistory();out_.state.life=robot::LifeState::Dead;out_.command_valid=false;return out_;
      }
      if(!armed_ && !arm_failed_ && in.peer_state.life==robot::LifeState::Resurrection && in.peer_flags.command_valid) {
        armed_=true;arm_start_=in.now_ms;
      }
    }
    const bool peer_axis=board_==Board::Gimbal?in.peer_flags.yaw_feedback_valid:in.peer_flags.pitch_feedback_valid;
    if(armed_ && in.now_ms-arm_start_>=params_.enable_window_ms && (!in.axis_valid || !peer_axis)) {
      armed_=false;arm_failed_=true;
    }
    if(previous.life==robot::LifeState::Working && (!in.axis_valid || !peer_axis)) {
      armed_=false;arm_failed_=true;seeded_=false;follow_.reset();
      out_.state.life=robot::LifeState::Dead;out_.command_valid=false;return out_;
    }
    if(!armed_ || arm_failed_) {
      out_.command_valid=false;if(arm_failed_)out_.state.life=robot::LifeState::Dead;return out_;
    }
    const bool want_work=board_==Board::Gimbal?in.remote.run:in.peer_state.life==robot::LifeState::Working;
    if(!want_work || !in.axis_valid || !peer_axis) {
      // 使能只在一次有界恢复窗口内请求，超时后必须回急停解除 arm_failed_。
      // 两轴已反馈就绪时可在右中保持零输入等待运行；只有缺反馈时才申请有界使能。
      out_.axis_action=in.axis_valid && peer_axis?AxisAction::Zero:AxisAction::ArmZero;
      seeded_=false;follow_.reset();return out_;
    }
    out_.state.life=robot::LifeState::Working;
    out_.axis_action=AxisAction::Zero;
    const bool both_work=in.peer_state.life==robot::LifeState::Working &&
        in.peer_state.chassis_mode==out_.state.chassis_mode;
    if(both_work && !in.peer_flags.command_valid) {stop();return out_;}
    if(!both_work || previous.life!=robot::LifeState::Working || previous.chassis_mode!=out_.state.chassis_mode) {
      seeded_=false;follow_.reset();return out_;
    }
    out_.control=true;out_.axis_action=AxisAction::Control;
    out_.motion=in.remote.motion;
    if(std::fabs(out_.motion.vx)>params_.max_vx_m_s || std::fabs(out_.motion.vy)>params_.max_vy_m_s ||
        std::fabs(out_.motion.w)>params_.max_w_rad_s || !std::isfinite(in.peer_yaw_target)) {
      stop();return out_;
    }
    if(board_==Board::Gimbal) {
      if(!std::isfinite(in.peer_chassis_yaw) || !std::isfinite(in.peer_feedback.yaw) || !std::isfinite(in.axis_angle)) {
        stop();return out_;
      }
      // 以本次恢复的相对航向建立保持目标，底盘转动时扣除其 IMU yaw；不假设两板绝对航向相同。
      if(!seeded_) {world_yaw_=wrap(in.peer_feedback.yaw+in.peer_chassis_yaw);pitch_=in.axis_angle;seeded_=true;}
      world_yaw_=wrap(world_yaw_+in.remote.yaw_rate*in.dt_s);
      pitch_+=in.remote.pitch_rate*in.dt_s;
      out_.target={wrap(world_yaw_-in.peer_chassis_yaw),pitch_};
    } else {
      out_.target.yaw=wrap(in.peer_yaw_target);
      if(in.remote.mode==robot::ChassisMode::Follow) {
        out_.motion.w=follow_.updateError(wrap(in.axis_angle),in.dt_s);
        if(!follow_.lastUpdateOk()) {stop();return out_;}
      }
    }
    return out_;
  }
  // 返回当前状态与动作许可，只读结果不能代替最新设备状态判断。
  const ControlOutput &output() const {return out_;}
  // pitch 积分目标在实际软件限位后回写，防止持续推杆在边界外累计隐藏目标。
  void acceptPitchTarget(float pitch) {if(std::isfinite(pitch))pitch_=pitch;}
  // 归一化单圈角度供相对 yaw 保持与跟随误差使用。
  static float wrap(float value) {
    value=std::fmod(value,2*kPi);if(value>=kPi)value-=2*kPi;if(value< -kPi)value+=2*kPi;return value;
  }
 private:
  // 判断动力参数为有限正数。
  static bool positive(float value) {return std::isfinite(value) && value>0;}
  // 复位恢复许可、目标和 PID；保留本周期本地就绪诊断，故障许可不带到下周期。
  void resetHistory() {armed_=false;holding_=false;arm_failed_=false;seeded_=false;follow_.reset();}
  static constexpr float kPi=3.141592653589793f;
  Board board_;Params params_;hello_world::pid::Pid follow_;
  ControlOutput out_{};
  bool armed_=false,holding_=false,arm_failed_=false,seeded_=false;
  uint32_t hold_start_=0,arm_start_=0;
  float world_yaw_=0,pitch_=0;
};
}
#endif
