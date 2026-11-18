#ifndef STEERED_ROBOT_SUPERVISOR_HPP_
#define STEERED_ROBOT_SUPERVISOR_HPP_
#include "robot_control_config.hpp"
#include "gimbal_chassis_comm.hpp"
#include <algorithm>
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
  float chassis_yaw=0,peer_world_yaw_target=0;
  bool peer_world_yaw_target_valid=false;
  RemoteIntent remote{};
};
struct ControlOutput {
  robot::ModuleState state{};
  bool ready=false,remote_valid=false,emergency_released=false,command_valid=false;
  bool control=false;
  AxisAction axis_action=AxisAction::Disable;
  robot::ChassisMotion motion{};
  robot::GimbalMotion target{};
  float follow_error_rad=0,follow_pid_rate_rad_s=0;
  float world_yaw_target=0;
  bool world_yaw_target_valid=false;
  bool chassis_heading_hold=false;
  float chassis_heading_target_rad=0,chassis_heading_error_rad=0;
};
// 纯状态机，不读 HAL、不使能电机。硬件层必须再次检查本地反馈和最新急停状态。
class Supervisor {
 public:
  // 保存板角色、参数并创建独立跟随 PID；默认参数保持停用。
  Supervisor(Board board,const Params &params) : board_(board),params_(params),follow_(params.follow) {}
  // 仅在外层确认急停后更换整组遥控映射，清除旧准备许可、目标与跟随积分。
  bool configure(const Params &params) {stop();params_=params;follow_.configure(params.follow);return configured();}
  // 检查三种模式共同需要的遥控范围，不能拿合法零参数当成可运行参数。
  bool configured() const {
    return params_.remote_mapping_confirmed && positive(params_.max_vx_m_s) && positive(params_.max_vy_m_s) &&
        std::isfinite(params_.aim_acceleration_rad_s2) && params_.aim_acceleration_rad_s2>=0 &&
        positive(params_.max_w_rad_s) && params_.max_w_rad_s<=32.767f &&
        params_.max_vx_m_s<=32.767f && params_.max_vy_m_s<=32.767f &&
        positive(params_.yaw_rate_rad_s) && positive(params_.pitch_rate_rad_s) &&
        std::isfinite(params_.deadzone) && params_.deadzone>0 && params_.deadzone<0.2f &&
        params_.neutral_hold_ms>=100 && params_.enable_window_ms>=params_.neutral_hold_ms &&
        params_.enable_window_ms<=5000 && positive(params_.max_dt_s) && params_.max_dt_s<=0.01f &&
        std::isfinite(params_.follow_deadband_rad) && params_.follow_deadband_rad>=0 && params_.follow_deadband_rad<kPi &&
        std::isfinite(params_.follow_accel_rad_s2) && params_.follow_accel_rad_s2>=0 &&
        (!main_tuning::enabled || !params_.allow_spin || (positive(params_.spin_max_rad_s) && params_.spin_max_rad_s<=params_.max_w_rad_s &&
          std::isfinite(params_.spin_target_rad_s) && std::fabs(params_.spin_target_rad_s)<=params_.spin_max_rad_s &&
          positive(params_.spin_accel_rad_s2))) &&
        follow_.configured() && params_.follow.kp>0 && params_.follow.ki>=0 && params_.follow.kd>=0 &&
        params_.follow.output_limit>0 && params_.follow.output_limit<=params_.max_w_rad_s &&
        std::fabs(params_.follow.error_period-2*kPi)<0.00001f;
  }
  // 最外层安全出口，清除所有历史；恢复必须重新右中回中，不能带着原指令直接回 Working。
  void stop() {out_={};armed_=false;holding_=false;seeded_=false;resetFollow();yaw_rate_state_=pitch_rate_state_=0;}
  // 按急停/就绪、恢复、Working 的顺序生成当前周期决策，不在恢复阶段生成回中运动。
  const ControlOutput &update(const ControlInput &in) {
    const auto previous=out_.state;
    const bool held_chassis_heading=out_.chassis_heading_hold;
    const float previous_heading_target=out_.chassis_heading_target_rad;
    out_={};
    out_.ready=configured() && in.local_ready && in.imu_ready;
    out_.remote_valid=in.remote.valid;
    out_.emergency_released=in.remote.valid && in.remote.released;
    const bool valid_mode=in.remote.mode==robot::ChassisMode::Separate || in.remote.mode==robot::ChassisMode::Follow ||
        (params_.allow_spin && in.remote.mode==robot::ChassisMode::Spin);
    if(in.peer_flags.debug_preview || !out_.ready || !in.remote.valid || !in.remote.released || !valid_mode ||
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
        // 自检过程中换档重新计回中时间；正式运行后的换档只做零输出交接。
        if(main_tuning::enabled && holding_ && prepared_mode_!=in.remote.mode) {armed_=false;holding_=false;}
        if(!holding_) {holding_=true;hold_start_=in.now_ms;prepared_mode_=in.remote.mode;}
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
        armed_=true;arm_start_=in.now_ms;prepared_mode_=in.remote.mode;
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
      seeded_=false;resetFollow();return out_;
    }
    out_.state.life=robot::LifeState::Working;
    out_.axis_action=AxisAction::Zero;
    const bool both_work=in.peer_state.life==robot::LifeState::Working &&
        in.peer_state.chassis_mode==out_.state.chassis_mode;
    if(both_work && !in.peer_flags.command_valid) {stop();return out_;}
    if(!both_work || previous.life!=robot::LifeState::Working || previous.chassis_mode!=out_.state.chassis_mode) {
      // 正常左拨杆换档保留云台瞄准目标，等对端以新模式返回完整Working帧。
      if(!main_tuning::enabled || previous.life!=robot::LifeState::Working)seeded_=false;
      resetFollow();yaw_rate_state_=pitch_rate_state_=0;return out_;
    }
    out_.control=true;out_.axis_action=AxisAction::Control;
    out_.motion=in.remote.motion;
    const bool fixed_spin=main_tuning::enabled && in.remote.mode==robot::ChassisMode::Spin;
    const bool local_yaw_target=main_tuning::enabled && robot::GimbalChassisComm::supportsWorldYawTarget(in.remote.mode);
    // 旧云台可能还发[-1,1]的拨轮比例；保留格式校验，自旋目标由底盘独立生成。
    const float input_w_limit=fixed_spin?1.0f:params_.max_w_rad_s;
    if(std::fabs(out_.motion.vx)>params_.max_vx_m_s || std::fabs(out_.motion.vy)>params_.max_vy_m_s ||
        std::fabs(out_.motion.w)>input_w_limit || !std::isfinite(in.peer_yaw_target)) {
      stop();return out_;
    }
    if(board_==Board::Gimbal) {
      if(!std::isfinite(in.peer_chassis_yaw) || !std::isfinite(in.peer_feedback.yaw) || !std::isfinite(in.axis_angle)) {
        stop();return out_;
      }
      // 以本次恢复的相对航向建立保持目标，底盘转动时扣除其 IMU yaw；不假设两板绝对航向相同。
      if(!seeded_) {world_yaw_=wrap(in.peer_feedback.yaw+in.peer_chassis_yaw);pitch_=params_.start_level_pitch?0:in.axis_angle;seeded_=true;}
      // 偏杆限加速度；回中冻结角度目标，位置/速度双环仍持续保持。
      auto rate=[&](float request,float &previous) {
        if(request==0 || params_.aim_acceleration_rad_s2<=0)previous=request;
        else {const float step=params_.aim_acceleration_rad_s2*in.dt_s,change=request-previous;
          previous+=change>step?step:(change< -step?-step:change);}
        return previous;
      };
      world_yaw_=wrap(world_yaw_+rate(in.remote.yaw_rate,yaw_rate_state_)*in.dt_s);
      pitch_+=rate(in.remote.pitch_rate,pitch_rate_state_)*in.dt_s;
      out_.target={wrap(world_yaw_-in.peer_chassis_yaw),pitch_};
      if(local_yaw_target) {
        out_.world_yaw_target=world_yaw_;out_.world_yaw_target_valid=true;
      }
    } else {
      out_.target.yaw=wrap(in.peer_yaw_target);
      if(local_yaw_target && in.peer_world_yaw_target_valid) {
        if(!std::isfinite(in.chassis_yaw) || !std::isfinite(in.peer_world_yaw_target)) {stop();return out_;}
        // 对地目标只做一次CAN传输，本机每周期扣除最新底盘航向，避免追赶往返延迟的相对目标。
        out_.world_yaw_target=wrap(in.peer_world_yaw_target);out_.world_yaw_target_valid=true;
        out_.target.yaw=wrap(out_.world_yaw_target-in.chassis_yaw);
      }
      if(in.remote.mode==robot::ChassisMode::Follow) {
        if(!std::isfinite(in.axis_angle)) {stop();return out_;}
        out_.follow_error_rad=wrap(in.axis_angle);
        if(!updateChassisHeading(out_.follow_error_rad,in.dt_s,&out_.follow_pid_rate_rad_s)) {stop();return out_;}
      } else if(main_tuning::enabled && in.remote.mode==robot::ChassisMode::Separate) {
        if(!std::isfinite(in.chassis_yaw)) {stop();return out_;}
        // 进入分离即锁本地车头航向；云台正在转动或操纵平移都不更新这个参考。
        out_.chassis_heading_hold=true;
        out_.chassis_heading_target_rad=held_chassis_heading?previous_heading_target:wrap(in.chassis_yaw);
        out_.chassis_heading_error_rad=wrap(out_.chassis_heading_target_rad-in.chassis_yaw);
        if(!updateChassisHeading(out_.chassis_heading_error_rad,in.dt_s)) {stop();return out_;}
      }
      if(fixed_spin) {
        // 左下档取得Working许可后才按斜率升到固定转速，退出或失效由状态机统一归零。
        updateSpinMotion(in.dt_s);
      }
    }
    return out_;
  }
  // 返回当前状态与动作许可，只读结果不能代替最新设备状态判断。
  const ControlOutput &output() const {return out_;}
  // 周期中发现正常换档时暂停输出，保留Working握手与原瞄准目标。
  void pauseForModeChange() {
    out_.control=false;out_.axis_action=AxisAction::Zero;out_.motion={};
    out_.world_yaw_target_valid=false;out_.chassis_heading_hold=false;
    resetFollow();yaw_rate_state_=pitch_rate_state_=0;
  }
  // 只读返回本次恢复准备是否已完成，不改变使能窗口或放行动力。
  bool prepared() const {return armed_;}
  // 只读返回关节反馈握手是否失败，故障仍由原有状态机要求右下复位。
  bool preparationFailed() const {return arm_failed_;}
  // pitch 积分目标在实际软件限位后回写，防止持续推杆在边界外累计隐藏目标。
  void acceptPitchTarget(float pitch) {if(std::isfinite(pitch))pitch_=pitch;}
  // 归一化单圈角度供相对 yaw 保持与跟随误差使用。
  static float wrap(float value) {
    value=std::fmod(value,2*kPi);if(value>=kPi)value-=2*kPi;if(value< -kPi)value+=2*kPi;return value;
  }
 private:
  // 保留普通模式的满杆平移预算，使有效自转目标既不挤占平移，也不随摇杆改变。
  void updateSpinMotion(float dt_s) {
    constexpr float translation_limit=robot_debug::drive::kMaxTargetSpeed*vehicle_config::kWheelRadiusM;
    const float vx=out_.motion.vx,vy=out_.motion.vy;
    const float translation_sq=vx*vx+vy*vy;
    // 板间报文量化可能使斜向满杆略大于圆形上限；统一缩放以保留方向。
    if(translation_sq>translation_limit*translation_limit) {
      const float scale=translation_limit/std::sqrt(translation_sq);
      out_.motion.vx=vx*scale;
      out_.motion.vy=vy*scale;
    }
    const float wheel_limit=spinWheelRateLimit();
    // yaw 关节需要抵消车体自转；预留手动 yaw 速度后，取云台与驱动轮共同允许的车体转速。
    // pitch 围绕自身横轴运行，底盘平面自转不占用其速度预算。
    constexpr float yaw_joint_limit=std::min(ROBOT_YAW_POSITION_SPEED_RPM,ROBOT_YAW_TARGET_RATE_RPM)*main_tuning::rpm;
    const float yaw_headroom=std::fmax(0.0f,yaw_joint_limit-params_.yaw_rate_rad_s);
    const float spin_limit=std::fmin(wheel_limit,yaw_headroom);
    const float spin_rate=std::copysign(std::fmin(std::fabs(params_.spin_target_rad_s),spin_limit),params_.spin_target_rad_s);
    out_.motion.w=applyTurnRate(spin_rate,params_.spin_accel_rad_s2,dt_s);
  }
  // 判断动力参数为有限正数。
  static bool positive(float value) {return std::isfinite(value) && value>0;}
  // 跟随和分离航向保持输入不同的最短角差，共用底盘PID、死区、限速和斜率。
  bool updateChassisHeading(float error,float dt_s,float *pid_rate=nullptr) {
    const float outside=std::fabs(error)-params_.follow_deadband_rad;
    if(outside<=0) {
      resetFollow();out_.motion.w=0;
      return true;
    }
    const float rate=follow_.updateError(std::copysign(outside,error),dt_s);
    if(!follow_.lastUpdateOk())return false;
    if(pid_rate!=nullptr)*pid_rate=rate;
    out_.motion.w=applyTurnRate(rate,params_.follow_accel_rad_s2,dt_s);
    return true;
  }
  // 角速度指令按限制平滑变化；目标为零时直接请求制动。
  float applyTurnRate(float request,float acceleration,float dt_s) {
    if(request==0 || acceleration<=0)turn_rate_rad_s_=request;
    else turn_rate_rad_s_+=hello_world::pid::detail::clampSymmetric(request-turn_rate_rad_s_,acceleration*dt_s);
    return turn_rate_rad_s_;
  }
  // 在撤销许可、回到误差死区和重新握手时清除积分与历史转速。
  void resetFollow() {follow_.reset();turn_rate_rad_s_=0;}
  // 复位恢复许可、目标和 PID；保留本周期本地就绪诊断，故障许可不带到下周期。
  void resetHistory() {armed_=false;holding_=false;arm_failed_=false;seeded_=false;resetFollow();yaw_rate_state_=pitch_rate_state_=0;}
  float yaw_rate_state_=0,pitch_rate_state_=0;
  float turn_rate_rad_s_=0;
  static constexpr float kPi=3.141592653589793f;
  Board board_;Params params_;hello_world::pid::Pid follow_;
  ControlOutput out_{};
  bool armed_=false,holding_=false,arm_failed_=false,seeded_=false;
  robot::ChassisMode prepared_mode_=robot::ChassisMode::Disabled;
  uint32_t hold_start_=0,arm_start_=0;
  float world_yaw_=0,pitch_=0;
};
}
#endif
