#ifndef STEERED_GIMBAL_CONTROLLER_HPP_
#define STEERED_GIMBAL_CONTROLLER_HPP_
#include <cmath>
#include "module_state.hpp"
#include "pid.hpp"
#include "motor_DaMiao.hpp"

namespace robot {
// 两板使用同一位置环实现，但每个对象仅绑定本板负责的一轴，不重复发送控制命令。
class Gimbal {
 public:
  enum class Axis { Yaw, Pitch };
  struct Params {
    float max_torque_nm=0.0f;           // 力矩上限，默认不放行；不得超过驱动允许范围。
    float max_target_rate_rad_s=0.0f;   // 位置目标变化率，默认不放行。
    bool pitch_limits_confirmed=false; // 只有现场确定 pitch 范围后才设 true。
    float pitch_min_rad=0.0f;
    float pitch_max_rad=0.0f;
    float pitch_guard_rad=0.03f;       // 接近软件边界时阻止继续向外的力矩。
    float max_dt_s=0.010f;
  };
  struct Safety {
    bool remote_valid=false;       // 有效且未过期的 DBUS/板间遥控输入。
    bool command_valid=false;      // 本周期位置目标有效且未过期。
    bool emergency_released=false; // 非急停档。
    bool imu_ready=false;          // 本板 IMU 已校准、收敛且数据新鲜。
    bool feedback_valid=false;     // 本板该电机反馈有效且未离线。
    bool peer_online=false;        // 另一板在线；接入第八/九项时由真实通信状态生成。
  };
  struct Result {
    float target_rad=0.0f;  // 钳位并限速后的本轴目标。
    float error_rad=0.0f;   // yaw 为最短角差，pitch 为线性误差。
    float torque_nm=0.0f;   // 写入本地电机对象的力矩，不等于已发送 CAN 帧。
    bool limited=false;    // 目标或边界力矩被限制。
    bool valid=false;      // false 时不得沿用目标，实际输入已尽可能归零。
  };

  // 绑定本板一轴、PID 和达妙对象；仅初始化内存中的零输入，不调用使能或 CAN 发送。
  Gimbal(Axis axis,hello_world::motor::DaMiao *motor,hello_world::pid::Pid *pid,const Params &params)
      : axis_(axis),motor_(motor),pid_(pid) { configure(params); }

  // 保存并检查位置环限制，清积分与目标历史；未知 pitch 范围保持停用。
  bool configure(const Params &params) {
    stop();params_=params;
    configured_=(axis_==Axis::Yaw || axis_==Axis::Pitch) &&
        std::isfinite(params.max_torque_nm) && params.max_torque_nm>=0 &&
        std::isfinite(params.max_target_rate_rad_s) && params.max_target_rate_rad_s>=0 &&
        std::isfinite(params.pitch_min_rad) && std::isfinite(params.pitch_max_rad) &&
        std::isfinite(params.pitch_guard_rad) && params.pitch_guard_rad>=0 &&
        std::isfinite(params.max_dt_s) && params.max_dt_s>0;
    if(axis_==Axis::Pitch && params.pitch_limits_confirmed) {
      configured_=configured_ && params.pitch_min_rad> -kPi && params.pitch_max_rad<kPi &&
          params.pitch_min_rad<params.pitch_max_rad &&
          params.pitch_guard_rad<(params.pitch_max_rad-params.pitch_min_rad)*0.5f;
    }
    return configured_;
  }

  // 检查全部动作条件，读取本地关节反馈并计算力矩；只写本地 setInput，不发送也不使能。
  bool update(const GimbalMotion &requested,const ModuleState &state,const Safety &safety,float dt_s=0.001f) {
    if(!configured_ || motor_==nullptr || pid_==nullptr || !isValid(state) ||
        state.life!=LifeState::Working || state.source!=ControlSource::RemoteControl ||
        state.gimbal_mode!=GimbalMode::JointPosition || !safety.remote_valid ||
        !safety.command_valid || !safety.emergency_released || !safety.imu_ready ||
        !safety.feedback_valid || !safety.peer_online || !isFinite(requested) ||
        !std::isfinite(dt_s) || dt_s<=0 || dt_s>params_.max_dt_s ||
        params_.max_torque_nm<=0 || params_.max_target_rate_rad_s<=0 ||
        !motor_->is_enabled() || motor_->get_input_type()!=hello_world::motor::InputType::kTorq)
      return fail();
    if(axis_==Axis::Pitch && !params_.pitch_limits_confirmed)return fail();
    const auto &pc=pid_->params();
    const float driver_limit=motor_->motor_info().torq_input_lim;
    // 约束 PID 不超过本模块和驱动限幅，避免下游截断而上游积分继续堆积。
    if(!pid_->configured() || pc.kp<0 || pc.ki<0 || pc.kd<0 || pc.output_limit<=0 ||
        pc.output_limit>params_.max_torque_nm || !std::isfinite(driver_limit) ||
        driver_limit<=0 || params_.max_torque_nm>driver_limit ||
        (axis_==Axis::Yaw?std::fabs(pc.error_period-2*kPi)>0.00001f:pc.error_period!=0))return fail();
    const float measured=motor_->angle();
    if(!std::isfinite(measured))return fail();
    // pitch 反馈已越界时不自动拉回，直接撤销控制；需要检查实物/零点后再恢复。
    if(axis_==Axis::Pitch && (measured<params_.pitch_min_rad || measured>params_.pitch_max_rad))return fail();
    if(have_state_ && (state.chassis_mode!=previous_state_.chassis_mode ||
        state.gimbal_mode!=previous_state_.gimbal_mode)) {
      pid_->reset();have_target_=false;
    }
    Result next;
    float goal=axis_==Axis::Yaw?wrap(requested.yaw):requested.pitch;
    if(axis_==Axis::Pitch) {
      const float bounded=clamp(goal,params_.pitch_min_rad,params_.pitch_max_rad);
      next.limited=bounded!=goal;goal=bounded;
    }
    // 首次/恢复从实际位置起步；故障前的目标和积分不会带入恢复后的首周期。
    const float base=have_target_?previous_target_:(axis_==Axis::Yaw?wrap(measured):measured);
    const float change=axis_==Axis::Yaw?wrap(goal-base):goal-base;
    const float max_step=params_.max_target_rate_rad_s*dt_s;
    if(!std::isfinite(change) || !std::isfinite(max_step))return fail();
    const float step=clamp(change,-max_step,max_step);
    next.limited=next.limited || step!=change;
    next.target_rad=axis_==Axis::Yaw?wrap(base+step):clamp(base+step,params_.pitch_min_rad,params_.pitch_max_rad);
    next.error_rad=axis_==Axis::Yaw?wrap(next.target_rad-wrap(measured)):next.target_rad-measured;
    if(!std::isfinite(next.target_rad) || !std::isfinite(next.error_rad))return fail();
    next.torque_nm=pid_->updateError(next.error_rad,dt_s);
    if(!pid_->lastUpdateOk() || !std::isfinite(next.torque_nm))return fail();
    // 软件边界附近不得向外施力；清积分，避免目标仍在边界外时累积堵转力矩。
    if(axis_==Axis::Pitch &&
        ((measured>=params_.pitch_max_rad-params_.pitch_guard_rad && next.torque_nm>0) ||
         (measured<=params_.pitch_min_rad+params_.pitch_guard_rad && next.torque_nm<0))) {
      next.torque_nm=0;next.limited=true;pid_->reset();
    }
    next.torque_nm=clamp(next.torque_nm,-params_.max_torque_nm,params_.max_torque_nm);
    if(motor_->setInput(next.torque_nm)!=hello_world::motor::Status::kOk)return fail();
    next.valid=true;result_=next;previous_target_=next.target_rad;have_target_=true;
    previous_state_=state;have_state_=true;
    return true;
  }

  // 立即清除本轴 PID、目标历史及达妙输入；setInput(0) 同时清 MIT 位置/速度/Kp/Kd。
  void stop() {
    result_={};have_target_=false;have_state_=false;previous_target_=0;
    if(pid_!=nullptr)pid_->reset();
    if(motor_!=nullptr)motor_->setInput(0.0f);
  }
  // 返回当前轴标识，供调度决定使用 yaw 还是 pitch 的目标/反馈。
  Axis axis() const { return axis_; }
  // 返回本轴上次结果；失效后 valid=false，力矩为零。
  const Result &result() const { return result_; }
  // 返回当前限制参数，只读，不绕过配置验证。
  const Params &params() const { return params_; }
  // 查询限制参数是否数值合法；未知 pitch 限位或零动力上限依然不允许更新。
  bool configured() const { return configured_; }

 private:
  // 将有限角度归一化到 [-π,π)，用于 yaw 的目标和误差，不进行多圈累计。
  static float wrap(float angle) {
    float value=std::fmod(angle,2*kPi);
    if(value>=kPi)value-=2*kPi;
    if(value< -kPi)value+=2*kPi;
    return value;
  }
  // 将有限数值限制在给定区间，用于 pitch 目标、变化率和力矩。
  static float clamp(float value,float lower,float upper) {
    if(value<lower)return lower;
    if(value>upper)return upper;
    return value;
  }
  // 失效统一出口：归零本轴输入并清全部历史，不能沿用上周期控制量。
  bool fail() { stop();return false; }
  static constexpr float kPi=3.14159265358979323846f;
  Axis axis_;
  hello_world::motor::DaMiao *motor_=nullptr;
  hello_world::pid::Pid *pid_=nullptr;
  Params params_{};
  Result result_{};
  ModuleState previous_state_{};
  float previous_target_=0;
  bool configured_=false,have_target_=false,have_state_=false;
};
}
#endif
