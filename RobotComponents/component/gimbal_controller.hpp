#ifndef STEERED_GIMBAL_CONTROLLER_HPP_
#define STEERED_GIMBAL_CONTROLLER_HPP_
#include <cmath>
#include "module_state.hpp"
#include "pid.hpp"
#include "motor_DaMiao.hpp"

namespace robot {
// 两板使用相同的位置/速度串级双环，每个对象仅绑定本板负责的一轴。
class Gimbal {
 public:
  enum class Axis { Yaw, Pitch };
  struct Params {
    float max_torque_nm=0.0f;           // 力矩上限，默认不放行；不得超过驱动允许范围。
    float max_target_rate_rad_s=0.0f;   // 位置目标变化率，默认不放行。
    bool pitch_limits_confirmed=false; // 只有现场确定 pitch 范围后才设 true。
    float pitch_min_rad=0.0f;
    float pitch_max_rad=0.0f;
    float pitch_guard_rad=0.03f;       // 接近目标边界时逐渐限制向外的速度目标。
    float max_dt_s=0.010f;
    float max_speed_rad_s=0.0f;         // 外环输出上限，单位rad/s；不能用力矩上限替代。
    hello_world::pid::Pid::Params speed_pid{}; // 内环以电机速度反馈计算力矩(Nm)，默认停用。
    float actual_speed_limit_rad_s=0; // 非零时按真实反馈独立超速停机；追加字段保留旧聚合初始化顺序。
    bool pitch_position_limits_enabled=true; // 软件角度上下限开关；关闭后仍执行全部力矩/超速/输入许可检查。
    float velocity_ff_nm_per_rad_s=0; // 速度前馈：系数×内环目标角速度，单位Nm/(rad/s)。
    bool gravity_ff_enabled=false; // 无实测拟合数据时关闭，不能猜测重力补偿力矩。
    float gravity_cos_nm=0,gravity_sin_nm=0,gravity_bias_nm=0;
    float gravity_fit_min_rad=0,gravity_fit_max_rad=0; // 只限制前馈曲线求值区间，不限制运动角度。
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
    float target_speed_rad_s=0.0f,speed_error_rad_s=0.0f;
    float feedforward_nm=0,pid_torque_nm=0;
    bool feedforward_endpoint=false; // 实际角度在拟合区间外时采用端点补偿，避免外推。
    bool torque_limited=false; // PID与前馈请求之和被总力矩预算截断。
    float chassis_rate_correction_rad_s=0; // 加到底盘上的yaw电机关节速度目标的实际补偿量。
  };

  // 绑定本板一轴、PID 和达妙对象；仅初始化内存中的零输入，不调用使能或 CAN 发送。
  Gimbal(Axis axis,hello_world::motor::DaMiao *motor,hello_world::pid::Pid *pid,const Params &params)
      : axis_(axis),motor_(motor),pid_(pid) { configure(params); }

  // 保存并检查位置环限制，清积分与目标历史；启用角度限位时未知pitch范围保持停用。
  bool configure(const Params &params) {
    stop();params_=params;speed_pid_.configure(params.speed_pid);
    configured_=(axis_==Axis::Yaw || axis_==Axis::Pitch) &&
        std::isfinite(params.max_torque_nm) && params.max_torque_nm>=0 &&
        std::isfinite(params.max_target_rate_rad_s) && params.max_target_rate_rad_s>=0 &&
        std::isfinite(params.pitch_min_rad) && std::isfinite(params.pitch_max_rad) &&
        std::isfinite(params.pitch_guard_rad) && params.pitch_guard_rad>=0 &&
        std::isfinite(params.max_dt_s) && params.max_dt_s>0 &&
        std::isfinite(params.max_speed_rad_s) && params.max_speed_rad_s>=0 && speed_pid_.configured();
    configured_=configured_ && std::isfinite(params.actual_speed_limit_rad_s) && params.actual_speed_limit_rad_s>=0 &&
      (params.actual_speed_limit_rad_s==0 || params.actual_speed_limit_rad_s>params.max_speed_rad_s);
    configured_=configured_ && std::isfinite(params.velocity_ff_nm_per_rad_s) && std::isfinite(params.gravity_cos_nm) &&
      std::isfinite(params.gravity_sin_nm) && std::isfinite(params.gravity_bias_nm) && std::isfinite(params.gravity_fit_min_rad) &&
      std::isfinite(params.gravity_fit_max_rad) && (!params.gravity_ff_enabled || params.gravity_fit_min_rad<params.gravity_fit_max_rad);
    if(axis_==Axis::Pitch && params.pitch_position_limits_enabled && params.pitch_limits_confirmed) {
      configured_=configured_ && params.pitch_min_rad> -kPi && params.pitch_max_rad<kPi &&
          params.pitch_min_rad<params.pitch_max_rad &&
          params.pitch_guard_rad<(params.pitch_max_rad-params.pitch_min_rad)*0.5f;
    }
    return configured_;
  }

  // 检查动作条件并计算本地力矩；yaw可使用底盘实测角速度修正关节目标。
  bool update(const GimbalMotion &requested,const ModuleState &state,const Safety &safety,
      float dt_s=0.001f,float chassis_yaw_rate_rad_s=0) {
    if(!configured_ || motor_==nullptr || pid_==nullptr || !isValid(state) ||
        state.life!=LifeState::Working || state.source!=ControlSource::RemoteControl ||
        state.gimbal_mode!=GimbalMode::JointPosition || !safety.remote_valid ||
        !safety.command_valid || !safety.emergency_released || !safety.imu_ready ||
        !safety.feedback_valid || !safety.peer_online || !isFinite(requested) ||
        !std::isfinite(dt_s) || dt_s<=0 || dt_s>params_.max_dt_s ||
        params_.max_torque_nm<=0 || params_.max_target_rate_rad_s<=0 ||
        !motor_->is_enabled() || motor_->get_input_type()!=hello_world::motor::InputType::kTorq)
      return fail();
    if(axis_==Axis::Pitch && params_.pitch_position_limits_enabled && !params_.pitch_limits_confirmed)return fail();
    // 约束 PID 不超过本模块和驱动限幅，避免下游截断而上游积分继续堆积。
    if(!loopsReady())return fail();
    const float measured=motor_->angle();
    const float measured_speed=motor_->vel();
    if(!std::isfinite(measured) || !std::isfinite(measured_speed))return fail();
    if(params_.actual_speed_limit_rad_s>0 && std::fabs(measured_speed)>params_.actual_speed_limit_rad_s)return fail();
    const bool limit_pitch=axis_==Axis::Pitch && params_.pitch_position_limits_enabled;
    const bool beyond_max=limit_pitch && measured>params_.pitch_max_rad;
    const bool beyond_min=limit_pitch && measured<params_.pitch_min_rad;
    if(have_state_ && (state.chassis_mode!=previous_state_.chassis_mode ||
        state.gimbal_mode!=previous_state_.gimbal_mode)) {
      pid_->reset();speed_pid_.reset();have_target_=false;
    }
    Result next;
    float goal=axis_==Axis::Yaw?wrap(requested.yaw):requested.pitch;
    if(limit_pitch) {
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
    next.target_rad=axis_==Axis::Yaw?wrap(base+step):(params_.pitch_position_limits_enabled?clamp(base+step,params_.pitch_min_rad,params_.pitch_max_rad):base+step);
    next.error_rad=axis_==Axis::Yaw?wrap(next.target_rad-wrap(measured)):next.target_rad-measured;
    if(!std::isfinite(next.target_rad) || !std::isfinite(next.error_rad))return fail();
    // 外环将角度误差变为目标速度，内环必须减去实际电机速度，不能直接把外环当力矩。
    next.target_speed_rad_s=pid_->updateError(next.error_rad,dt_s);
    if(!pid_->lastUpdateOk())return fail();
    if(axis_==Axis::Yaw) {
      if(!std::isfinite(chassis_yaw_rate_rad_s))return fail();
      // 底盘与关节转速相加得到云台的对地转速。用本地IMU先抵消底盘转速，再让速度环追关节目标。
      const float compensated_speed=next.target_speed_rad_s-chassis_yaw_rate_rad_s;
      if(!std::isfinite(compensated_speed))return fail();
      const float joint_speed=clamp(compensated_speed,-params_.max_speed_rad_s,params_.max_speed_rad_s);
      next.chassis_rate_correction_rad_s=joint_speed-next.target_speed_rad_s;
      next.target_speed_rad_s=joint_speed;
      next.limited=next.limited || joint_speed!=compensated_speed;
    }
    // 保护带逐渐收紧向外速度目标；速度环仍可制动和施加静止保持力矩。
    if(limit_pitch && params_.pitch_guard_rad>0) {
      const float toward_min=params_.max_speed_rad_s*clamp((measured-params_.pitch_min_rad)/params_.pitch_guard_rad,0,1);
      const float toward_max=params_.max_speed_rad_s*clamp((params_.pitch_max_rad-measured)/params_.pitch_guard_rad,0,1);
      const float bounded_speed=clamp(next.target_speed_rad_s,-toward_min,toward_max);
      next.limited=next.limited || bounded_speed!=next.target_speed_rad_s;
      next.target_speed_rad_s=bounded_speed;
    }
    next.speed_error_rad_s=next.target_speed_rad_s-measured_speed;
    next.feedforward_nm=params_.velocity_ff_nm_per_rad_s*next.target_speed_rad_s;
    if(axis_==Axis::Pitch && params_.gravity_ff_enabled) {
      const float angle=clamp(measured,params_.gravity_fit_min_rad,params_.gravity_fit_max_rad);
      next.feedforward_endpoint=angle!=measured;
      next.feedforward_nm+=params_.gravity_cos_nm*std::cos(angle)+params_.gravity_sin_nm*std::sin(angle)+params_.gravity_bias_nm;
    }
    // 越界时滤掉向外的前馈分量，避免它抵消内环朝安全区返回的力矩。
    if((beyond_max && next.feedforward_nm>0) || (beyond_min && next.feedforward_nm<0))next.feedforward_nm=0;
    next.torque_nm=speed_pid_.updateErrorWithFeedforward(next.speed_error_rad_s,next.feedforward_nm,dt_s);
    next.pid_torque_nm=speed_pid_.feedbackOutput();
    if(!speed_pid_.lastUpdateOk() || !std::isfinite(next.torque_nm))return fail();
    next.torque_limited=std::fabs(next.pid_torque_nm+next.feedforward_nm)>params_.speed_pid.output_limit;
    // 只有实测越界才阻断合成的向外力矩，同时清除积累的外推积分。
    if((beyond_max && next.torque_nm>0) || (beyond_min && next.torque_nm<0)) {
      next.torque_nm=0;next.limited=true;pid_->reset();speed_pid_.reset();
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
    speed_pid_.reset();
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
  // 同时验证外环速度单位、内环力矩单位及驱动限幅，供运行时自检和真正更新共用。
  bool loopsReady() const {
    if(!configured_ || pid_==nullptr || motor_==nullptr)return false;
    const auto &p=pid_->params();const auto &s=speed_pid_.params();
    const float driver_limit=motor_->motor_info().torq_input_lim;
    return pid_->configured() && p.kp>=0 && p.ki>=0 && p.kd>=0 && p.output_limit>0 &&
        p.output_limit<=params_.max_speed_rad_s && params_.max_speed_rad_s>0 &&
        (axis_==Axis::Yaw?std::fabs(p.error_period-2*kPi)<0.00001f:p.error_period==0) &&
        speed_pid_.configured() && s.kp>=0 && s.ki>=0 && s.kd>=0 && s.error_period==0 &&
        s.output_limit>0 && s.output_limit<=params_.max_torque_nm &&
        std::isfinite(driver_limit) && driver_limit>0 && params_.max_torque_nm<=driver_limit;
  }

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
  hello_world::pid::Pid speed_pid_; // 两轴各自的内环历史，绝不共享积分。
  Params params_{};
  Result result_{};
  ModuleState previous_state_{};
  float previous_target_=0;
  bool configured_=false,have_target_=false,have_state_=false;
};
}
#endif
