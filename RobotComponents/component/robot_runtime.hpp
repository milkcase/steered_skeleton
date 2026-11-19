#ifndef STEERED_ROBOT_RUNTIME_HPP_
#define STEERED_ROBOT_RUNTIME_HPP_
#include "robot_supervisor.hpp"
#include "motor_feedback.hpp"
#include "gimbal_controller.hpp"
#include "imu.hpp"
#include "dbus_uart.hpp"
#include "ins_comm.hpp"
#include "robot_diagnostics.hpp"
#include "main_tuning_parameters.hpp"
#include "pitch_feedback_link.hpp"
namespace robot {
// 正常主程序首次停止的调用来源；仅诊断用途，不是可写的运行许可。
enum class MainStopSource : uint32_t {External=0,GimbalLoop=1,DriveLoop=2,ControlGate=3,ArmGate=4,
  AxisGate=5,Transport=6,GroupEncoding=7,DisableCommand=8,FeedbackLost=9,SupervisorGate=10};
// 底盘闭环端口：云台板不链接或调用底盘实例，仅底盘本地实现此接口。
class DrivePort {
 public:
  // 端口按派生类型析构，不操作硬件。
  virtual ~DrivePort()=default;
  // 检查配置及八台本地电机反馈，不生成运动。
  virtual bool ready(uint32_t now) const=0;
  // 运行逆解和八个 PID，任何失败由调用方统一归零。
  virtual bool update(const ChassisMotion &,float,const ModuleState &,float)=0;
  // 清输入和全部闭环历史。
  virtual void stop()=0;
};
// 两板复用的硬件编排；角色决定唯一负责 yaw 或 pitch，不允许跨板对象调用。
class RobotRuntime {
 public:
  // 只创建默认安全状态，不启动 CAN/SPI，不发送或使能电机。
  RobotRuntime(robot_control::Board board,const robot_control::Params &params)
      : board_(board),params_(params),supervisor_(board,params) {}
  // 注册本板 IMU；初始化单独在非中断阶段进行。
  void registerImu(hello_world::imu::Imu *imu) {imu_=imu;}
  // 注册本板独占关节、位置环和 PID；云台只能注册 pitch，底盘只能注册 yaw。
  void registerAxis(hello_world::motor::DaMiao *axis,Gimbal *gimbal,hello_world::pid::Pid *pid) {
    axis_=axis;gimbal_=gimbal;axis_pid_=pid;
  }
  // 注册本板通信对象，不接收另一固件的指针。
  void registerCommunication(GimbalChassisComm *comm) {comm_=comm;}
  // 发送层撤销硬件邮箱时同步放弃已编码的残页，避免下一次只发缺少前页的尾部。
  void restartCommunicationTx() {if(comm_!=nullptr)comm_->restartTx();}
  // 仅底盘注册轮组闭环，云台角色拒绝使用该指针。
  void registerDrive(DrivePort *drive) {if(board_==robot_control::Board::Chassis)drive_=drive;}
  // 保存 CAN 初始化结果，失败保持停用；不以 UART 或对象非空代替 CAN 可用。
  void setTransportReady(bool ready) {transport_ready_=ready;if(!ready)stop(MainStopSource::Transport);}
  // 已确认映射时才初始化 BMI088；候选 PA4/PB0 和单位矩阵不自动视为实物确认。
  bool initializeImu(const hello_world::imu::BMI088HWConfig &hardware,const float rotation[9],bool confirmed) {
    imu_started_=confirmed && imu_!=nullptr && imu_->init(hardware,rotation);
    return imu_started_;
  }
  // 在主循环进行有超时的 SPI 读取，不阻塞 TIM6；发布时只短暂关中断复制结果。
  void pollSensors() {
    if(!imu_started_)return;
    const uint32_t now=hello_world::tick::GetTickMs();
    if(sensor_polled_ && now-last_poll_ms_<1)return;
    if(sensor_polled_ && now-last_poll_ms_>max_poll_gap_ms_)max_poll_gap_ms_=now-last_poll_ms_;
    ++poll_count_;
    sensor_polled_=true;last_poll_ms_=now;
    if(main_tuning::enabled) {
      bool restart=false;
      {
        remote_input::InterruptLock lock;
        restart=imu_restart_requested_;
        imu_restart_requested_=false;
      }
      if(restart && imu_->restartAfterSampleFault())++imu_recalibration_count_;
    }
    const bool updated=imu_->update(),ready=imu_->isReady();
    const auto timing=imu_->timingDiagnostics();
    robot_runtime_health.imu_state=static_cast<uint32_t>(imu_->state());
    robot_runtime_health.imu_last_interval_us=timing.last_interval_us;
    robot_runtime_health.imu_max_interval_us=timing.max_interval_us;
    robot_runtime_health.imu_timing_fault=timing.interval_fault;
    robot_runtime_health.imu_recalibrations=imu_recalibration_count_;
    const float yaw=ready?imu_->yaw():0;
    const float yaw_rate=ready && main_tuning::enabled && board_==robot_control::Board::Chassis?imu_->bodyYawRate():0;
    remote_input::InterruptLock lock;
    imu_ready_=ready;
    if(updated && ready) {imu_yaw_=yaw;imu_yaw_rate_=yaw_rate;imu_sample_ms_=hello_world::tick::GetTickMs();++ready_updates_;}
  }
  // 按真实调度间隔处理最新输入、迁移、计算、统一归零和状态发布；不在此发送 CAN。
  void update(float dt_s=0.001f) {
    updateMainTuning();
    robot_control::ControlInput input;
    input.dt_s=dt_s;
    float chassis_yaw_rate=0;
    {
      remote_input::InterruptLock lock;
      input.now_ms=hello_world::tick::GetTickMs();
      input.imu_ready=imuFresh(input.now_ms);
      if(main_tuning::enabled && board_==robot_control::Board::Chassis && input.imu_ready) {
        chassis_yaw_rate=imu_yaw_rate_;input.chassis_yaw=imu_yaw_;
      }
      publishRuntimeHealth(input.now_ms,input.imu_ready);
      input.local_ready=localReady(input.now_ms);
      input.axis_valid=axisFresh(input.now_ms);
      input.axis_angle=input.axis_valid?axis_->angle():0;
      readPeer(input);
    }
    if(board_==robot_control::Board::Gimbal)input.remote=readRemote();
    else {
      // 遥控在线与运动指令许可分开：Dead 心跳也能传播遥控在线，避免两板启动互相等待。
      input.remote.valid=input.peer_valid && input.peer_flags.remote_valid;
      input.remote.released=input.peer_flags.emergency_released;
      input.remote.arm=input.peer_state.life==LifeState::Resurrection;
      input.remote.run=input.peer_state.life==LifeState::Working;
      input.remote.mode=input.peer_state.chassis_mode;
      input.remote.motion=peer_motion_;
    }
    // 诊断按真实遥控准备周期清资格；短暂收不到对端不冒充一次新的右下复位。
    if(main_tuning::enabled) {
      // 仅一份有效遥控的右下切换发出恢复请求；丢遥控不算切换，重校准由前台执行。
      if(input.remote.valid) {
        const bool down=!input.remote.released;
        if(down && !imu_right_was_down_)imu_restart_requested_=true;
        imu_right_was_down_=down;
      }
      if(!input.remote.valid || input.remote.released)imu_restart_requested_=false;
      if(input.remote.valid && !input.remote.released) {main_preparation_seen_=false;main_axis_permission_seen_=false;}
      else if(input.remote.arm && !main_preparation_seen_) {
        main_preparation_seen_=true;main_stop_recorded_=false;main_axis_permission_seen_=false;
      }
      if(main_axis_was_valid_ && !input.axis_valid)recordMainStop(MainStopSource::FeedbackLost);
      main_axis_was_valid_=input.axis_valid;
    }
    supervisor_.update(input);
    const auto &out=supervisor_.output();
    // 在状态机刚撤销许可时就保存本轮输入，早于发送层失能及下一帧电机反馈。
    if(out.axis_action==robot_control::AxisAction::Disable)
      recordMainStop(MainStopSource::SupervisorGate,&input);
    if(out.axis_action!=robot_control::AxisAction::Disable)main_axis_permission_seen_=true;
    if(!out.control)zeroInputs();
    else {
      GimbalMotion target=out.target;
      if(board_==robot_control::Board::Gimbal && gimbal_->params().pitch_position_limits_enabled) {
        const auto &p=gimbal_->params();
        if(target.pitch<p.pitch_min_rad)target.pitch=p.pitch_min_rad;
        if(target.pitch>p.pitch_max_rad)target.pitch=p.pitch_max_rad;
        supervisor_.acceptPitchTarget(target.pitch);
      }
      Gimbal::Safety safe{true,true,true,input.imu_ready,input.axis_valid,input.peer_valid};
      if(!gimbal_->update(target,out.state,safe,dt_s,chassis_yaw_rate))stop(MainStopSource::GimbalLoop);
      else if(board_==robot_control::Board::Chassis && !drive_->update(out.motion,input.axis_angle,out.state,dt_s))stop(MainStopSource::DriveLoop);
    }
    publish(input);
  }
  // 最终发帧前再次核对异步更新的急停与反馈；非控制阶段不借旧状态恢复非零输出。
  bool controlStillSafe(bool *mode_change=nullptr) {
    if(mode_change!=nullptr)*mode_change=false;
    if(!supervisor_.output().control)return false;
    remote_input::InterruptLock lock;
    const uint32_t now=hello_world::tick::GetTickMs();
    if(!imuFresh(now) || !localReady(now) || !axisFresh(now) || comm_==nullptr || !comm_->commandsUsable())return false;
    const auto mode=supervisor_.output().state.chassis_mode;
    bool changed=false;
    if(board_==robot_control::Board::Gimbal) {
      GimbalChassisComm::ChassisData data;
      if(!comm_->readPeer(data) || !data.flags.yaw_feedback_valid ||
          data.state.chassis_mode==ChassisMode::Disabled || data.state.gimbal_mode!=GimbalMode::JointPosition)return false;
      const auto remote=readRemote();
      if(!remote.valid || !remote.released || !remote.run || (remote.mode==ChassisMode::Spin && !params_.allow_spin))return false;
      changed=remote.mode!=mode || data.state.chassis_mode!=mode;
    } else {
      GimbalChassisComm::GimbalData data;
      if(!comm_->readPeer(data) || !data.flags.pitch_feedback_valid || data.state.chassis_mode==ChassisMode::Disabled ||
          data.state.gimbal_mode!=GimbalMode::JointPosition)return false;
      changed=data.state.chassis_mode!=mode;
    }
    if(mode_change!=nullptr)*mode_change=main_tuning::enabled && changed;
    return !changed;
  }
  // 发送和看门狗共用最终复查；仅健康模式不一致时保留准备许可，其余失效按保护停机。
  bool recheckActiveControl() {
    remote_input::InterruptLock lock;
    bool mode_change=false;
    if(controlStillSafe(&mode_change))return true;
    if(!mode_change) {stop(MainStopSource::ControlGate);return false;}
    supervisor_.pauseForModeChange();zeroInputs();
    if(board_==robot_control::Board::Gimbal)comm_->gimbal_data().motion={};
    return false;
  }
  // 最终发使能前重新确认窗口、遥控和对端自检；使能仅用于零输入反馈握手。
  bool armStillSafe() {
    if(supervisor_.output().axis_action!=robot_control::AxisAction::ArmZero)return false;
    remote_input::InterruptLock lock;
    const uint32_t now=hello_world::tick::GetTickMs();
    if(!localReady(now) || !imuFresh(now) || comm_==nullptr)return false;
    robot_control::ControlInput input;readPeer(input);
    if(!input.peer_valid || !input.peer_flags.ready || !input.peer_flags.remote_valid ||
        !input.peer_flags.emergency_released || input.peer_flags.fault)return false;
    if(board_==robot_control::Board::Gimbal) {
      const auto r=readRemote();return r.valid && r.released && ((r.arm && r.neutral) || r.run);
    }
    return input.peer_flags.command_valid &&
        (input.peer_state.life==LifeState::Resurrection || input.peer_state.life==LifeState::Working);
  }
  // 失效统一清状态与所有本板输入；通信层随后撤销旧邮箱并发送零帧/失能帧。
  void stop(MainStopSource source=MainStopSource::External) {
    recordMainStop(source);
    // 通用保护仍执行原停止动作；只补充首次失效记录，不覆盖先前复查已保存的现场。
    supervisor_.stop();zeroInputs();
    // 发送层发现异步失效后也立即撤销本板发布缓存，避免再发出本周期早先的 Working 指令。
    remote_input::InterruptLock lock;
    if(comm_!=nullptr) {
      if(board_==robot_control::Board::Chassis) {comm_->chassis_data()={};comm_->chassis_data().flags.fault=true;}
      else {comm_->gimbal_data()={};comm_->gimbal_data().flags.fault=true;}
    }
  }
  // 查询当前决策，供通信层选择纯力矩、零输入或失能帧。
  const robot_control::ControlOutput &output() const {return supervisor_.output();}
  // 在清状态/发送失能之前保存首个现场；不取诊断值控制电机，不修改任何安全条件。
  void recordMainStop(MainStopSource source,const robot_control::ControlInput *observed=nullptr) {
    if(!main_tuning::enabled || main_stop_recorded_ || !main_axis_permission_seen_)return;
    remote_input::InterruptLock lock;main_stop_recorded_=true;
    const uint32_t now=hello_world::tick::GetTickMs();robot_control::ControlInput in;
    if(observed!=nullptr)in=*observed;
    else {
      readPeer(in);
      if(board_==robot_control::Board::Gimbal)in.remote=readRemote();
      else {in.remote.valid=in.peer_valid && in.peer_flags.remote_valid;in.remote.released=in.peer_flags.emergency_released;
        in.remote.arm=in.peer_state.life==LifeState::Resurrection;in.remote.run=in.peer_state.life==LifeState::Working;}
    }
    const auto &o=supervisor_.output();auto &d=robot_main_stop;
    d.count=++main_stop_count_;d.time_ms=now;d.source=static_cast<uint32_t>(source);
    d.life=static_cast<uint32_t>(o.state.life);d.control=o.control;d.axis_action=static_cast<uint32_t>(o.axis_action);
    d.axis_enabled=axis_!=nullptr && axis_->is_enabled();d.axis_valid=axisFresh(now);
    d.axis_status=axis_!=nullptr?static_cast<uint32_t>(axis_->status_code()):0xffffffffu;
    d.peer_valid=in.peer_valid;d.peer_life=static_cast<uint32_t>(in.peer_state.life);
    d.peer_ready=in.peer_flags.ready;d.peer_imu=in.peer_flags.imu_ready;d.peer_command_valid=in.peer_flags.command_valid;d.peer_fault=in.peer_flags.fault;
    d.peer_axis_valid=board_==robot_control::Board::Chassis?in.peer_flags.pitch_feedback_valid:in.peer_flags.yaw_feedback_valid;
    d.commands_usable=comm_!=nullptr && comm_->commandsUsable();d.imu=imuFresh(now);d.parameters=mainTuningReady(now);
    d.prepared=supervisor_.prepared();d.prepare_failed=supervisor_.preparationFailed();
    d.remote_valid=in.remote.valid;d.remote_released=in.remote.released;d.remote_arm=in.remote.arm;d.remote_run=in.remote.run;
    d.remote_neutral=in.remote.neutral;d.dt_s=observed!=nullptr?observed->dt_s:0;
    d.axis_angle_rad=axis_!=nullptr?axis_->angle():0;d.axis_velocity_rad_s=axis_!=nullptr?axis_->vel():0;
    if(comm_!=nullptr) {
      // 读取失败会清空对端输出；因此另存接收拒绝事件与最后完整缓存，后者仅供诊断绝不授权。
      const auto rx=comm_->rxDiagnostics();d.peer_age_ms=comm_->peerAgeMs();d.rx_completed=rx.completed;
      d.crc_errors=rx.crc_errors;d.format_errors=rx.format_errors;d.order_errors=rx.order_errors;
      d.assembly_timeouts=rx.assembly_timeouts;d.replays=rx.replays;robot_diagnostics::publishRxEvent(d.rx_event,rx.last_rejection);
      const auto &flags=board_==robot_control::Board::Chassis?comm_->gimbal_data().flags:comm_->chassis_data().flags;
      const auto &state=board_==robot_control::Board::Chassis?comm_->gimbal_data().state:comm_->chassis_data().state;
      d.last_peer_life=static_cast<uint32_t>(state.life);d.last_peer_ready=flags.ready;d.last_peer_imu=flags.imu_ready;
      d.last_peer_axis_valid=board_==robot_control::Board::Chassis?flags.pitch_feedback_valid:flags.yaw_feedback_valid;
      d.last_peer_command_valid=flags.command_valid;d.last_peer_fault=flags.fault;
      d.last_peer_axis_rad=board_==robot_control::Board::Chassis?comm_->gimbal_data().feedback.pitch:comm_->chassis_data().feedback.yaw;
    }
  }
  // 返回本板关节对象，只供本板统一发送层使用。
  hello_world::motor::DaMiao *axis() const {return axis_;}
 protected:
  // 主程序启动参数同步；CAN只缓存完整组，实际换参限于有效遥控的右下急停档。
  void updateMainTuning() {
    if(!main_tuning::enabled)return;
    remote_input::InterruptLock lock;
    if(board_==robot_control::Board::Chassis) {
      if(!main_tuning::parameter_source.loaded()) {const auto bundle=main_tuning::parameters();
        main_tuning::parameter_source.load(bundle);publishMainTuning(bundle);}
    } else {
      hello_world::remote_control::DT7 rc;main_tuning::Bundle bundle;uint32_t hash=0;
      // 同组续租不需要再次读取DT7或重配PID；只有确有待应用整组时才检查右下急停。
      if(main_tuning::parameter_sink.pending(bundle,hash,hello_world::tick::GetTickMs()) &&
          ReadRemoteControl(rc) && rc.rc_r_switch()==hello_world::remote_control::kSwitchStateDown && gimbal_!=nullptr && axis_pid_!=nullptr) {
        supervisor_.stop();zeroInputs();
        const auto &v=bundle.value;auto control=params_;auto axis=gimbal_->params();auto position=axis_pid_->params();
        using namespace main_tuning;
        control.max_vx_m_s=v[Vx];control.max_vy_m_s=v[Vy];control.yaw_rate_rad_s=v[YawRate];
        control.pitch_rate_rad_s=v[PitchRate];control.aim_acceleration_rad_s2=v[AimAccel];
        position.kp=v[PositionP];position.ki=v[PositionI];position.kd=v[PositionD];
        position.integral_limit=v[PositionIntegral];position.output_limit=v[PositionSpeed];
        axis.speed_pid.kp=v[SpeedP];axis.speed_pid.ki=v[SpeedI];axis.speed_pid.kd=v[SpeedD];
        axis.speed_pid.integral_limit=v[SpeedIntegral];axis.speed_pid.output_limit=v[Torque];
        axis.max_torque_nm=v[Torque];axis.max_speed_rad_s=v[PositionSpeed];axis.max_target_rate_rad_s=v[TargetRate];
        axis.actual_speed_limit_rad_s=v[TripSpeed];axis.pitch_min_rad=v[PitchMin];axis.pitch_max_rad=v[PitchMax];axis.pitch_guard_rad=v[PitchGuard];
        axis.pitch_position_limits_enabled=v[PitchLimitEnabled]==1;
        axis.gravity_ff_enabled=v[GravityEnabled]==1;axis.gravity_cos_nm=v[GravityCos];axis.gravity_sin_nm=v[GravitySin];
        axis.gravity_bias_nm=v[GravityBias];axis.gravity_fit_min_rad=v[GravityMin];axis.gravity_fit_max_rad=v[GravityMax];
        if(supervisor_.configure(control) && axis_pid_->configure(position) && gimbal_->configure(axis) && gimbal_->loopsReady()) {
          params_=control;parameter_sink.applied(hash);++main_tuning_apply_count_;
          publishMainTuning(bundle);
        }
      }
    }
    const uint32_t now=hello_world::tick::GetTickMs();auto &d=robot_tuning_diagnostics;
    d.ready=mainTuningReady(now);d.hash=board_==robot_control::Board::Chassis?main_tuning::parameter_source.hash():main_tuning::parameter_sink.hash();
    d.applied_count=main_tuning_apply_count_;
  }
  // 主程序数值同步与电机控制双重门控，旧调试模式完全沿用原有独立授权。
  bool mainTuningReady(uint32_t now) const {
    return !main_tuning::enabled ||
      (board_==robot_control::Board::Chassis?main_tuning::parameter_source.ready(now):main_tuning::parameter_sink.ready(now));
  }
  // 发布底盘拟下发值/云台实际应用值；这些只读镜像不参与PID或安全许可计算。
  void publishMainTuning(const main_tuning::Bundle &bundle) {
    using namespace main_tuning;const auto &v=bundle.value;auto &d=robot_tuning_diagnostics;
    d.pitch_position_p=v[PositionP];d.pitch_speed_p=v[SpeedP];d.pitch_torque_nm=v[Torque];
    d.max_vx_m_s=v[Vx];d.yaw_rate_rad_s=v[YawRate];d.pitch_rate_rad_s=v[PitchRate];
    d.pitch_position_limits_enabled=v[PitchLimitEnabled]==1;
    d.pitch_gravity_ff_enabled=v[GravityEnabled]==1;d.gravity_cos_nm=v[GravityCos];d.gravity_sin_nm=v[GravitySin];
    d.gravity_bias_nm=v[GravityBias];d.gravity_fit_min_rad=v[GravityMin];d.gravity_fit_max_rad=v[GravityMax];
  }
  // 将 DT7 映射为有界遥控意图，Main只用四个摇杆通道检查回中。
  robot_control::RemoteIntent readRemote() {
    robot_control::RemoteIntent out;hello_world::remote_control::DT7 rc;
    if(!ReadRemoteControl(rc))return out;
    const float values[5]={rc.rc_lv(),rc.rc_lh(),rc.rc_rv(),rc.rc_rh(),main_tuning::enabled?0.0f:rc.rc_wheel()};
    out.neutral=true;
    for(float v:values) {
      if(!std::isfinite(v) || std::fabs(v)>1.001f)return {};
      if(std::fabs(v)>params_.deadzone)out.neutral=false;
    }
    using namespace hello_world::remote_control;
    out.valid=true;out.released=rc.rc_r_switch()!=kSwitchStateDown;
    out.arm=rc.rc_r_switch()==kSwitchStateMid;out.run=rc.rc_r_switch()==kSwitchStateUp;
    if(rc.rc_l_switch()==kSwitchStateUp)out.mode=ChassisMode::Separate;
    else if(rc.rc_l_switch()==kSwitchStateMid)out.mode=ChassisMode::Follow;
    else if(rc.rc_l_switch()==kSwitchStateDown)out.mode=ChassisMode::Spin;
    else return {};
    out.motion={channel(rc.rc_lv())*params_.max_vx_m_s,-channel(rc.rc_lh())*params_.max_vy_m_s,
                channel(values[4])*params_.max_w_rad_s};
    // Main的三种模式共用二维平移，云台侧w保持零，自转目标由底盘产生。
    if(main_tuning::enabled) {
      const float length=std::hypot(values[0],values[1]);
      if(length<=params_.deadzone)out.motion={};
      else {const float scale=length>1?1/length:1;out.motion={values[0]*scale*params_.max_vx_m_s,-values[1]*scale*params_.max_vy_m_s,0};}
    }
    out.yaw_rate=-channel(rc.rc_rh())*params_.yaw_rate_rad_s;
    // 按实车确认的操作方向，右杆前推上仰、后拉下俯。
    out.pitch_rate=channel(rc.rc_rv())*params_.pitch_rate_rad_s;
    return out;
  }
 private:
  // 使用死区保留中位静止，输出幅值仍不超过归一化通道范围。
  float channel(float v) const {return std::fabs(v)<=params_.deadzone?0:v;}
  // 检查已发布的完整 IMU 样本新鲜度；前台阻塞或校准未完成均不放行。
  bool imuFresh(uint32_t now) const {return imu_ready_ && now-imu_sample_ms_<5 && std::isfinite(imu_yaw_);}
  // 本板轴必须收到新鲜已使能状态；不采信另一板转述的旧反馈。
  bool axisFresh(uint32_t now) const {
    return axis_!=nullptr && robot_control::MotorFeedbacks().axisEnabled(now) && axis_->is_enabled() &&
        std::isfinite(axis_->angle()) && std::isfinite(axis_->vel());
  }
  // 本地自检不依赖对端 Working 或本轴已使能，避免零输入使能前等待反馈造成死锁。
  bool localReady(uint32_t now) const {
    if(!mainTuningReady(now) || !transport_ready_ || !supervisor_.configured() || axis_==nullptr || gimbal_==nullptr || axis_pid_==nullptr ||
        !gimbal_->configured() || !gimbal_->loopsReady() || !axis_pid_->configured() || axis_pid_->params().kp<0 ||
        axis_pid_->params().output_limit<=0 || axis_->get_input_type()!=hello_world::motor::InputType::kTorq ||
        axis_->current_ctrl_mode()!=hello_world::motor::DaMiao::CtrlMode::kMIT)return false;
    const auto &p=gimbal_->params();
    if(p.max_torque_nm<=0 || p.max_target_rate_rad_s<=0 || axis_pid_->params().output_limit>p.max_speed_rad_s ||
        p.max_torque_nm>axis_->motor_info().torq_input_lim ||
        (p.actual_speed_limit_rad_s>0 && (!std::isfinite(axis_->vel()) || std::fabs(axis_->vel())>p.actual_speed_limit_rad_s)))return false;
    if(board_==robot_control::Board::Chassis)return gimbal_->axis()==Gimbal::Axis::Yaw && drive_!=nullptr &&
        drive_->ready(now) && p.max_target_rate_rad_s>=params_.yaw_rate_rad_s+params_.max_w_rad_s;
    return gimbal_->axis()==Gimbal::Axis::Pitch && (!p.pitch_position_limits_enabled || p.pitch_limits_confirmed) && p.max_target_rate_rad_s>=params_.pitch_rate_rad_s;
  }
  // 读取本板方向的完整状态；恢复自检需要读取非 Working 安全状态，而非强求控制许可。
  void readPeer(robot_control::ControlInput &in) {
    if(comm_==nullptr)return;
    if(board_==robot_control::Board::Chassis) {
      GimbalChassisComm::GimbalData data;
      in.peer_valid=comm_->readPeer(data);in.peer_state=data.state;in.peer_flags=data.flags;
      in.peer_feedback=data.feedback;in.peer_yaw_target=data.yaw_target;peer_motion_=data.motion;
      in.peer_world_yaw_target_valid=in.peer_valid && data.world_yaw_target_valid;
      in.peer_world_yaw_target=data.world_yaw_target;
    } else {
      GimbalChassisComm::ChassisData data;
      in.peer_valid=comm_->readPeer(data);in.peer_state=data.state;in.peer_flags=data.flags;
      in.peer_feedback=data.feedback;in.peer_chassis_yaw=data.chassis_yaw;
    }
  }
  // 任一模块失效时归零所有本板控制输入，不以重置目标代替实际 setInput(0)。
  void zeroInputs() {
    if(gimbal_!=nullptr)gimbal_->stop();
    if(axis_!=nullptr)axis_->setInput(0);
    if(drive_!=nullptr)drive_->stop();
  }
  // 发布本板自检/遥控/轴反馈；转述轴有效位保持 false，不制造循环续命的反馈。
  void publish(const robot_control::ControlInput &in) {
    if(comm_==nullptr)return;
    remote_input::InterruptLock lock;
    const auto &o=output();
    auto &d=robot_main_diagnostics;d.board=board_==robot_control::Board::Chassis?0:1;
    d.life=static_cast<uint32_t>(o.state.life);d.control=o.control;d.ready=o.ready;
    d.chassis_mode=static_cast<uint32_t>(o.state.chassis_mode);
    d.follow_active=board_==robot_control::Board::Chassis && o.control && o.state.chassis_mode==ChassisMode::Follow;
    d.follow_error_rad=o.follow_error_rad;d.follow_pid_rpm=o.follow_pid_rate_rad_s/main_tuning::rpm;
    d.chassis_target_rpm=d.board==0 && o.control?o.motion.w/main_tuning::rpm:0;
    d.chassis_feedback_rpm=d.board==0 && in.imu_ready && std::isfinite(imu_yaw_rate_)?imu_yaw_rate_/main_tuning::rpm:0;
    d.chassis_heading_hold=o.chassis_heading_hold;
    d.chassis_heading_target_rad=o.chassis_heading_target_rad;
    d.chassis_heading_error_rad=o.chassis_heading_error_rad;
    d.chassis_heading_actual_rad=d.board==0 && in.imu_ready?SupervisorWrap(in.chassis_yaw):0;
    // 平地时对地yaw转速可用本地IMU与关节速度叠加观测，输出补偿单独记录。
    const bool yaw_observed=d.board==0 && in.axis_valid && in.imu_ready && std::isfinite(imu_yaw_rate_);
    d.yaw_world_feedback_rpm=yaw_observed?(axis_->vel()+imu_yaw_rate_)/main_tuning::rpm:0;
    d.yaw_world_angle_rad=yaw_observed?SupervisorWrap(in.axis_angle+in.chassis_yaw):0;
    d.yaw_world_target_valid=d.board==0 && o.control && o.world_yaw_target_valid;
    d.yaw_world_target_rad=d.yaw_world_target_valid?o.world_yaw_target:0;
    d.yaw_world_error_rad=d.yaw_world_target_valid && yaw_observed?SupervisorWrap(d.yaw_world_target_rad-d.yaw_world_angle_rad):0;
    d.yaw_chassis_compensation_rpm=d.board==0 && gimbal_!=nullptr?
        gimbal_->result().chassis_rate_correction_rad_s/main_tuning::rpm:0;
    d.imu=in.imu_ready;d.peer=in.peer_valid;d.axis_valid=in.axis_valid;
    d.axis_action=static_cast<uint32_t>(o.axis_action);d.axis_enabled=axis_!=nullptr && axis_->is_enabled();
    d.axis_status=axis_!=nullptr?static_cast<uint32_t>(axis_->status_code()):0xffffffffu;
    d.prepared=supervisor_.prepared();d.prepare_failed=supervisor_.preparationFailed();
    d.peer_life=static_cast<uint32_t>(in.peer_state.life);d.peer_ready=in.peer_flags.ready;d.peer_imu=in.peer_flags.imu_ready;
    d.peer_axis_valid=board_==robot_control::Board::Chassis?in.peer_flags.pitch_feedback_valid:in.peer_flags.yaw_feedback_valid;
    d.peer_command_valid=in.peer_flags.command_valid;d.peer_fault=in.peer_flags.fault;
    d.yaw_target_rad=o.target.yaw;d.pitch_target_rad=o.target.pitch;
    if(axis_!=nullptr) {d.axis_angle_rad=axis_->angle();d.axis_speed_rad_s=axis_->vel();}
    d.axis_feedback_torque_valid=in.axis_valid && axis_!=nullptr && std::isfinite(axis_->torq());
    d.axis_feedback_torque_nm=d.axis_feedback_torque_valid?axis_->torq():0;
    // 独立观测只能供显示/采样，不写回本地电机反馈或Supervisor输入。
    const uint32_t feedback_now=hello_world::tick::GetTickMs();
    auto pitch=pitch_feedback::channel.read(feedback_now);
    if(board_==robot_control::Board::Gimbal) {
      pitch={d.axis_angle_rad,d.axis_speed_rad_s,d.axis_feedback_torque_nm,d.axis_feedback_torque_valid!=0};
      if(gimbal_!=nullptr) {
        const auto &r=gimbal_->result();pitch.position_target=r.target_rad;pitch.speed_target=r.target_speed_rad_s;
        pitch.torque_command=r.torque_nm;pitch.feedforward=r.feedforward_nm;pitch.pid_torque=r.pid_torque_nm;
        pitch.position_error=r.error_rad;pitch.torque_limited=r.torque_limited;pitch.feedforward_endpoint=r.feedforward_endpoint;
      }
      pitch.control=o.control;pitch_feedback::channel.publish(pitch,feedback_now);
      d.pitch_feedback_age_ms=0;d.pitch_feedback_count=0;
    } else {
      pitch.valid=pitch.valid && in.peer_valid && in.peer_flags.pitch_feedback_valid;
      d.pitch_feedback_age_ms=pitch_feedback::channel.age(feedback_now);d.pitch_feedback_count=pitch_feedback::channel.count();
    }
    d.pitch_feedback_valid=pitch.valid;d.pitch_feedback_angle_rad=pitch.valid?pitch.angle:0;
    d.pitch_feedback_speed_rad_s=pitch.valid?pitch.speed:0;d.pitch_feedback_torque_nm=pitch.valid?pitch.torque:0;
    d.pitch_position_target_rad=pitch.valid?pitch.position_target:0;d.pitch_speed_target_rad_s=pitch.valid?pitch.speed_target:0;
    d.pitch_torque_nm=pitch.valid?pitch.torque_command:0;d.pitch_feedforward_nm=pitch.valid?pitch.feedforward:0;
    d.pitch_pid_torque_nm=pitch.valid?pitch.pid_torque:0;d.pitch_position_error_rad=pitch.valid?pitch.position_error:0;
    d.pitch_control=pitch.valid && pitch.control;d.pitch_torque_limited=pitch.valid && pitch.torque_limited;
    d.pitch_feedforward_endpoint=pitch.valid && pitch.feedforward_endpoint;
    if(gimbal_!=nullptr) {d.axis_position_target_rad=gimbal_->result().target_rad;d.axis_position_error_rad=gimbal_->result().error_rad;
      d.axis_speed_target_rad_s=gimbal_->result().target_speed_rad_s;d.torque_nm=gimbal_->result().torque_nm;}
    if(gimbal_!=nullptr) {d.feedforward_nm=gimbal_->result().feedforward_nm;d.pid_torque_nm=gimbal_->result().pid_torque_nm;
      d.feedforward_endpoint=gimbal_->result().feedforward_endpoint;d.torque_limited=gimbal_->result().torque_limited;}
    GimbalChassisComm::Flags flags;
    flags.ready=o.ready;flags.imu_ready=in.imu_ready;
    flags.remote_valid=o.remote_valid;flags.emergency_released=o.emergency_released;flags.command_valid=o.command_valid;
    const bool fresh=axisFresh(hello_world::tick::GetTickMs());
    if(board_==robot_control::Board::Chassis) {
      GimbalChassisComm::ChassisData data;data.state=o.state;data.flags=flags;
      data.flags.yaw_feedback_valid=fresh;data.feedback.yaw=fresh?SupervisorWrap(axis_->angle()):0;
      data.chassis_yaw=in.imu_ready?SupervisorWrap(imu_yaw_):0;
      data.target=o.target;comm_->chassis_data()=data;
    } else {
      GimbalChassisComm::GimbalData data;data.state=o.state;data.flags=flags;
      data.flags.pitch_feedback_valid=fresh;data.feedback.pitch=fresh?axis_->angle():0;
      data.yaw_target=o.control?o.target.yaw:(in.peer_flags.yaw_feedback_valid?in.peer_feedback.yaw:0);
      data.world_yaw_target_valid=o.control && o.world_yaw_target_valid;
      data.world_yaw_target=data.world_yaw_target_valid?o.world_yaw_target:0;
      data.motion=o.motion;comm_->gimbal_data()=data;
    }
  }
  // 使用共享状态机的单圈角度规则，不累积原始 IMU 圈数。
  static float SupervisorWrap(float a) {return robot_control::Supervisor::wrap(a);}
  uint32_t main_tuning_apply_count_=0;
  bool main_preparation_seen_=false,main_axis_permission_seen_=false,main_axis_was_valid_=false,main_stop_recorded_=false;
  uint32_t main_stop_count_=0;
  // 在原有新鲜度判断旁记录采样节奏和首次掉零现场，不修改5ms门槛或传感器就绪判据。
  void publishRuntimeHealth(uint32_t now,bool fresh) {
    auto &h=robot_runtime_health;h.board=board_==robot_control::Board::Chassis?0:1;
    h.poll_count=poll_count_;h.ready_updates=ready_updates_;h.imu_cache_ready=imu_ready_;
    h.imu_age_ms=now-imu_sample_ms_;h.max_poll_gap_ms=max_poll_gap_ms_;
    if(health_was_fresh_ && !fresh) {
      h.imu_drops=++imu_drop_count_;h.last_imu_drop_age_ms=now-imu_sample_ms_;
      h.last_imu_drop_cache_ready=imu_ready_;
    }
    health_was_fresh_=fresh;
  }
  robot_control::Board board_;robot_control::Params params_;robot_control::Supervisor supervisor_;
  hello_world::imu::Imu *imu_=nullptr;
  hello_world::motor::DaMiao *axis_=nullptr;Gimbal *gimbal_=nullptr;hello_world::pid::Pid *axis_pid_=nullptr;
  GimbalChassisComm *comm_=nullptr;DrivePort *drive_=nullptr;
  bool transport_ready_=false,imu_started_=false,imu_ready_=false,sensor_polled_=false;
  bool imu_right_was_down_=false,imu_restart_requested_=false;
  uint32_t imu_recalibration_count_=0;
  uint32_t poll_count_=0,ready_updates_=0,max_poll_gap_ms_=0,imu_drop_count_=0;
  bool health_was_fresh_=false;
  uint32_t imu_sample_ms_=0,last_poll_ms_=0;float imu_yaw_=0,imu_yaw_rate_=0;
  ChassisMotion peer_motion_{};
};
}
#endif
