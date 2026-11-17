#ifndef STEERED_ROBOT_RUNTIME_HPP_
#define STEERED_ROBOT_RUNTIME_HPP_
#include "robot_supervisor.hpp"
#include "motor_feedback.hpp"
#include "gimbal_controller.hpp"
#include "imu.hpp"
#include "dbus_uart.hpp"
#include "ins_comm.hpp"
#include "motor_debug_preview.hpp"
#include "motor_debug_power.hpp"
#include "main_tuning_parameters.hpp"
namespace robot {
// 底盘闭环端口：云台板不链接或调用底盘实例，仅底盘本地实现此接口。
class DrivePort {
 public:
  // 组合调试默认拒绝；只有底盘本地实现，云台不链接底盘逆解。
  virtual bool vectorUpdate(float,float,float,float,float &) {return false;}
  // 发送前检查四舵反馈、速度和类型。
  virtual bool vectorSafe(uint32_t) const {return false;}
  // 合并四舵帧，不操作其他驱动槽。
  virtual bool vectorFrame(uint8_t *) {return false;}
  // 撤销组合调试历史，默认无硬件操作。
  virtual void vectorStop() {}
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
      : board_(board),params_(params),supervisor_(board,params),debug_preview_(robot_debug::kPreviewEnabled,board==robot_control::Board::Chassis),
        debug_power_(robot_debug::kPowerEnabled,board==robot_control::Board::Chassis),
        extra_debug_{robot_debug::Power(robot_debug::direction::kFour,board==robot_control::Board::Chassis),
          robot_debug::Power(robot_debug::direction::kFour,board==robot_control::Board::Chassis),
          robot_debug::Power(robot_debug::direction::kFour,board==robot_control::Board::Chassis)} {}
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
  // 仅注册编译模式指定的左前电机：转向版为GM6020，驱动版为M3508，不能从Watch动态换对象。
  // 只有最终发送层重查独立许可后才可写入受限输入。
  void registerDebugTarget(hello_world::motor::Motor *motor) {
    if(board_==robot_control::Board::Chassis)debug_target_=motor;
  }
  // 注册本板四个转向对象；四舵模式每台独立反馈和PID，禁止把左前输出直接复制到其他电机。
  void registerDebugSteers(hello_world::motor::Motor *const motors[4]) {
    if(board_!=robot_control::Board::Chassis)return;
    for(int i=0;i<4;++i)debug_steers_[i]=motors[i];
  }
  // 保存 CAN 初始化结果，失败保持停用；不以 UART 或对象非空代替 CAN 可用。
  void setTransportReady(bool ready) {transport_ready_=ready;if(!ready)stop();}
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
    const bool updated=imu_->update(),ready=imu_->isReady();
    const auto timing=imu_->timingDiagnostics();
    robot_runtime_health.imu_state=static_cast<uint32_t>(imu_->state());
    robot_runtime_health.imu_last_interval_us=timing.last_interval_us;
    robot_runtime_health.imu_max_interval_us=timing.max_interval_us;
    robot_runtime_health.imu_timing_fault=timing.interval_fault;
    const float yaw=ready?imu_->yaw():0;
    remote_input::InterruptLock lock;
    imu_ready_=ready;
    if(updated && ready) {imu_yaw_=yaw;imu_sample_ms_=hello_world::tick::GetTickMs();++ready_updates_;}
  }
  // 按真实调度间隔处理最新输入、迁移、计算、统一归零和状态发布；不在此发送 CAN。
  void update(float dt_s=0.001f) {
    updateMainTuning();
    robot_control::ControlInput input;
    input.dt_s=dt_s;
    debug_dt_s_=dt_s;debug_control_ms_=hello_world::tick::GetTickMs();
    {
      remote_input::InterruptLock lock;
      input.now_ms=hello_world::tick::GetTickMs();
      input.imu_ready=imuFresh(input.now_ms);
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
    if(robot_debug::kDebugEnabled) {
      // 预演独立于整车Working，不伪造原有ready；先清全部真实输入，再发布零输出快照。
      forceZeroForPreview();updateDebugPreview(input);publish(input);return;
    }
    supervisor_.update(input);
    const auto &out=supervisor_.output();
    if(!out.control)zeroInputs();
    else {
      GimbalMotion target=out.target;
      if(board_==robot_control::Board::Gimbal) {
        const auto &p=gimbal_->params();
        if(target.pitch<p.pitch_min_rad)target.pitch=p.pitch_min_rad;
        if(target.pitch>p.pitch_max_rad)target.pitch=p.pitch_max_rad;
        supervisor_.acceptPitchTarget(target.pitch);
      }
      Gimbal::Safety safe{true,true,true,input.imu_ready,input.axis_valid,input.peer_valid};
      if(!gimbal_->update(target,out.state,safe,dt_s) ||
          (board_==robot_control::Board::Chassis && !drive_->update(out.motion,input.axis_angle,out.state,dt_s)))stop();
    }
    publish(input);
    updateDebugPreview(input);
  }
  // 编译期模式只表示零输出预演，发送层据此强制禁用非零帧及两轴使能。
  bool debugPreviewEnabled() const {return robot_debug::kPreviewEnabled;}
  // 独立动力构建标记，不从诊断镜像或普通预演标志推断动力许可。
  bool debugPowerEnabled() const {return robot_debug::kPowerEnabled;}
  // 返回固定调试目标供发送层校验注册指针一致，不允许动态选择其他电机。
  hello_world::motor::Motor *debugTarget() const {return debug_target_;}
  // 供预演与最终发送层共同清真实输入；不清通信快照，避免破坏预演心跳。
  void forceZeroForPreview() {supervisor_.stop();zeroInputs();}
  // 最终发帧前再次核对异步更新的急停与反馈；非控制阶段不借旧状态恢复非零输出。
  bool controlStillSafe() {
    // 看门狗也调用此入口；动力调试必须走同一套真实会话复查，而不是被误判为普通控制。
    if(robot_debug::kPowerEnabled)return debugPowerStillSafe();
    if(robot_debug::kDebugEnabled || !supervisor_.output().control)return false;
    remote_input::InterruptLock lock;
    const uint32_t now=hello_world::tick::GetTickMs();
    if(!imuFresh(now) || !localReady(now) || !axisFresh(now) || comm_==nullptr || !comm_->commandsUsable())return false;
    if(board_==robot_control::Board::Gimbal) {
      GimbalChassisComm::ChassisData data;
      if(!comm_->readPeer(data) || !data.flags.yaw_feedback_valid ||
          data.state.chassis_mode!=supervisor_.output().state.chassis_mode || data.state.gimbal_mode!=GimbalMode::JointPosition)return false;
      const auto remote=readRemote();
      return remote.valid && remote.released && remote.run && remote.mode==supervisor_.output().state.chassis_mode;
    }
    GimbalChassisComm::GimbalData data;
    return comm_->readPeer(data) && data.flags.pitch_feedback_valid &&
        data.state.chassis_mode==supervisor_.output().state.chassis_mode && data.state.gimbal_mode==GimbalMode::JointPosition;
  }
  // 最终发使能前重新确认窗口、遥控和对端自检；使能仅用于零输入反馈握手。
  bool armStillSafe() {
    if(robot_debug::kDebugEnabled || supervisor_.output().axis_action!=robot_control::AxisAction::ArmZero)return false;
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
  void stop() {
    if(robot_debug::drive::kVector && drive_!=nullptr)drive_->vectorStop();
    // 通用保护仍执行原停止动作；只补充首次失效记录，不覆盖先前复查已保存的现场。
    if(robot_debug::kPowerEnabled) {
      if(debug_power_.output().control)debug_power_.recordStopRequest();
      debug_power_.stop(robot_debug::Reason::PeerConsent);
      if(robot_debug::direction::kFour)for(auto &p:extra_debug_)p.stop(robot_debug::Reason::PeerConsent);
    }
    supervisor_.stop();zeroInputs();publishFourSteers();
    // 发送层发现异步失效后也立即撤销本板发布缓存，避免再发出本周期早先的 Working 指令。
    remote_input::InterruptLock lock;
    if(comm_!=nullptr) {
      if(board_==robot_control::Board::Chassis) {comm_->chassis_data()={};comm_->chassis_data().flags.fault=true;}
      else {comm_->gimbal_data()={};comm_->gimbal_data().flags.fault=true;}
    }
  }
  // 查询当前决策，供通信层选择纯力矩、零输入或失能帧。
  const robot_control::ControlOutput &output() const {return robot_debug::kPowerEnabled?debug_power_.output():supervisor_.output();}
  // 停止动力调试并发布撤销会话，避免发送层沿用上一份Working授权快照。
  void stopDebugPower(robot_debug::Reason reason) {
    if(robot_debug::drive::kVector && drive_!=nullptr)drive_->vectorStop();
    debug_power_.stop(reason);
    if(robot_debug::direction::kFour)for(auto &p:extra_debug_)p.stop(reason);
    zeroInputs();publishFourSteers();
    if(comm_!=nullptr) {
      if(board_==robot_control::Board::Chassis) {
        auto &d=comm_->chassis_data();d.state={};d.flags.ready=d.flags.command_valid=false;d.session=debug_power_.session();
      } else {
        auto &d=comm_->gimbal_data();d.state={};d.flags.ready=d.flags.command_valid=false;d.session=debug_power_.session();
      }
    }
    robot_debug::publish(debug_power_.diagnostics());
  }
  // 发送前再次采集真实状态、完整会话及截止时间；失败锁止，不在条件恢复时自动重新输出。
  bool debugPowerStillSafe() {
    if(!robot_debug::kPowerEnabled || board_!=robot_control::Board::Chassis || !debug_power_.output().control)return false;
    robot_control::ControlInput in;in.now_ms=hello_world::tick::GetTickMs();in.dt_s=debug_dt_s_;
    in.imu_ready=imuFresh(in.now_ms);robot_debug::Session session;
    const auto sample=collectDebugInput(in,session);
    const uint32_t age=in.now_ms-debug_control_ms_;
    uint32_t mask=debug_power_.safetyFailureMask(sample,session);
    if(robot_debug::drive::kVector && (drive_==nullptr || !drive_->vectorSafe(hello_world::tick::GetTickMs())))mask|=robot_debug::CheckFeedback;
    if(mask!=0)recordFourSteerFault(0,2,mask,&sample,&session);
    if(robot_debug::direction::kFour)for(int i=1;i<4;++i) {
      robot_debug::Session peer;const auto other=collectDebugInput(in,peer,i);
      const uint32_t other_mask=extra_debug_[i-1].safetyFailureMask(other,peer);
      if(other_mask!=0)recordFourSteerFault(i,2,other_mask,&other,&peer);
      mask|=other_mask;
    }
    if(age>3)mask|=robot_debug::CheckControlAge;
    if(mask==0)return true;
    if(age>3)recordFourSteerFault(0,2,robot_debug::CheckControlAge,&sample,&session);
    debug_power_.recordStopCheck(sample,session,mask,2,age);
    if(age>3) {stopDebugPower(robot_debug::Reason::Clock);return false;}
    // 直接使用刚完成的真实截止检查，不为取起始时间再复制整份诊断记录。
    stopDebugPower(debug_power_.failureReason(sample,mask));
    return false;
  }
  // 按模式编码：四舵合并ID1～4，其余模式仅ID1；GM用A，C620用原始值。
  bool fillDebugTargetFrame(uint8_t data[8]) {
    for(int i=0;i<8;++i)data[i]=0;
    if(!debugPowerStillSafe())return false;
    if(robot_debug::direction::kFour) {
      // 最终许可检查覆盖四台后才合并四槽；任一编码失败则整帧清零，并停止全部闭环。
      for(int i=0;i<4;++i) {
        auto *m=debug_steers_[i];const float value=powerAt(i).command();
        if(m==nullptr || m->motor_info().id!=i+1 || m->get_input_type()!=hello_world::motor::InputType::kCurr ||
            !std::isfinite(value) || std::fabs(value)>robot_debug::mode::kCurrentA ||
            m->setInput(value)!=hello_world::motor::Status::kOk || !m->encode(data)) {
          stopDebugPower(robot_debug::Reason::Configuration);for(int j=0;j<8;++j)data[j]=0;return false;
        }
      }
      for(int i=0;i<4;++i) {
        const int bits=(data[2*i]<<8)|data[2*i+1],raw=bits>=32768?bits-65536:bits;
        if(std::abs(raw)>robot_debug::mode::kRawLimit) {
          stopDebugPower(robot_debug::Reason::Configuration);for(int j=0;j<8;++j)data[j]=0;return false;
        }
      }
      return true;
    }
    const float value=debug_power_.command();
    if(debug_target_==nullptr || debug_target_->motor_info().id!=1 ||
        debug_target_->get_input_type()!=(robot_debug::drive::kEnabled?hello_world::motor::InputType::kRaw:hello_world::motor::InputType::kCurr) ||
        !std::isfinite(value) || std::fabs(value)>robot_debug::mode::kCurrentA ||
        debug_target_->setInput(robot_debug::drive::kEnabled?value*robot_debug::drive::kRawPerAmp:value)!=hello_world::motor::Status::kOk || !debug_target_->encode(data)) {
      stopDebugPower(robot_debug::Reason::Configuration);for(int i=0;i<8;++i)data[i]=0;return false;
    }
    for(int i=2;i<8;++i)if(data[i]!=0) {
      stopDebugPower(robot_debug::Reason::Configuration);for(int j=0;j<8;++j)data[j]=0;return false;
    }
    const int encoded=(data[0]<<8)|data[1];const int signed_raw=encoded>=32768?encoded-65536:encoded;
    // 电流协议±16384对应±3A；0.05A向零量化最多273，不能沿用原电压原始输入上限。
    // 可调版按编译时同一档电流计算最终整数上限，仍与私有控制器限流独立复查。
    const int raw_limit=robot_debug::mode::kRawLimit;
    if(signed_raw>raw_limit || signed_raw< -raw_limit) {
      stopDebugPower(robot_debug::Reason::Configuration);for(int i=0;i<8;++i)data[i]=0;return false;
    }
    return true;
  }
  // 返回本板关节对象，只供本板统一发送层使用。
  hello_world::motor::DaMiao *axis() const {return axis_;}
  // 仅在Working与真实会话均通过复查时编码四舵，失败立即撤销全组。
  bool fillVectorSteerFrame(uint8_t data[8]) {
    if(!debugPowerStillSafe()) {
      for(int i=0;i<8;++i)data[i]=0;
      return false;
    }
    if(drive_==nullptr || !drive_->vectorFrame(data)) {
      for(int i=0;i<8;++i)data[i]=0;
      stopDebugPower(robot_debug::Reason::Feedback);return false;
    }
    return true;
  }
 protected:
  // 主程序启动参数同步；CAN只缓存完整组，实际换参限于有效遥控的右下急停档。
  void updateMainTuning() {
    if(!main_tuning::enabled || robot_debug::kDebugEnabled)return;
    remote_input::InterruptLock lock;
    if(board_==robot_control::Board::Chassis) {
      if(!main_tuning::parameter_source.loaded()) {const auto bundle=main_tuning::parameters();
        main_tuning::parameter_source.load(bundle);publishMainTuning(bundle);}
    } else {
      hello_world::remote_control::DT7 rc;main_tuning::Bundle bundle;uint32_t hash=0;
      if(ReadRemoteControl(rc) && rc.rc_r_switch()==hello_world::remote_control::kSwitchStateDown &&
          main_tuning::parameter_sink.pending(bundle,hash,hello_world::tick::GetTickMs()) && gimbal_!=nullptr && axis_pid_!=nullptr) {
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
    return !main_tuning::enabled || robot_debug::kDebugEnabled ||
      (board_==robot_control::Board::Chassis?main_tuning::parameter_source.ready(now):main_tuning::parameter_sink.ready(now));
  }
  // 发布底盘拟下发值/云台实际应用值；这些只读镜像不参与PID或安全许可计算。
  void publishMainTuning(const main_tuning::Bundle &bundle) {
    using namespace main_tuning;const auto &v=bundle.value;auto &d=robot_tuning_diagnostics;
    d.pitch_position_p=v[PositionP];d.pitch_speed_p=v[SpeedP];d.pitch_torque_nm=v[Torque];
    d.max_vx_m_s=v[Vx];d.yaw_rate_rad_s=v[YawRate];d.pitch_rate_rad_s=v[PitchRate];
  }
  // 将 DT7 五通道映射为有界遥控意图；水平正值按候选配置映射为向右，现场核对。
  robot_control::RemoteIntent readRemote() {
    robot_control::RemoteIntent out;hello_world::remote_control::DT7 rc;
    if(!ReadRemoteControl(rc))return out;
    const float values[5]={rc.rc_lv(),rc.rc_lh(),rc.rc_rv(),rc.rc_rh(),rc.rc_wheel()};
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
                channel(rc.rc_wheel())*params_.max_w_rad_s};
    if(params_.separate_only) {
      const float length=std::hypot(values[0],values[1]);
      if(length<=params_.deadzone)out.motion={};
      else {const float scale=length>1?1/length:1;out.motion={values[0]*scale*params_.max_vx_m_s,-values[1]*scale*params_.max_vy_m_s,0};}
    }
    out.yaw_rate=-channel(rc.rc_rh())*params_.yaw_rate_rad_s;
    // 本车已确认抬枪pitch为负；分离主程序前推右杆应使枪口上抬。
    out.pitch_rate=(params_.separate_only?-1.0f:1.0f)*channel(rc.rc_rv())*params_.pitch_rate_rad_s;
    return out;
  }
 private:
  // 读取云台真实DT7快照，检查全部通道和拨杆；预演不借用零速度上限生成的运动量。
  // 可选debug_sample只记录同一次读取的五通道原值，定位轻推横杆时其他通道是否越过原死区。
  GimbalChassisComm::GimbalData::PreviewRemote previewRemote(robot_debug::Input *debug_sample=nullptr) {
    GimbalChassisComm::GimbalData::PreviewRemote out;
    hello_world::remote_control::DT7 rc;
    if(!ReadRemoteControl(rc))return out;
    const float values[5]={rc.rc_lv(),rc.rc_lh(),rc.rc_rv(),rc.rc_rh(),rc.rc_wheel()};
    if(debug_sample!=nullptr) {
      debug_sample->channels_valid=true;
      for(int i=0;i<5;++i)debug_sample->channels[i]=values[i];
    }
    out.neutral=out.other_neutral=true;
    for(int i=0;i<5;++i) {
      if(!std::isfinite(values[i]) || std::fabs(values[i])>1.001f)return {};
      const bool controlled=robot_debug::drive::kVector?(i==0 || i==1):(robot_debug::drive::kEnabled?i==0:(i==1 || (robot_debug::direction::kEnabled && i==0)));
      if(std::fabs(values[i])>0.05f) {out.neutral=false;if(!controlled)out.other_neutral=false;}
    }
    out.right=static_cast<uint8_t>(rc.rc_r_switch());out.left=static_cast<uint8_t>(rc.rc_l_switch());
    if((out.right!=1 && out.right!=2 && out.right!=3) || (out.left!=1 && out.left!=2 && out.left!=3))return {};
    if(robot_debug::drive::kVector) {
      const float length=std::hypot(values[0],values[1]);
      const bool active=length>0.05f;
      out.steer=active?std::atan2(-values[1],values[0])/robot_debug::direction::kPi:0;
      out.magnitude=active?(length>1?1:length):0;out.neutral=!active && out.other_neutral;
    } else if(robot_debug::direction::kEnabled) {
      const auto solved=robot_debug::direction::solve(values[0],values[1]);
      if(!solved.valid)return {};
      out.neutral=!solved.active && out.other_neutral;
      out.steer=solved.active?solved.angle/robot_debug::direction::kPi:0;
    } else if(robot_debug::drive::kEnabled)out.steer=std::fabs(values[0])>0.05f?values[0]:0;
    else out.steer=rc.rc_lh();
    out.valid=true;return out;
  }
  // 将真实遥控、完整对端快照和本地被动反馈送入纯内存预演器，发布全局诊断。
  robot_debug::Input collectDebugInput(const robot_control::ControlInput &input,robot_debug::Session &peer_session,int index=0) {
    auto *target=robot_debug::direction::kFour?debug_steers_[index]:debug_target_;
    robot_debug::Input sample;sample.now_ms=input.now_ms;sample.dt_s=input.dt_s;
    sample.transport=transport_ready_;sample.imu=input.imu_ready;
    GimbalChassisComm::GimbalData::PreviewRemote remote;
    {
      remote_input::InterruptLock lock;
      bool received=false;GimbalChassisComm::Flags peer_flags;
      // CAN可在控制步骤之间抢占并更新反馈时间；在同一短临界区取当前时间，避免旧now减新时间下溢。
      const uint32_t feedback_now=hello_world::tick::GetTickMs();
      if(board_==robot_control::Board::Chassis) {
        GimbalChassisComm::GimbalData data;
        received=comm_!=nullptr && comm_->readPeer(data);peer_flags=data.flags;
        peer_session=data.session;
        sample.peer=received && (!robot_debug::kDebugEnabled ||
            (data.flags.debug_preview && data.flags.debug_power==robot_debug::kPowerEnabled && data.flags.imu_ready && data.flags.pitch_feedback_valid && !data.flags.fault));
        if(received && data.flags.debug_preview)remote=data.preview;
        sample.feedback=target!=nullptr && target->motor_info().id==(robot_debug::direction::kFour?index+1:1) &&
            target->get_input_type()==(robot_debug::drive::kEnabled?hello_world::motor::InputType::kRaw:hello_world::motor::InputType::kCurr);
        for(int i=0;i<9;++i)sample.feedback=robot_control::MotorFeedbacks().fresh(i,feedback_now) && sample.feedback;
        if(robot_debug::kPowerEnabled && target!=nullptr)
          sample.feedback=sample.feedback && target->motor_info().curr_input_lim>=
              robot_debug::mode::kCurrentA;
        if(robot_debug::drive::kEnabled && debug_target_!=nullptr) {
          // 连续轮轴角用于位移保护，约19.2减速比保证速度单位已经是轮轴rad/s。
          sample.feedback=sample.feedback && debug_target_->angle_range()==hello_world::motor::AngleRange::kNegInfToPosInf &&
              std::fabs(debug_target_->motor_info().redu_rat-3591.0f/187.0f)<0.001f &&
              debug_target_->motor_info().raw_input_lim>=robot_debug::drive::kRawLimit;
        }
        if(robot_debug::direction::kEnabled || robot_debug::drive::kVector) {
          sample.feedback=sample.feedback && axis_!=nullptr && passiveAxisFresh(feedback_now);
          sample.yaw_delta=axis_!=nullptr?axis_->angle():0;
        }
        if(target!=nullptr) {
          sample.angle=target->angle();
          // 驱动首轮用未滤波的真实轮轴速度，速度保护不被滤波延迟掩盖；并未再除减速比。
          sample.velocity=robot_debug::drive::kEnabled?target->vel_raw():target->vel();
          // 使用提供驱动从反馈帧解码的curr()，绝不由setInput或命令反推实际电流。
          sample.feedback_current_a=target->curr();
          sample.feedback_current_valid=sample.feedback && std::isfinite(sample.feedback_current_a);
        }
      } else {
        remote=previewRemote(&sample);debug_remote_=remote;GimbalChassisComm::ChassisData data;
        received=comm_!=nullptr && comm_->readPeer(data);peer_flags=data.flags;
        peer_session=data.session;
        sample.peer=received && (!robot_debug::kDebugEnabled ||
            (data.flags.debug_preview && data.flags.debug_power==robot_debug::kPowerEnabled && data.flags.imu_ready && data.flags.yaw_feedback_valid && !data.flags.fault));
        sample.feedback=passiveAxisFresh(feedback_now);
      }
      sample.link.snapshot_valid=received;sample.link.preview_mode=peer_flags.debug_preview;
      sample.link.peer_imu=peer_flags.imu_ready;sample.link.peer_fault=peer_flags.fault;
      sample.link.peer_axis=board_==robot_control::Board::Chassis?peer_flags.pitch_feedback_valid:peer_flags.yaw_feedback_valid;
      sample.link.remote_in_snapshot=peer_flags.remote_valid;
      if(comm_!=nullptr) {
        sample.link.age_ms=comm_->peerAgeMs();const auto rx=comm_->rxDiagnostics();
        sample.link.complete_count=rx.completed;sample.link.crc_errors=rx.crc_errors;
        sample.link.format_errors=rx.format_errors;sample.link.order_errors=rx.order_errors;
        sample.link.assembly_timeouts=rx.assembly_timeouts;sample.link.replays=rx.replays;
        sample.link.rx_event=rx.last_rejection;
      }
    }
    sample.remote=remote.valid;sample.right=remote.right;sample.left=remote.left;
    sample.neutral=remote.neutral;sample.other_neutral=remote.other_neutral;sample.steer=remote.steer;
    sample.magnitude=remote.magnitude;
    return sample;
  }
  // 先计算本板专用状态，再发布诊断；动力值只在私有控制器内生成，预演仍固定零输出。
  // 四轮共用同周期遥控/对端会话，仅分别取得本轮反馈；避免CAN分页抢占使四份状态机看到不同准备阶段。
  robot_debug::Input collectSteerInput(const robot_debug::Input &common,int index) {
    auto sample=common;remote_input::InterruptLock lock;
    const uint32_t now=hello_world::tick::GetTickMs();auto *m=debug_steers_[index];
    sample.feedback=sample.feedback && m!=nullptr && m->motor_info().id==index+1 &&
        m->get_input_type()==hello_world::motor::InputType::kCurr &&
        m->motor_info().curr_input_lim>=robot_debug::mode::kCurrentA;
    for(int i=0;i<9;++i)sample.feedback=robot_control::MotorFeedbacks().fresh(i,now) && sample.feedback;
    sample.feedback=sample.feedback && passiveAxisFresh(now);
    if(m!=nullptr) {sample.angle=m->angle();sample.velocity=m->vel();sample.feedback_current_a=m->curr();}
    sample.feedback_current_valid=sample.feedback && std::isfinite(sample.feedback_current_a);
    return sample;
  }
  // 每轮只计算自己的闭环；相同周期采用同一份遥控与对端会话，发送前仍独立重查最新许可。
  void updateDebugPreview(const robot_control::ControlInput &input) {
    robot_debug::Session session;auto sample=collectDebugInput(input,session);
    if(robot_debug::drive::kVector && board_==robot_control::Board::Chassis) {
      if(!sample.neutral && sample.remote && sample.peer && sample.right==1 && sample.imu && sample.feedback) {
        if(drive_==nullptr || !drive_->vectorUpdate(sample.steer*robot_debug::direction::kPi,sample.magnitude,
            sample.yaw_delta,sample.dt_s,sample.vector_wheel_target)) {
          stopDebugPower(robot_debug::Reason::Feedback);return;
        }
      }
    }
    if(robot_debug::kPowerEnabled) {
      debug_power_.step(sample,session);robot_debug::publish(debug_power_.diagnostics());
      if(robot_debug::drive::kVector && drive_!=nullptr && !debug_power_.output().control)drive_->vectorStop();
      if(robot_debug::direction::kFour && board_==robot_control::Board::Chassis) {
        for(int i=1;i<4;++i) {
          const auto other=collectSteerInput(sample,i);extra_debug_[i-1].step(other,session);
        }
        for(int i=1;i<4;++i)if(extra_debug_[i-1].output().control!=debug_power_.output().control) {
          recordFourSteerFault(debug_power_.output().control?i:0,1);
          const auto reason=debug_power_.output().control?extra_debug_[i-1].diagnostics().reason:debug_power_.diagnostics().reason;
          stopDebugPower(reason);break;
        }
        for(int i=0;i<4;++i)if(powerAt(i).session().phase==3) {
          recordFourSteerFault(i,1);stopDebugPower(powerAt(i).diagnostics().reason);break;
        }
        publishFourSteers();
      }
    } else {debug_preview_.step(sample);robot_debug::publish(debug_preview_.diagnostics());}
  }
  // 预演只核对关节反馈是否新鲜、有限，不要求使能；此函数不用于正常动力许可。
  bool passiveAxisFresh(uint32_t now) const {
    return axis_!=nullptr && robot_control::MotorFeedbacks().fresh(8,now) &&
        std::isfinite(axis_->angle()) && std::isfinite(axis_->vel());
  }
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
        drive_->ready(now) && p.max_target_rate_rad_s>=params_.yaw_rate_rad_s+(params_.separate_only?0:params_.max_w_rad_s);
    return gimbal_->axis()==Gimbal::Axis::Pitch && p.pitch_limits_confirmed && p.max_target_rate_rad_s>=params_.pitch_rate_rad_s;
  }
  // 读取本板方向的完整状态；恢复自检需要读取非 Working 安全状态，而非强求控制许可。
  void readPeer(robot_control::ControlInput &in) {
    if(comm_==nullptr)return;
    if(board_==robot_control::Board::Chassis) {
      GimbalChassisComm::GimbalData data;
      in.peer_valid=comm_->readPeer(data);in.peer_state=data.state;in.peer_flags=data.flags;
      in.peer_feedback=data.feedback;in.peer_yaw_target=data.yaw_target;peer_motion_=data.motion;
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
    if(!robot_debug::kDebugEnabled) {
      auto &d=robot_main_diagnostics;d.board=board_==robot_control::Board::Chassis?0:1;
      d.life=static_cast<uint32_t>(o.state.life);d.control=o.control;d.ready=o.ready;
      d.imu=in.imu_ready;d.peer=in.peer_valid;d.axis_valid=in.axis_valid;
      d.yaw_target_rad=o.target.yaw;d.pitch_target_rad=o.target.pitch;
      if(axis_!=nullptr) {d.axis_angle_rad=axis_->angle();d.axis_speed_rad_s=axis_->vel();}
      if(gimbal_!=nullptr) {d.axis_position_target_rad=gimbal_->result().target_rad;d.axis_position_error_rad=gimbal_->result().error_rad;
        d.axis_speed_target_rad_s=gimbal_->result().target_speed_rad_s;d.torque_nm=gimbal_->result().torque_nm;}
    }
    GimbalChassisComm::Flags flags;
    flags.ready=o.ready;flags.imu_ready=in.imu_ready;
    flags.remote_valid=o.remote_valid;flags.emergency_released=o.emergency_released;flags.command_valid=o.command_valid;
    const bool fresh=axisFresh(hello_world::tick::GetTickMs());
    if(board_==robot_control::Board::Chassis) {
      GimbalChassisComm::ChassisData data;data.state=o.state;data.flags=flags;
      data.flags.yaw_feedback_valid=fresh;data.feedback.yaw=fresh?SupervisorWrap(axis_->angle()):0;
      data.chassis_yaw=in.imu_ready?SupervisorWrap(imu_yaw_):0;
      if(robot_debug::kDebugEnabled) {
        data.flags.debug_preview=true;data.flags.remote_valid=in.remote.valid;
        data.flags.debug_power=robot_debug::kPowerEnabled;data.session=debug_power_.session();
        data.flags.emergency_released=in.remote.valid && in.remote.released;
        data.flags.yaw_feedback_valid=passiveAxisFresh(in.now_ms);
      }
      data.target=o.target;comm_->chassis_data()=data;
    } else {
      GimbalChassisComm::GimbalData data;data.state=o.state;data.flags=flags;
      data.flags.pitch_feedback_valid=fresh;data.feedback.pitch=fresh?axis_->angle():0;
      data.yaw_target=o.control?o.target.yaw:(in.peer_flags.yaw_feedback_valid?in.peer_feedback.yaw:0);
      data.motion=o.motion;comm_->gimbal_data()=data;
      if(robot_debug::kDebugEnabled) {
        // 使用本周期状态机实际检查的同一份遥控样本，避免两次读取跨越回中而发布矛盾字段。
        data.flags.debug_preview=true;data.preview=debug_remote_;
        data.flags.debug_power=robot_debug::kPowerEnabled;data.session=debug_power_.session();
        data.flags.remote_valid=data.preview.valid;
        data.flags.emergency_released=data.preview.valid && data.preview.right!=2;
        data.flags.pitch_feedback_valid=passiveAxisFresh(in.now_ms);
        comm_->gimbal_data()=data;
      }
    }
  }
  // 使用共享状态机的单圈角度规则，不累积原始 IMU 圈数。
  static float SupervisorWrap(float a) {return robot_control::Supervisor::wrap(a);}
  // 按轮位取得私有控制器，诊断镜像不能作为输出来源。
  robot_debug::Power &powerAt(int i) {return i==0?debug_power_:extra_debug_[i-1];}
  // 在整组清状态前锁存首个失败轮的原始诊断；只读镜像不参与去重或输出判断。
  void recordFourSteerFault(int index,uint32_t source,uint32_t mask=0,
      const robot_debug::Input *sample=nullptr,const robot_debug::Session *peer=nullptr) {
    if(!robot_debug::direction::kFour || board_!=robot_control::Board::Chassis)return;
    const uint32_t token=powerAt(index).session().token;
    if(token==0 || token==four_fault_token_)return;
    const auto d=powerAt(index).diagnostics();
    if(source==1 && (d.reason==robot_debug::Reason::Emergency || d.reason==robot_debug::Reason::Centered))return;
    four_fault_token_=d.session_token;
    auto &f=robot_four_steer_fault;
    f.count=++four_fault_count_;f.time_ms=d.now_ms;f.wheel_id=index+1;f.source=source;
    f.stage=d.stage;f.reason=d.reason;f.local_token=d.session_token;f.peer_token=d.peer_token;
    f.local_phase=d.local_phase;f.peer_phase=d.peer_phase;
    f.transport=d.transport;f.imu=d.imu;f.feedback=d.feedback;f.remote=d.remote;f.peer=d.peer;
    f.right=d.right;f.left=d.left;f.snapshot_valid=d.link.snapshot_valid;f.peer_age_ms=d.link.age_ms;
    f.dt_s=d.dt_s;f.angle_rad=d.actual_angle;f.velocity_rad_s=d.actual_velocity;
    robot_debug::publishRxEvent(f.rx_event,d.link.rx_event);
    f.failure_mask=mask;
    f.control_age_ms=hello_world::tick::GetTickMs()-debug_control_ms_;
    uint32_t control_mask=0;
    for(int i=0;i<4;++i)if(powerAt(i).output().control)control_mask|=1u<<i;
    f.control_mask=control_mask;
    if(sample!=nullptr && peer!=nullptr) {
      f.time_ms=sample->now_ms;f.reason=powerAt(index).failureReason(*sample,mask);
      f.peer_token=peer->token;f.peer_phase=peer->phase;
      f.transport=sample->transport;f.imu=sample->imu;f.feedback=sample->feedback;
      f.remote=sample->remote;f.peer=sample->peer;f.right=sample->right;f.left=sample->left;
      f.snapshot_valid=sample->link.snapshot_valid;f.peer_age_ms=sample->link.age_ms;
      f.dt_s=sample->dt_s;f.angle_rad=sample->angle;f.velocity_rad_s=sample->velocity;
      robot_debug::publishRxEvent(f.rx_event,sample->link.rx_event);
    }
  }
  uint32_t four_fault_token_=0,four_fault_count_=0;
  uint32_t main_tuning_apply_count_=0;
  // 发布四个闭环的只读目标/反馈/电流，正常停止后active=0。
  void publishFourSteers() {
    if(!robot_debug::direction::kFour || board_!=robot_control::Board::Chassis)return;
    for(int i=0;i<4;++i)robot_debug::publishLoops(robot_four_steer_diagnostics[i],powerAt(i).loopDiagnostics());
  }
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
  uint32_t poll_count_=0,ready_updates_=0,max_poll_gap_ms_=0,imu_drop_count_=0;
  bool health_was_fresh_=false;
  uint32_t imu_sample_ms_=0,last_poll_ms_=0;float imu_yaw_=0;
  ChassisMotion peer_motion_{};
  hello_world::motor::Motor *debug_target_=nullptr;
  robot_debug::Preview debug_preview_;
  robot_debug::Power debug_power_;
  robot_debug::Power extra_debug_[3];
  hello_world::motor::Motor *debug_steers_[4]{};
  GimbalChassisComm::GimbalData::PreviewRemote debug_remote_{};
  uint32_t debug_control_ms_=0;float debug_dt_s_=0;
};
}
#endif
