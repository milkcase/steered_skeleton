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
namespace robot {
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
      : board_(board),params_(params),supervisor_(board,params),debug_preview_(robot_debug::kPreviewEnabled,board==robot_control::Board::Chassis),
        debug_power_(robot_debug::kPowerEnabled,board==robot_control::Board::Chassis) {}
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
  // 仅注册底盘左前转向的只读采样对象，零输出预演不会通过此指针设置目标或动力。
  // 动力调试版也固定此对象，但只有最终发送层重查独立许可后才可写入受限输入。
  void registerDebugTarget(hello_world::motor::Motor *motor) {
    if(board_==robot_control::Board::Chassis)debug_target_=motor;
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
    sensor_polled_=true;last_poll_ms_=now;
    const bool updated=imu_->update(),ready=imu_->isReady();
    const float yaw=ready?imu_->yaw():0;
    remote_input::InterruptLock lock;
    imu_ready_=ready;
    if(updated && ready) {imu_yaw_=yaw;imu_sample_ms_=hello_world::tick::GetTickMs();}
  }
  // 按真实调度间隔处理最新输入、迁移、计算、统一归零和状态发布；不在此发送 CAN。
  void update(float dt_s=0.001f) {
    robot_control::ControlInput input;
    input.dt_s=dt_s;
    debug_dt_s_=dt_s;debug_control_ms_=hello_world::tick::GetTickMs();
    {
      remote_input::InterruptLock lock;
      input.now_ms=hello_world::tick::GetTickMs();
      input.imu_ready=imuFresh(input.now_ms);
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
    // 通用保护仍执行原停止动作；只补充首次失效记录，不覆盖先前复查已保存的现场。
    if(robot_debug::kPowerEnabled) {
      if(debug_power_.output().control)debug_power_.recordStopRequest();
      debug_power_.stop(robot_debug::Reason::PeerConsent);
    }
    supervisor_.stop();zeroInputs();
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
    debug_power_.stop(reason);zeroInputs();
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
    if(age>3)mask|=robot_debug::CheckControlAge;
    if(mask==0)return true;
    debug_power_.recordStopCheck(sample,session,mask,2,age);
    if(age>3) {stopDebugPower(robot_debug::Reason::Clock);return false;}
    // 直接使用刚完成的真实截止检查，不为取起始时间再复制整份诊断记录。
    stopDebugPower((mask&robot_debug::CheckTime)!=0?robot_debug::Reason::TimeLimit:robot_debug::Reason::PeerConsent);
    return false;
  }
  // 仅编码左前ID1一槽；重设私有受限指令并验证驱动写入范围，其他六字节必须保持零。
  bool fillDebugSteerFrame(uint8_t data[8]) {
    for(int i=0;i<8;++i)data[i]=0;
    if(!debugPowerStillSafe())return false;
    const float value=debug_power_.command();
    if(debug_target_==nullptr || debug_target_->motor_info().id!=1 ||
        debug_target_->get_input_type()!=hello_world::motor::InputType::kCurr ||
        !std::isfinite(value) || std::fabs(value)>robot_debug::tuning::kCurrentLimitA ||
        debug_target_->setInput(value)!=hello_world::motor::Status::kOk || !debug_target_->encode(data)) {
      stopDebugPower(robot_debug::Reason::Configuration);for(int i=0;i<8;++i)data[i]=0;return false;
    }
    for(int i=2;i<8;++i)if(data[i]!=0) {
      stopDebugPower(robot_debug::Reason::Configuration);for(int j=0;j<8;++j)data[j]=0;return false;
    }
    const int encoded=(data[0]<<8)|data[1];const int signed_raw=encoded>=32768?encoded-65536:encoded;
    // 电流协议±16384对应±3A；0.05A向零量化最多273，不能沿用原电压原始输入上限。
    // 可调版按编译时同一档电流计算最终整数上限，仍与私有控制器限流独立复查。
    if(signed_raw>robot_debug::tuning::kRawCurrentLimit || signed_raw< -robot_debug::tuning::kRawCurrentLimit) {
      stopDebugPower(robot_debug::Reason::Configuration);for(int i=0;i<8;++i)data[i]=0;return false;
    }
    return true;
  }
  // 返回本板关节对象，只供本板统一发送层使用。
  hello_world::motor::DaMiao *axis() const {return axis_;}
 protected:
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
    out.yaw_rate=-channel(rc.rc_rh())*params_.yaw_rate_rad_s;
    out.pitch_rate=channel(rc.rc_rv())*params_.pitch_rate_rad_s;
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
      if(std::fabs(values[i])>0.05f) {out.neutral=false;if(i!=1)out.other_neutral=false;}
    }
    out.right=static_cast<uint8_t>(rc.rc_r_switch());out.left=static_cast<uint8_t>(rc.rc_l_switch());
    if((out.right!=1 && out.right!=2 && out.right!=3) || (out.left!=1 && out.left!=2 && out.left!=3))return {};
    out.steer=rc.rc_lh();out.valid=true;return out;
  }
  // 将真实遥控、完整对端快照和本地被动反馈送入纯内存预演器，发布全局诊断。
  robot_debug::Input collectDebugInput(const robot_control::ControlInput &input,robot_debug::Session &peer_session) {
    robot_debug::Input sample;sample.now_ms=input.now_ms;sample.dt_s=input.dt_s;
    sample.transport=transport_ready_;sample.imu=input.imu_ready;
    GimbalChassisComm::GimbalData::PreviewRemote remote;
    {
      remote_input::InterruptLock lock;
      bool received=false;GimbalChassisComm::Flags peer_flags;
      if(board_==robot_control::Board::Chassis) {
        GimbalChassisComm::GimbalData data;
        received=comm_!=nullptr && comm_->readPeer(data);peer_flags=data.flags;
        peer_session=data.session;
        sample.peer=received && (!robot_debug::kDebugEnabled ||
            (data.flags.debug_preview && data.flags.debug_power==robot_debug::kPowerEnabled && data.flags.imu_ready && data.flags.pitch_feedback_valid && !data.flags.fault));
        if(received && data.flags.debug_preview)remote=data.preview;
        sample.feedback=debug_target_!=nullptr && debug_target_->motor_info().id==1 &&
            debug_target_->get_input_type()==hello_world::motor::InputType::kCurr;
        for(int i=0;i<9;++i)sample.feedback=robot_control::MotorFeedbacks().fresh(i,input.now_ms) && sample.feedback;
        if(robot_debug::kPowerEnabled && debug_target_!=nullptr)
          sample.feedback=sample.feedback && debug_target_->motor_info().curr_input_lim>=robot_debug::tuning::kCurrentLimitA;
        if(debug_target_!=nullptr) {
          sample.angle=debug_target_->angle();sample.velocity=debug_target_->vel();
          // 使用提供驱动从反馈帧解码的curr()，绝不由setInput或命令反推实际电流。
          sample.feedback_current_a=debug_target_->curr();
          sample.feedback_current_valid=sample.feedback && std::isfinite(sample.feedback_current_a);
        }
      } else {
        remote=previewRemote(&sample);GimbalChassisComm::ChassisData data;
        received=comm_!=nullptr && comm_->readPeer(data);peer_flags=data.flags;
        peer_session=data.session;
        sample.peer=received && (!robot_debug::kDebugEnabled ||
            (data.flags.debug_preview && data.flags.debug_power==robot_debug::kPowerEnabled && data.flags.imu_ready && data.flags.yaw_feedback_valid && !data.flags.fault));
        sample.feedback=passiveAxisFresh(input.now_ms);
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
    return sample;
  }
  // 先计算本板专用状态，再发布诊断；动力值只在私有控制器内生成，预演仍固定零输出。
  void updateDebugPreview(const robot_control::ControlInput &input) {
    robot_debug::Session session;const auto sample=collectDebugInput(input,session);
    if(robot_debug::kPowerEnabled) {
      debug_power_.step(sample,session);robot_debug::publish(debug_power_.diagnostics());
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
    if(!transport_ready_ || !supervisor_.configured() || axis_==nullptr || gimbal_==nullptr || axis_pid_==nullptr ||
        !gimbal_->configured() || !axis_pid_->configured() || axis_pid_->params().kp<=0 ||
        axis_pid_->params().output_limit<=0 || axis_->get_input_type()!=hello_world::motor::InputType::kTorq ||
        axis_->current_ctrl_mode()!=hello_world::motor::DaMiao::CtrlMode::kMIT)return false;
    const auto &p=gimbal_->params();
    if(p.max_torque_nm<=0 || p.max_target_rate_rad_s<=0 || axis_pid_->params().output_limit>p.max_torque_nm ||
        p.max_torque_nm>axis_->motor_info().torq_input_lim)return false;
    if(board_==robot_control::Board::Chassis)return gimbal_->axis()==Gimbal::Axis::Yaw && drive_!=nullptr &&
        drive_->ready(now) && p.max_target_rate_rad_s>=params_.yaw_rate_rad_s+params_.max_w_rad_s;
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
        data.flags.debug_preview=true;data.preview=previewRemote();
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
  robot_control::Board board_;robot_control::Params params_;robot_control::Supervisor supervisor_;
  hello_world::imu::Imu *imu_=nullptr;
  hello_world::motor::DaMiao *axis_=nullptr;Gimbal *gimbal_=nullptr;hello_world::pid::Pid *axis_pid_=nullptr;
  GimbalChassisComm *comm_=nullptr;DrivePort *drive_=nullptr;
  bool transport_ready_=false,imu_started_=false,imu_ready_=false,sensor_polled_=false;
  uint32_t imu_sample_ms_=0,last_poll_ms_=0;float imu_yaw_=0;
  ChassisMotion peer_motion_{};
  hello_world::motor::Motor *debug_target_=nullptr;
  robot_debug::Preview debug_preview_;
  robot_debug::Power debug_power_;
  uint32_t debug_control_ms_=0;float debug_dt_s_=0;
};
}
#endif
