#ifndef STEERED_ROBOT_RUNTIME_HPP_
#define STEERED_ROBOT_RUNTIME_HPP_
#include "robot_supervisor.hpp"
#include "motor_feedback.hpp"
#include "gimbal_controller.hpp"
#include "imu.hpp"
#include "dbus_uart.hpp"
#include "ins_comm.hpp"
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
      : board_(board),params_(params),supervisor_(board,params) {}
  // 注册本板 IMU；初始化单独在非中断阶段进行。
  void registerImu(hello_world::imu::Imu *imu) {imu_=imu;}
  // 注册本板独占关节、位置环和 PID；云台只能注册 pitch，底盘只能注册 yaw。
  void registerAxis(hello_world::motor::DaMiao *axis,Gimbal *gimbal,hello_world::pid::Pid *pid) {
    axis_=axis;gimbal_=gimbal;axis_pid_=pid;
  }
  // 注册本板通信对象，不接收另一固件的指针。
  void registerCommunication(GimbalChassisComm *comm) {comm_=comm;}
  // 仅底盘注册轮组闭环，云台角色拒绝使用该指针。
  void registerDrive(DrivePort *drive) {if(board_==robot_control::Board::Chassis)drive_=drive;}
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
  }
  // 最终发帧前再次核对异步更新的急停与反馈；非控制阶段不借旧状态恢复非零输出。
  bool controlStillSafe() {
    if(!supervisor_.output().control)return false;
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
  void stop() {
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
    const auto &o=supervisor_.output();
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
      data.motion=o.motion;comm_->gimbal_data()=data;
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
};
}
#endif
