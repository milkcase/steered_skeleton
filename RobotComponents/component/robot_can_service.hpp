#ifndef STEERED_ROBOT_CAN_SERVICE_HPP_
#define STEERED_ROBOT_CAN_SERVICE_HPP_
#include "robot_runtime.hpp"
#include "board_comm_task.hpp"
#include "can.h"
#include "motor_GM6020.hpp"
#include "motor_M3508.hpp"
#include "vehicle_motor_config.hpp"
namespace robot {
// 本板唯一的电机发送层，不发送非本板负责的关节，不在此生成运动目标。
class RobotCanService {
 public:
  // 保存角色和本板状态机；电机列表在初始化时注册，不在中断内创建对象。
  RobotCanService(RobotRuntime *robot,bool chassis) : robot_(robot),chassis_(chassis) {}
  // 注册底盘本地的共享帧电机，云台固件不调用该接口。
  void registerDrive(hello_world::motor::Motor *const wheels[4],hello_world::motor::Motor *const steers[4]) {
    for(int i=0;i<4;++i) {wheels_[i]=wheels[i];steers_[i]=steers[i];}
    if(robot_debug::direction::kFour && robot_!=nullptr)robot_->registerDebugSteers(steers);
  }
  // 在非中断初始化阶段配置 CAN1；CAN2 使用第八项接口，所有 HAL 字段明确初始化。
  bool init() {
    // 最新拓扑：底盘先启动 CAN1 舵轮；云台先启动 CAN2 pitch，再启动各自板间接口。
    // 两路都是必需接口，任一配置、启动或通知失败均不能报告整机通信就绪。
    static_assert(vehicle_config::kChassisBoardCommBus==vehicle_config::CanBus::Can2);
    static_assert(vehicle_config::kGimbalBoardCommBus==vehicle_config::CanBus::Can1);
    {
      auto *motor_can=chassis_?&hcan1:&hcan2;
      CAN_FilterTypeDef f{};f.FilterBank=chassis_?0:14;f.SlaveStartFilterBank=14;
      f.FilterMode=CAN_FILTERMODE_IDMASK;f.FilterScale=CAN_FILTERSCALE_32BIT;
      f.FilterFIFOAssignment=chassis_?CAN_FILTER_FIFO0:CAN_FILTER_FIFO1;f.FilterActivation=ENABLE;
      if(HAL_CAN_ConfigFilter(motor_can,&f)!=HAL_OK ||
          (HAL_CAN_GetState(motor_can)!=HAL_CAN_STATE_LISTENING && HAL_CAN_Start(motor_can)!=HAL_OK) ||
          HAL_CAN_ActivateNotification(motor_can,chassis_?CAN_IT_RX_FIFO0_MSG_PENDING:CAN_IT_RX_FIFO1_MSG_PENDING)!=HAL_OK)return false;
    }
    return StartBoardCommunication();
  }
  // 每 1ms 调用：合并同帧电机、分频共享 CAN2，并在失效转换时先撤销旧邮箱。
  void tick() {
    remote_input::InterruptLock lock;
    if(robot_==nullptr)return;
    if(robot_->debugPowerEnabled()) {tickDebugPower();return;}
    auto action=robot_->output().axis_action;
    bool control=robot_->output().control;
    // 零输出预演的编译期硬隔离：不读取可被Watch修改的诊断字段来决定是否允许动力。
    if(robot_->debugPreviewEnabled()) {
      robot_->forceZeroForPreview();control=false;action=robot_control::AxisAction::Disable;
    }
    const uint32_t now=hello_world::tick::GetTickMs();
    if(!have_tx_clock_) {last_steer_ms_=last_wheel_ms_=last_axis_ms_=now;have_tx_clock_=true;}
    if(control && ((chassis_ && (now-last_steer_ms_>=20 || now-last_wheel_ms_>=20)) || now-last_axis_ms_>=20)) {
      robot_->setTransportReady(false);control=false;action=robot_control::AxisAction::Disable;
    }
    if(control && !robot_->recheckActiveControl()) {
      control=false;action=robot_->output().axis_action;
    } else if(action==robot_control::AxisAction::ArmZero && !robot_->armStillSafe()) {
      robot_->stop(MainStopSource::ArmGate);control=false;action=robot_control::AxisAction::Disable;
    }
    const bool active=control || action==robot_control::AxisAction::ArmZero || action==robot_control::AxisAction::Zero;
    const bool changed=!have_state_ || (was_active_ && !active) || (was_control_ && !control);
    if(changed)abortOld();
    have_state_=true;was_active_=active;was_control_=control;
    // 非 Working 的全部电机输入保持零，电调共享帧仍显式发送零，不能依靠“不发包”停车。
    if(chassis_ && (control || changed || cycle_%20==0))sendGroup(&hcan1,steers_,control);
    if(chassis_ && (changed || cycle_%4==0 || (!active && cycle_%20==0)))sendGroup(&hcan2,wheels_,control);
    if(changed || cycle_%4==1 || (!active && cycle_%20==0))sendAxis(action);
    // 一页间隔 3ms、同快照三页首尾约 6ms，满足第八项 10ms 拼包限制。
    if(cycle_%3==2)SendBoardCommunication();
    // 参数帧最后排队且预留一个邮箱，不抢占运动/保护/普通板间帧。
    if(main_tuning::enabled && !robot_debug::kDebugEnabled && cycle_%5==3)sendMainTuning();
    // 云台每2ms最多排一页pitch观测，9页同组；最后排队并留一个邮箱，拥塞时跳过不阻塞控制。
    if(!chassis_ && main_tuning::enabled && !robot_debug::kDebugEnabled && cycle_%2==0 &&
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>=2) {
      uint8_t data[8]{};pitch_feedback::channel.frame(data,now);
      if(send(&hcan1,pitch_feedback::kId,data))pitch_feedback::channel.sent();
    }
    cycle_=(cycle_+1)%60;
  }
 private:
  // 主程序参数通道独立于电机帧；只在成功排队后推进发送序号，不阻塞等待ACK。
  void sendMainTuning() {
    auto *can=chassis_?&hcan2:&hcan1;
    if(HAL_CAN_GetTxMailboxesFreeLevel(can)<2)return;
    uint8_t data[8]{};const uint32_t now=hello_world::tick::GetTickMs();
    if(chassis_) {
      if(main_tuning::parameter_source.frame(data) && send(can,main_tuning::kParameterId,data))main_tuning::parameter_source.sent();
    } else if(main_tuning::parameter_sink.frame(data,now))send(can,main_tuning::kParameterAckId,data);
  }
  // 独立动力调试：编译模式选择四转向或旧单电机路径，必须Working和最新私有会话同时许可。
  void tickDebugPower() {
    robot_->forceZeroForPreview();
    if(HAL_CAN_GetState(&hcan1)!=HAL_CAN_STATE_LISTENING || HAL_CAN_GetState(&hcan2)!=HAL_CAN_STATE_LISTENING) {
      robot_->stopDebugPower(robot_debug::Reason::Transport);robot_->setTransportReady(false);
    }
    const uint32_t now=hello_world::tick::GetTickMs();
    if(!have_tx_clock_) {last_steer_ms_=last_wheel_ms_=last_axis_ms_=now;have_tx_clock_=true;}
    bool active=chassis_ && robot_->output().control && robot_->output().state.life==LifeState::Working;
    const uint32_t last_target_ms=robot_debug::drive::kEnabled?last_wheel_ms_:last_steer_ms_;
    if(active && (now-last_target_ms>=20 || (robot_debug::drive::kVector && now-last_steer_ms_>=20))) {
      robot_->stopDebugPower(robot_debug::Reason::Transmit);active=false;
    }
    uint8_t target[8]{};
    auto *selected=robot_debug::drive::kEnabled?wheels_[0]:steers_[0];
    if(active && (selected==nullptr || selected!=robot_->debugTarget() || selected->motor_info().id!=1)) {
      robot_->stopDebugPower(robot_debug::Reason::Configuration);active=false;
    }
    if(active)active=robot_->fillDebugTargetFrame(target);
    uint8_t vector_steer[8]{};
    if(active && robot_debug::drive::kVector && !robot_->fillVectorSteerFrame(vector_steer)) {
      active=false;for(auto &v:target)v=0;
    }
    const bool changed=!have_power_state_ || (was_power_active_ && !active);
    if(changed)abortOld();
    have_power_state_=true;was_power_active_=active;
    // 停止后的首帧立即归零；稳态仍周期发送零帧，绝不以“不发包”代替停车。
    if(robot_debug::drive::kEnabled) {
      // 驱动和板间通信共用CAN2，驱动每4ms一帧；四转向同帧持续为零，停止转换立即发零。
      if(chassis_ && (changed || cycle_%4==0))send(&hcan2,hello_world::motor::M3508::GetTxId(1),target);
      if(chassis_ && robot_debug::drive::kVector && (active || changed || cycle_%20==0)) {
        if(!active)for(auto &v:vector_steer)v=0;
        send(&hcan1,hello_world::motor::GM6020::GetTxId(1,hello_world::motor::InputType::kCurr),vector_steer);
      } else if(chassis_ && (changed || cycle_%20==0))sendGroup(&hcan1,steers_,false);
    } else {
      if(chassis_ && (active || changed || cycle_%20==0))
        send(&hcan1,hello_world::motor::GM6020::GetTxId(1,hello_world::motor::InputType::kCurr),target);
      if(chassis_ && (changed || cycle_%4==0))sendGroup(&hcan2,wheels_,false);
    }
    if(changed || cycle_%4==1)sendAxis(robot_control::AxisAction::Disable);
    if(cycle_%3==2)SendBoardCommunication();
    cycle_=(cycle_+1)%60;
  }
  // 丢弃未发送完的旧命令；只在首次或从活动转保护时调用，不不断撤销新的零帧。
  void abortOld() {
    // 两板现均使用两个本地控制器：必须同时撤销板间帧和本板电机的旧邮箱。
    HAL_CAN_AbortTxRequest(&hcan1,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
    HAL_CAN_AbortTxRequest(&hcan2,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
    robot_->restartCommunicationTx();
  }
  // 非阻塞排队；邮箱忙等待下一周期，HAL 真正报错则撤销本板许可并归零。
  bool send(CAN_HandleTypeDef *can,uint32_t id,uint8_t data[8]) {
    if(HAL_CAN_GetState(can)!=HAL_CAN_STATE_LISTENING || HAL_CAN_GetTxMailboxesFreeLevel(can)==0)return false;
    CAN_TxHeaderTypeDef header{};header.StdId=id;header.IDE=CAN_ID_STD;header.RTR=CAN_RTR_DATA;header.DLC=8;
    uint32_t mailbox=0;
    if(HAL_CAN_AddTxMessage(can,&header,data,&mailbox)==HAL_OK) {
      const uint32_t now=hello_world::tick::GetTickMs();
      if(can==&hcan1 && id==hello_world::motor::GM6020::GetTxId(1,hello_world::motor::InputType::kCurr))last_steer_ms_=now;
      else if(can==&hcan2 && id==hello_world::motor::M3508::GetTxId(1))last_wheel_ms_=now;
      else if(id<16)last_axis_ms_=now;
      return true;
    }
    if(robot_->debugPowerEnabled())robot_->stopDebugPower(robot_debug::Reason::Transmit);
    robot_->setTransportReady(false);abortOld();return false;
  }
  // ID1..4 的各电机仅改自己两字节，合并后只发一次 0x200；保护时不使用残留编码。
  // 协议复核修正：CAN1 GM6020 的 kRaw 为 0x1FF，CAN2 M3508 为 0x200，实际 ID 以驱动 API 为准。
  // 最新实物确认四台GM6020为电流模式：现用kCurr/0x1FE，保护零帧也用同一电流帧；驱动轮不变。
  void sendGroup(CAN_HandleTypeDef *can,hello_world::motor::Motor *const motors[4],bool control) {
    uint8_t data[8]{};
    if(control && !robot_->debugPreviewEnabled() && robot_->controlStillSafe()) {
      for(int i=0;i<4;++i) {
        if(motors[i]==nullptr || motors[i]->motor_info().id<1 || motors[i]->motor_info().id>4 ||
            !std::isfinite(motors[i]->getInput()) || !motors[i]->encode(data)) {
          robot_->stop(MainStopSource::GroupEncoding);for(auto &b:data)b=0;abortOld();break;
        }
      }
    }
    const uint32_t id=can==&hcan1?
        hello_world::motor::GM6020::GetTxId(1,hello_world::motor::InputType::kCurr):
        hello_world::motor::M3508::GetTxId(1);
    send(can,id,data);
  }
  // 只发送明确的使能/失能命令；恢复使能受遥控、自检和 Supervisor 的时间窗口约束。
  void axisCommand(uint8_t command) {
    if((robot_->debugPreviewEnabled() || robot_->debugPowerEnabled()) && command!=0xFD)return;
    auto *axis=robot_->axis();if(axis==nullptr)return;
    // 电机仍报告使能时先记下软件主动失能现场，后续禁用反馈不能覆盖首个来源。
    if(command==0xFD && axis->is_enabled())robot_->recordMainStop(MainStopSource::DisableCommand);
    uint8_t data[8];for(auto &b:data)b=0xFF;data[7]=command;
    send(&hcan2,axis->motor_info().id,data);
  }
  // 使用提供的 MIT 编码器，但拒绝隐含清错、保存零位、寄存器写入和非零刚度。
  void sendAxis(robot_control::AxisAction action) {
    if(robot_->debugPreviewEnabled() || robot_->debugPowerEnabled())action=robot_control::AxisAction::Disable;
    auto *axis=robot_->axis();if(axis==nullptr)return;
    const uint32_t now=hello_world::tick::GetTickMs();
    if(action==robot_control::AxisAction::Disable) {axis->setInput(0);axisCommand(0xFD);return;}
    if(action==robot_control::AxisAction::ArmZero) {
      axis->setInput(0);
      if(!robot_->armStillSafe()) {robot_->stop(MainStopSource::ArmGate);axisCommand(0xFD);return;}
      if(!robot_control::MotorFeedbacks().axisEnabled(now)) {
        if(!enable_sent_ || now-last_enable_ms_>=100) {
          axisCommand(0xFC);enable_sent_=true;last_enable_ms_=now;
        }
        return;
      }
    } else if(action==robot_control::AxisAction::Zero)axis->setInput(0);
    else if(!robot_->controlStillSafe()) {robot_->stop(MainStopSource::AxisGate);axisCommand(0xFD);return;}
    uint8_t data[8]{};
    if(!axis->encode(data) || axis->tx_id()!=axis->motor_info().id ||
        (data[3]&15)!=0 || data[4]!=0 || data[5]!=0 || (data[6]&0xF0)!=0) {
      // 提供驱动异常或排入非纯力矩命令时停机，不绕过驱动格式或自动清除实物故障。
      robot_->setTransportReady(false);axis->setInput(0);abortOld();axisCommand(0xFD);return;
    }
    send(&hcan2,axis->tx_id(),data);
  }
  RobotRuntime *robot_;bool chassis_=false,have_state_=false,was_active_=false,was_control_=false,enable_sent_=false;
  bool have_tx_clock_=false;
  bool have_power_state_=false,was_power_active_=false;
  uint32_t last_steer_ms_=0,last_wheel_ms_=0,last_axis_ms_=0;
  uint32_t cycle_=0,last_enable_ms_=0;
  hello_world::motor::Motor *wheels_[4]{},*steers_[4]{};
};
}
#endif
