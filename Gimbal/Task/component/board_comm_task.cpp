#include "board_comm_task.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "can.h"
#include "dbus_uart.hpp"
#include "vehicle_motor_config.hpp"

// 云台板间通信由 CAN1 接入底盘 CAN2 的灰色总线，pitch 独占另一条 CAN2 蓝色总线。
static_assert(vehicle_config::kGimbalBoardCommBus==vehicle_config::CanBus::Can1);

// 全收掩码保留同总线电机反馈；明确初始化所有滤波字段，不修改提供的 CAN 基础函数。
bool StartBoardCommunication() {
  CAN_FilterTypeDef filter{};
  filter.FilterMode=CAN_FILTERMODE_IDMASK;filter.FilterScale=CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;filter.FilterBank=0;
  filter.SlaveStartFilterBank=14;filter.FilterActivation=ENABLE;
  if(HAL_CAN_ConfigFilter(&hcan1,&filter)!=HAL_OK)return false;
  if(HAL_CAN_GetState(&hcan1)!=HAL_CAN_STATE_LISTENING && HAL_CAN_Start(&hcan1)!=HAL_OK)return false;
  return HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)==HAL_OK;
}

// 每次仅排入一页，邮箱忙时不编码、不推进页号；HAL 失败不会虚构成功发送。
bool SendBoardCommunication() {
  if(HAL_CAN_GetState(&hcan1)!=HAL_CAN_STATE_LISTENING || HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0)return false;
  uint8_t frame[8]{};CAN_TxHeaderTypeDef header{};uint32_t mailbox=0;
  {
    remote_input::InterruptLock lock;
    auto *comm=GetGimbalChassisComm();
    if(comm==nullptr)return false;
    comm->encode(frame);header.StdId=comm->txId();
  }
  header.IDE=CAN_ID_STD;header.RTR=CAN_RTR_DATA;header.DLC=8;
  return HAL_CAN_AddTxMessage(&hcan1,&header,frame,&mailbox)==HAL_OK;
}

// 复制完整的新鲜状态，允许读取未 Working 状态，避免两板等待对方先 Working 的死锁。
bool ReadPeerBoard(robot::GimbalChassisComm::ChassisData &out) {
  remote_input::InterruptLock lock;
  auto *comm=GetGimbalChassisComm();
  if(comm==nullptr) {out={};return false;}
  return comm->readPeer(out);
}

// 在同一临界区检查许可并复制指令；失败清空输出，避免读取与许可检查交错。
bool ReadPeerCommand(robot::GimbalChassisComm::ChassisData &out) {
  remote_input::InterruptLock lock;
  auto *comm=GetGimbalChassisComm();
  if(comm==nullptr || !comm->commandsUsable()) {out={};return false;}
  return comm->readPeer(out);
}

// 在同一临界区内读取离线、急停/故障抑制和对端必要许可，供第九项组成完整门控。
bool PeerCommandsUsable() {
  remote_input::InterruptLock lock;
  auto *comm=GetGimbalChassisComm();
  return comm!=nullptr && comm->commandsUsable();
}
