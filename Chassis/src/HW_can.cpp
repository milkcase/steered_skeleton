/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *******************************************************************************
 * @attention :
 *  接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已在此预留，仅完成「取 8 字节 +
 *  再次使能接收中断」，具体「按帧 ID 分发」逻辑留给新人：
 *    - M3508 / DM 电机反馈：调用对应电机的 decode(can?_rx_data)
 *    - 双板通讯帧：调用新人自写的协议解码
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "ins_motor.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"
#include "motor_feedback.hpp"

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter;

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox);
}

/**
 * @brief   CAN1 FIFO0 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    此处只取了 8 字节并重新使能接收中断，具体分发由新人实现
 */
// 处理本板 CAN1 FIFO0 的接收；仅分发本板负责的反馈，最后恢复接收通知。
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      /* TODO【新人任务】: 在此按 rx_header1.StdId 路由 8 字节 can1_rx_data。
       * 判定方法：把 StdId 与各电机的接收 ID（GetRxId）及双板通讯帧 ID 逐一比对：
       *   - 命中某电机的接收 ID：调用该电机 decode(can1_rx_data) 解包反馈；
       *   - 命中双板通讯帧 ID：调用自己的 GimbalChassisComm 解码；
       *   - 都未命中：丢弃（返回前记得重新使能接收中断，见下方调用）。
       */
      // 拓扑图中底盘 CAN1 连接四台转向电机；只解析完整的标准数据帧。
      // 不提前返回，确保无效帧和未知帧也会经过函数末尾的接收通知恢复。
      if (rx_header1.IDE == CAN_ID_STD && rx_header1.RTR == CAN_RTR_DATA &&
          rx_header1.DLC == 8) {
        for (int idx = 0; idx < 4; ++idx) {
          auto *motor = GetMotorSteer(idx);
          // 从实例读取设备 ID，避免把设备 ID 与 CAN 反馈报文 ID 混为一谈。
          if (motor != nullptr &&
              rx_header1.StdId == hello_world::motor::GM6020::GetRxId(
                                       motor->motor_info().id)) {
            motor->decode(can1_rx_data);
            // 第九项：只刷新本板该舵机的本地反馈时间，非法编码会撤销有效性。
            robot_control::MotorFeedbacks().recordDji(idx+4,can1_rx_data,hello_world::tick::GetTickMs());
            break;
          }
        }
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    同上，具体分发由新人实现
 */
// 处理共享 CAN2 的 FIFO1 接收；按帧格式和反馈 ID 解码本板负责的电机，不发送命令。
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      /* TODO【新人任务】: 同 FIFO0，按 rx_header2.StdId 路由 8 字节 can2_rx_data。 */
      // 两板 CAN2 共线，包含驱动轮、yaw、pitch 和板间报文。
      // 本板只解码自己负责的驱动轮与 yaw；其余帧当前丢弃，板间协议后续接入。
      // 此处仅更新反馈，不设置电机输入，也不发送使能或运动命令。
      if (rx_header2.IDE == CAN_ID_STD && rx_header2.RTR == CAN_RTR_DATA &&
          rx_header2.DLC == 8) {
        // 第八项：只在共享 CAN2 按对端 ID 分发，协议对象自行检查页号、序号与 CRC。
        auto *comm=GetGimbalChassisComm();
        if(comm!=nullptr && rx_header2.StdId==comm->rxId())comm->decode(can2_rx_data);
        // yaw 由底盘负责，MIT 反馈帧为设备 ID + 0x10，与驱动反馈分开。
        auto *yaw = GetMotorYaw();
        if (yaw != nullptr &&
            rx_header2.StdId == hello_world::motor::DaMiao::GetRxId(yaw->motor_info().id)) {
          yaw->decode(can2_rx_data);
          // 使能位必须来自新鲜状态帧，不以 getter 非空或转述反馈代替。
          robot_control::MotorFeedbacks().recordAxis(yaw->motor_info().id,can2_rx_data,hello_world::tick::GetTickMs());
        }
        for (int idx = 0; idx < 4; ++idx) {
          auto *motor = GetMotorWheel(idx);
          if (motor != nullptr &&
              rx_header2.StdId == hello_world::motor::M3508::GetRxId(
                                       motor->motor_info().id)) {
            motor->decode(can2_rx_data);
            robot_control::MotorFeedbacks().recordDji(idx,can2_rx_data,hello_world::tick::GetTickMs());
            break;
          }
        }
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
