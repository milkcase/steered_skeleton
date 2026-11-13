#pragma once
#include "platform.hpp"
struct CAN_HandleTypeDef {uint32_t id;};
extern CAN_HandleTypeDef hcan1,hcan2;
constexpr uint32_t CAN_TX_MAILBOX0=1,CAN_TX_MAILBOX1=2,CAN_TX_MAILBOX2=4;
// 捕获撤销旧邮箱动作，不访问 CAN 外设。
extern "C" HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *,uint32_t);
