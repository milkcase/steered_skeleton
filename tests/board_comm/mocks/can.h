#pragma once
#include <cstdint>
struct CAN_HandleTypeDef {int port;};
struct CAN_RxHeaderTypeDef {uint32_t StdId=0,IDE=0,RTR=0,DLC=0;};
using CAN_TxHeaderTypeDef=CAN_RxHeaderTypeDef;
struct CAN_FilterTypeDef {
  uint32_t FilterMode=0,FilterScale=0,FilterIdHigh=0,FilterIdLow=0,FilterMaskIdHigh=0,FilterMaskIdLow=0;
  uint32_t FilterActivation=0,FilterFIFOAssignment=0,FilterBank=0,SlaveStartFilterBank=0;
};
enum HAL_StatusTypeDef {HAL_OK,HAL_ERROR};
enum HAL_CAN_StateTypeDef {HAL_CAN_STATE_READY,HAL_CAN_STATE_LISTENING};
constexpr uint32_t CAN_ID_STD=0,CAN_ID_EXT=4,CAN_RTR_DATA=0,CAN_RTR_REMOTE=2;
constexpr uint32_t CAN_RX_FIFO0=0,CAN_RX_FIFO1=1,CAN_FILTER_FIFO0=0,CAN_FILTER_FIFO1=1;
constexpr uint32_t CAN_FILTERMODE_IDMASK=0,CAN_FILTERSCALE_32BIT=1,ENABLE=1;
constexpr uint32_t CAN_IT_RX_FIFO0_MSG_PENDING=1,CAN_IT_RX_FIFO1_MSG_PENDING=2;
extern CAN_HandleTypeDef hcan1,hcan2;
extern "C" {
// 模拟滤波器配置，检查共享 CAN2 是否正确使用 FIFO1/bank14。
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *,CAN_FilterTypeDef *);
// 模拟读取当前 CAN 状态，不访问硬件。
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *);
// 模拟启动 CAN，允许注入启动失败。
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
// 模拟恢复接收通知，并记录是否经过回调末尾。
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *,uint32_t);
// 返回测试设置的发送邮箱余量。
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *);
// 记录发送帧内容/ID，或返回预设发送失败。
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *,CAN_TxHeaderTypeDef *,uint8_t *,uint32_t *);
// 返回测试注入的接收帧和头部信息。
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *,uint32_t,CAN_RxHeaderTypeDef *,uint8_t *);
// 测试空处理函数，生产错误处理不会在模拟器中运行。
void Error_Handler();
// 真实 CAN1 回调入口，供测试验证协议不从 CAN1 接收。
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
// 真实 CAN2 回调入口，供测试验证格式和 ID 路由。
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *);
}
