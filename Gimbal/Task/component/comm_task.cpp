/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"
#include "dbus_uart.hpp"
#include "ins_comm.hpp"
#include "usart.h"

namespace {
// 在启用接收前创建对象，回调只使用 USART3，不处理 CAN 或其他串口。
// 返回绑定本板 USART3 和接收单例的 UART 适配器；创建对象不启动接收。
remote_input::DbusUart &RemoteUart() {
  static remote_input::DbusUart uart(huart3, *GetRemoteControlReceiver());
  return uart;
}
}  // namespace

/* Exported function definitions ---------------------------------------------*/
// 当前实现仅初始化 DBUS 接收；由 main 在串口和 DMA 初始化后调用。
void CommTaskInit(void) {
  /* TODO【新人任务】:
   * 通讯层一次性初始化：
   *   1. 初始化 CAN 滤波器：inc/HW_can.hpp 的 CanFilter_Init(&hcan1) / CanFilter_Init(&hcan2)。
   *   2. 启动遥控器串口空闲 + DMA 接收（18 字节 DBUS，见 inc/DT7.hpp 注释）：
   *      调用 HAL_UARTEx_ReceiveToIdle_DMA 并记录接收缓冲区。
   *   3. 启动 / 刷新独立看门狗 IWDG（inc/iwdg.h）。
   */
  // 本项只接通 DBUS 接收，不发送电机命令，不使能电机，也不提前加入喂狗。
  RemoteUart().init();
}

// 当前实现仅维护 DBUS 超时和 DMA 重试，留待后续任务接入周期调度。
void CommTask(void) {
  /* TODO【新人任务】:
   * 每个通讯周期（1ms）调用一次，把各模块 encode 出的 8 字节 CAN 帧发出去：
   *   1. 遍历电机：调用 encode(data) 得到 8 字节 + tx_id()，再 CAN_Send_Msg 发出。
   *   2. 双板通讯对象 encode 后同样发送。
   *   3. 刷新 IWDG。
   */
  // 后续 1ms 调度接入后负责清除过期数据，并重试失败的 DMA 启动。
  RemoteUart().poll();
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */

// HAL 接收事件入口：只把 USART3 的事件及长度交给 DBUS 适配器，其他串口忽略。
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
  if (huart == &huart3) RemoteUart().onReceive(size);
}

// HAL 串口错误入口：仅使 USART3 的遥控输入失效并尝试恢复接收。
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart3) RemoteUart().onError();
}
