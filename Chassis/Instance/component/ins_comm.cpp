/**
 *******************************************************************************
 * @file      :ins_comm.cpp
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
#include "ins_comm.hpp"
#include "dbus_uart.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::remote_control::DT7 *GetRemoteControl() {
  /* TODO【新人任务】:
   * 返回遥控器解码对象（DT7）单例。
   *
   * 你要做：
   *   1. 静态定义一个 hello_world::remote_control::DT7 对象并返回其地址。
   *   2. （可选）如果你自建了其它需要共享的通信对象，也在此暴露访问器。
   *
   * 约束 / 提示：
   *   - DT7 只负责解码 18 字节 DBUS，串口收发在 Task/component/comm_task.cpp 完成。
   *   - 解码入口：Task/component/comm_task.cpp 的 HAL_UARTEx_RxEventCallback。
   */
  return GetRemoteControlReceiver()->raw();
}

/* Private function definitions ----------------------------------------------*/

// 两板各自拥有独立对象；实际只接云台 USART3 时，底盘本地接收一直保持无效。
remote_input::DbusReceiver *GetRemoteControlReceiver() {
  static remote_input::DbusReceiver receiver;
  return &receiver;
}

bool ReadRemoteControl(hello_world::remote_control::DT7 &snapshot) {
  // 同时保护有效期判断和整份数据复制，避免读取到新旧帧混合的控制量。
  remote_input::InterruptLock lock;
  return GetRemoteControlReceiver()->read(snapshot, HAL_GetTick());
}
