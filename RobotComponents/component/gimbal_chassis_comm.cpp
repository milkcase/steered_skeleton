/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
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
#include "gimbal_chassis_comm.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm::GimbalChassisComm(robot::GimbalChassisComm::CodePart code_part,
                                            uint32_t tx_id, uint32_t rx_id) {
  /* TODO【新人任务】:
   * 保存 code_part / tx_id / rx_id，并把 last_rx_tick_ 初始化为当前 GetTickMs()。
   */
  code_part_ = code_part;
  tx_id_ = tx_id;
  rx_id_ = rx_id;
  last_rx_tick_ = hello_world::tick::GetTickMs();
}

void robot::GimbalChassisComm::encode(uint8_t tx_data[8]) {
  /* TODO【新人任务】:
   * 编码己方帧：把本地 ChassisData / GimbalData 按 hpp 里定的字段布局打包进 8 字节。
   *   - 由 code_part_ 决定打包哪个方向（Chassis → ChassisData，Gimbal → GimbalData）。
   *   - 浮点量按约定比例缩放成整数，避免丢位；收发两端比例必须一致。
   *   - 打包完成后由 Task/component/comm_task.cpp 用 tx_id_ 发送。
   */
  (void)tx_data;
}

void robot::GimbalChassisComm::decode(const uint8_t rx_data[8]) {
  /* TODO【新人任务】:
   * 解码对方帧：把 8 字节按同样布局还原成对方语义，更新本地缓存，并刷新 last_rx_tick_。
   *   - 严格与 encode() 对称；任何一端偏移写错都会让对面读到错乱数据。
   *   - 建议先写自测：同一结构 encode 后再 decode 应能还原原值。
   */
  (void)rx_data;
}

bool robot::GimbalChassisComm::isOffline() const {
  /* TODO【新人任务】:
   * 用 GetTickMs() 与 last_rx_tick_ 的差值判断对方是否离线（超过阈值即离线）。
   * 阈值可自定义（如 100ms），供状态机 / 看门狗判断对方是否还活着。
   */
  return false;
}

/* Private function definitions ----------------------------------------------*/
