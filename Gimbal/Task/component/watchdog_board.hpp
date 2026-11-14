#ifndef STEERED_WATCHDOG_BOARD_HPP_
#define STEERED_WATCHDOG_BOARD_HPP_
#include "ins_motor.hpp"
#include "watchdog_outputs.hpp"
#include "can.h"
// 只检查本板负责的电机：非控制阶段必须真实零输入，未知配置空对象不会被强行使能。
inline bool WatchdogBoardInputsSafe(bool controlling) {
  return robot_watchdog::inputSafe(GetMotorPitch(),controlling);
}
// 保护时撤销本板可发送的旧邮箱；共享 CAN2 的另一板仍独立运行自身保护。
inline void WatchdogAbortBoardTx() {
  // 最新拓扑更正：CAN1 为板间通信，CAN2 为独立 pitch；看门狗保护须撤销两路。
  HAL_CAN_AbortTxRequest(&hcan1,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
  HAL_CAN_AbortTxRequest(&hcan2,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
}
#endif
