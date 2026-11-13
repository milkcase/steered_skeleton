#ifndef STEERED_WATCHDOG_BOARD_HPP_
#define STEERED_WATCHDOG_BOARD_HPP_
#include "ins_motor.hpp"
#include "watchdog_outputs.hpp"
#include "can.h"
// 只检查本板负责的电机：非控制阶段必须真实零输入，未知配置空对象不会被强行使能。
inline bool WatchdogBoardInputsSafe(bool controlling) {
  for(int i=0;i<4;++i) {
    if(!robot_watchdog::inputSafe(GetMotorWheel(i),controlling) ||
        !robot_watchdog::inputSafe(GetMotorSteer(i),controlling))return false;
  }
  return robot_watchdog::inputSafe(GetMotorYaw(),controlling);
}
// 保护时撤销本板可发送的旧邮箱；共享 CAN2 的另一板仍独立运行自身保护。
inline void WatchdogAbortBoardTx() {
  HAL_CAN_AbortTxRequest(&hcan1,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
  HAL_CAN_AbortTxRequest(&hcan2,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
}
#endif
