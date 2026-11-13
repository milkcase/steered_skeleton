/**
 *******************************************************************************
 * @file      :main_task.cpp
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
#include "main_task.hpp"
#include "comm_task.hpp"
#include "robot_tasks.hpp"
#include "tim.h"

// 第十项钩子在整机固件中有强定义，独立任务分发测试无需链接硬件看门狗。
extern "C" void RobotWatchdogInit() __attribute__((weak));
extern "C" void RobotWatchdogControlBegin() __attribute__((weak));
extern "C" void RobotWatchdogControlEnd() __attribute__((weak));

/* Exported function definitions ---------------------------------------------*/
// 创建并注册本板模块，再完成通信接入；此处不使能电机或执行回中。
void MainTaskInit(void) {
  /* TODO【新人任务】:
   * 一次性初始化（由 main() 在 while(1) 之前调用一次）：
   *   1. 通过 Instance 层的 GetXxx() 拿到电机 / IMU / 遥控器 / 双板通讯 / Robot 等对象，
   *      按依赖顺序完成初始化（电机使能、IMU 零漂采集、CAN 滤波器等）。
   *   2. 把各模块注册进 Robot（registerXxx）。
   *   3. 调用 CommTaskInit() 完成通讯初始化（UART 接收、IWDG 等）。
   * 注意：不要在这里写任何周期性逻辑。
   */
  InitializeRobotRuntime();
  CommTaskInit();
  if(RobotWatchdogInit!=nullptr)RobotWatchdogInit();
}

// 1 ms 控制入口只转交状态机，不在任务薄层生成电机指令。
void MainTask(void) {
  /* TODO【新人任务】:
   * 1ms 控制周期入口：调用 robot->update()，由状态机完成
   * 「读数据 → 迁移 → 生成指令 → 下发」的整条链路。
   * 保持薄薄一层，不要在这里堆业务逻辑。
   */
  if(RobotWatchdogControlBegin!=nullptr)RobotWatchdogControlBegin();
  RunRobotControl();
  if(RobotWatchdogControlEnd!=nullptr)RobotWatchdogControlEnd();
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 htim6 周期中断回调 HAL_TIM_PeriodElapsedCallback：
 *   - 先判中断来自 htim6（避免把别的定时器中断误当成 1ms tick）；
 *   - 再调用 MainTask() 与 CommTask()。
 * 注意：
 *   - main.c 已 HAL_TIM_Base_Start_IT(&htim6)，这里只需补中断里的分发；
 *   - HAL 回调是 C 链接，需用 extern "C" 包裹，否则中断里找不到符号。
 */
// 仅 TIM6 驱动本板控制和通信，TIM2 等其他定时器中断不得推进机器人周期。
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim==&htim6) {MainTask();CommTask();}
}
