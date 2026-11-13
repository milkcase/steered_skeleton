#ifndef STEERED_WATCHDOG_TASK_IMPL_HPP_
#define STEERED_WATCHDOG_TASK_IMPL_HPP_
#include "watchdog_task.hpp"
#include "watchdog_supervisor.hpp"
#include "robot.hpp"
#include "ins_robot.hpp"
#include <dbus_uart.hpp>
#include "iwdg.h"
extern "C" void RobotCommunicationTick();
extern "C" {
volatile RobotWatchdogDiagnostics robot_watchdog_diagnostics{};
}
namespace {
robot_watchdog::Supervisor watchdog;
uint32_t watchdog_reset_flags=0,watchdog_iwdg_reset=0;
bool watchdog_protected=false;
// 更新调试器可见镜像，同时让诊断读取入口保留在最终链接中；该镜像不参与控制决策。
void WatchdogPublishDiagnostics() {
  const auto d=ReadRobotWatchdogDiagnostics();
  robot_watchdog_diagnostics.successful_feeds=d.successful_feeds;
  robot_watchdog_diagnostics.last_feed_ms=d.last_feed_ms;
  robot_watchdog_diagnostics.fault=d.fault;
  robot_watchdog_diagnostics.completed_controls=d.completed_controls;
  robot_watchdog_diagnostics.completed_comms=d.completed_comms;
  robot_watchdog_diagnostics.reset_flags=d.reset_flags;
  robot_watchdog_diagnostics.reset_from_iwdg=d.reset_from_iwdg;
}
// 故障只执行一次锁住输出/取消旧邮箱/发零帧，避免每个回调重复挤占总线。
void WatchdogProtect() {
  if(watchdog_protected)return;
  watchdog_protected=true;
  auto *robot=GetRobot();
  if(robot!=nullptr)robot->setTransportReady(false);
  WatchdogPublishDiagnostics();
  WatchdogAbortBoardTx();
  if(robot!=nullptr)RobotCommunicationTick();
}
// 普通遥控/反馈异步失效走正常归零，不把急停本身误判成必须复位的软件故障。
bool WatchdogOutputsSafe() {
  auto *robot=GetRobot();if(robot==nullptr)return false;
  if(robot->output().control && !robot->controlStillSafe()) {robot->stop();RobotCommunicationTick();}
  const auto &out=robot->output();
  if(out.control && out.state.life!=::robot::LifeState::Working)return false;
  return WatchdogBoardInputsSafe(out.control);
}
}
// 保存并清除硬件复位标志，让下一次复位原因可独立判断；初始化阶段不刷新 IWDG。
extern "C" void RobotWatchdogInit() {
  remote_input::InterruptLock lock;
  watchdog_reset_flags=RCC->CSR;watchdog_iwdg_reset=__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)!=RESET;
  __HAL_RCC_CLEAR_RESET_FLAGS();watchdog_protected=false;
  if(!watchdog.init(DWT->CYCCNT,HAL_GetTick(),SystemCoreClock,(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)!=0))WatchdogProtect();
  WatchdogPublishDiagnostics();
}
// 仅记录主循环进度；即使传感器尚未就绪，也必须由真正运行的前台产生心跳。
extern "C" void RobotWatchdogForeground() {
  remote_input::InterruptLock lock;watchdog.foreground(DWT->CYCCNT);
}
// 在 RunRobotControl 前检查顺序与时钟；故障后锁住设备，后续控制不能重新放行。
extern "C" void RobotWatchdogControlBegin() {
  if(!watchdog.controlBegin(DWT->CYCCNT,HAL_GetTick(),SystemCoreClock,(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)!=0))WatchdogProtect();
}
// 控制返回后检查执行时长，不能在控制尚未完成时留下可续用的喂狗凭据。
extern "C" void RobotWatchdogControlEnd() {
  if(!watchdog.controlEnd(DWT->CYCCNT))WatchdogProtect();
}
// 通讯必须取得本轮新控制记录，重复通讯或控制缺席都停止喂狗。
extern "C" void RobotWatchdogCommBegin() {
  if(!watchdog.commBegin(DWT->CYCCNT))WatchdogProtect();
}
// 在 CommTask 末尾检查完整执行链及输出；这是整份固件唯一的 HAL_IWDG_Refresh 调用点。
extern "C" void RobotWatchdogCommEnd() {
  const bool safe=WatchdogOutputsSafe();
  const bool feed=watchdog.commEnd(DWT->CYCCNT,HAL_GetTick(),safe,robot_watchdog::kRefreshEnabled);
  if(watchdog.fault()!=robot_watchdog::Fault::None) {WatchdogProtect();WatchdogPublishDiagnostics();return;}
  if(feed) {
    if(HAL_IWDG_Refresh(&hiwdg)==HAL_OK)watchdog.refreshed(HAL_GetTick());
    else {watchdog.refreshFailed();WatchdogProtect();}
  }
  WatchdogPublishDiagnostics();
}
// 读取锁存故障、执行次数和真实刷新次数；返回值仅供诊断，不作为运动许可。
extern "C" RobotWatchdogDiagnostics ReadRobotWatchdogDiagnostics() {
  remote_input::InterruptLock lock;
  return {watchdog.feeds(),watchdog.lastFeedMs(),static_cast<uint32_t>(watchdog.fault()),
      watchdog.controlCount(),watchdog.commCount(),watchdog_reset_flags,watchdog_iwdg_reset};
}
#endif
