#ifndef STEERED_WATCHDOG_TASK_HPP_
#define STEERED_WATCHDOG_TASK_HPP_
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  uint32_t successful_feeds;
  uint32_t last_feed_ms;
  uint32_t fault;
  uint32_t completed_controls;
  uint32_t completed_comms;
  uint32_t reset_flags;
  uint32_t reset_from_iwdg;
} RobotWatchdogDiagnostics;
// 调试器可直接观察此只读用途的镜像；控制逻辑不从该全局诊断量读取动作许可。
extern volatile RobotWatchdogDiagnostics robot_watchdog_diagnostics;
// 记录复位原因并初始化策略，不修改提供的 IWDG 配置，也不在此喂狗。
void RobotWatchdogInit(void);
// 前台主循环心跳，传感器读取前后均可调用，不读取硬件传感器。
void RobotWatchdogForeground(void);
// 控制开始标记，检测真实周期和顺序。
void RobotWatchdogControlBegin(void);
// 控制完成标记，只有函数返回后才记录。
void RobotWatchdogControlEnd(void);
// 通讯开始标记，要求已有本轮控制完成记录。
void RobotWatchdogCommBegin(void);
// 通讯完成及唯一喂狗入口，必须位于 CommTask 的全部工作之后。
void RobotWatchdogCommEnd(void);
// 原子读取调试诊断，不产生动作、不清除锁存故障。
RobotWatchdogDiagnostics ReadRobotWatchdogDiagnostics(void);
#ifdef __cplusplus
}
#endif
#endif
