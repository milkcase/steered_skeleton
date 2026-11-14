#include "watchdog_task.hpp"
#include "watchdog_config.hpp"
#include "watchdog_supervisor.hpp"
#include "ins_robot.hpp"
#include "ins_motor.hpp"
#include "dbus_uart.hpp"
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "main_task.hpp"
#include "comm_task.hpp"
#include "support.hpp"
#include <limits>
DWT_Type test_dwt;RCC_Type test_rcc;
uint32_t SystemCoreClock=168000000,test_ms=0,irq_mask=0,aborts=0,stage=0,uart_inits=0;
uint32_t refresh_attempts=0,control_calls=0,comm_calls=0,init_calls=0;
uint32_t abort_mask=0;
bool wrong_feed_order=false;
HAL_StatusTypeDef refresh_result=HAL_OK;
IWDG_HandleTypeDef hiwdg;CAN_HandleTypeDef hcan1{1},hcan2{2};
TIM_HandleTypeDef htim6,htim2;UART_HandleTypeDef huart3;
TestMotor wheels[4],steers[4],yaw,pitch;
robot::Robot robot_instance;
remote_input::Receiver receiver;
// 真实板级输入检查使用同一组测试对象。
TestMotor *GetMotorWheel(int i) {return &wheels[i];}
// 返回测试舵电机缓存。
TestMotor *GetMotorSteer(int i) {return &steers[i];}
// 返回底盘本地轴缓存。
TestMotor *GetMotorYaw() {return &yaw;}
// 返回云台本地轴缓存。
TestMotor *GetMotorPitch() {return &pitch;}
// 返回记录型 Robot，真实看门狗任务必须调用它的保护接口。
robot::Robot *GetRobot() {return &robot_instance;}
// 为真实 CommTask 的原 DBUS 维护提供接收对象。
remote_input::Receiver *GetRemoteControlReceiver() {return &receiver;}
// 返回当前注入时间。
extern "C" uint32_t HAL_GetTick() {return test_ms;}
// 记录 HAL 刷新必须位于控制及通信结束之后，支持失败注入。
extern "C" HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *handle) {
  ++refresh_attempts;if(stage!=3 || handle!=&hiwdg)wrong_feed_order=true;return refresh_result;
}
// 记录保护是否撤销原有邮箱。
extern "C" HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can,uint32_t) {
  abort_mask|=can==&hcan1?1:2;++aborts;return HAL_OK;
}
// 真实 MainTaskInit 仍按原顺序完成本板注册，再初始化通信与看门狗。
void InitializeRobotRuntime() {++init_calls;test_dwt.CTRL=1;}
// 只替代完整 Robot 计算，不替代待验证的任务薄层和看门狗，模拟少量执行时间。
void RunRobotControl() {++control_calls;stage=1;test_dwt.CYCCNT+=10000;}
// 模拟整机通信启动，真实 CommTaskInit 的 DBUS 初始化仍保留。
extern "C" void RobotCommunicationInit() {}
// 记录真实 CommTask 已执行通信，保护额外零帧刷新也经过同一接口。
extern "C" void RobotCommunicationTick() {++comm_calls;stage=3;test_dwt.CYCCNT+=1000;}
// 声明真实任务回调，用句柄注入而非启动硬件定时器。
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);

// 重建测试启动条件；本函数只重置模拟对象，不声称模拟了真实芯片复位。
void resetTest() {
  robot_instance=robot::Robot{};robot_instance.stop();robot_instance.transport=true;robot_instance.safe=true;
  test_dwt.CYCCNT=0;test_dwt.CTRL=1;SystemCoreClock=168000000;test_ms=0;irq_mask=0;
  aborts=stage=refresh_attempts=0;wrong_feed_order=false;refresh_result=HAL_OK;
  abort_mask=0;
  RobotWatchdogInit();
}
// 运行真实 TIM6 分发；可独立冻结 HAL 时间或停止前台心跳。
void tick(uint32_t cpu_ms,bool foreground=true,int32_t hal_ms=-1) {
  test_ms=hal_ms<0?cpu_ms:static_cast<uint32_t>(hal_ms);test_dwt.CYCCNT=cpu_ms*168000;
  if(foreground)RobotWatchdogForeground();
  HAL_TIM_PeriodElapsedCallback(&htim6);
}
// 验证两板实际任务接线、唯一 HAL 刷新点、复位诊断与故障保护；不触发物理看门狗。
extern "C" int run_tests() {
  test_rcc.CSR=RCC_FLAG_IWDGRST|0x100;
  MainTaskInit();CHECK(init_calls==1 && uart_inits==1);
  auto d=ReadRobotWatchdogDiagnostics();CHECK(d.reset_from_iwdg==1 && d.reset_flags==(RCC_FLAG_IWDGRST|0x100));
  CHECK(test_rcc.CSR==0 && refresh_attempts==0);
  HAL_TIM_PeriodElapsedCallback(&htim2);CHECK(control_calls==0 && refresh_attempts==0);
#if !ROBOT_WATCHDOG_REFRESH_ENABLED
  tick(1);d=ReadRobotWatchdogDiagnostics();
  CHECK(d.fault==9 && refresh_attempts==0 && !robot_instance.transport && aborts>0);
  CHECK(abort_mask==3); // 两板停止喂狗保护均须撤销两个本地 CAN 的旧邮箱。
  for(uint32_t i=2;i<30;++i)tick(i);
  CHECK(refresh_attempts==0 && ReadRobotWatchdogDiagnostics().fault==9);
  return 0;
#else
  for(uint32_t i=1;i<=1000;++i)tick(i);
  d=ReadRobotWatchdogDiagnostics();
  CHECK(d.fault==0 && d.successful_feeds==50 && refresh_attempts==50 && !wrong_feed_order);
  CHECK(d.completed_controls==1000 && d.completed_comms==1000 && robot_instance.transport);
  CHECK(robot_watchdog_diagnostics.successful_feeds==50 && robot_watchdog_diagnostics.fault==0);
  CHECK(robot_instance.output().state.life==robot::LifeState::Dead); // 无遥控/未配置的安全空闲也能正常喂狗。
  irq_mask=1;ReadRobotWatchdogDiagnostics();CHECK(irq_mask==1);irq_mask=0;
  resetTest();tick(1);const auto before=refresh_attempts;
  for(uint32_t i=2;i<=101;++i)tick(i,false);
  CHECK(ReadRobotWatchdogDiagnostics().fault==6 && !robot_instance.transport && aborts>0);
  CHECK(abort_mask==3);
  const auto failed=refresh_attempts;tick(102,true);CHECK(refresh_attempts==failed && failed>=before);
  resetTest();tick(1);test_ms=2;test_dwt.CYCCNT=2*168000;CommTask();
  CHECK(ReadRobotWatchdogDiagnostics().fault==3 && refresh_attempts==1 && !robot_instance.transport);
  resetTest();tick(1);tick(2,true,1);tick(3,true,1);tick(4,true,1);
  CHECK(ReadRobotWatchdogDiagnostics().fault==4 && refresh_attempts==1);
  resetTest();refresh_result=HAL_ERROR;tick(1);
  CHECK(ReadRobotWatchdogDiagnostics().fault==8 && ReadRobotWatchdogDiagnostics().successful_feeds==0);
  CHECK(!robot_instance.transport && refresh_attempts==1);tick(2);CHECK(refresh_attempts==1);
  resetTest();
#if defined(TEST_CHASSIS)
  wheels[2].input=1;
#else
  pitch.input=1;
#endif
  tick(1);CHECK(ReadRobotWatchdogDiagnostics().fault==7 && !robot_instance.transport && refresh_attempts==0);
  CHECK(wheels[2].input==0 && pitch.input==0);
  resetTest();
#if defined(TEST_CHASSIS)
  yaw.input=std::numeric_limits<float>::quiet_NaN();
#else
  pitch.input=std::numeric_limits<float>::infinity();
#endif
  tick(1);CHECK(ReadRobotWatchdogDiagnostics().fault==7 && refresh_attempts==0);
  resetTest();robot_instance.out.control=true;robot_instance.out.state.life=robot::LifeState::Working;
  robot_instance.safe=false;yaw.input=pitch.input=0.5f;tick(1);
  CHECK(ReadRobotWatchdogDiagnostics().fault==0 && refresh_attempts==1 && robot_instance.transport);
  CHECK(!robot_instance.output().control && yaw.input==0 && pitch.input==0); // 普通急停归零后允许健康任务继续喂狗。
  return 0;
#endif
}
