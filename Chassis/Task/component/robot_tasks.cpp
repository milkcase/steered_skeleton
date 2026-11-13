#include "robot_tasks.hpp"
#include "robot.hpp"
#include "robot_can_service.hpp"
#include "control_clock.hpp"
#include "ins_robot.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "ins_imu.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "spi.h"
#include "gpio.h"
#include "ins_drive.hpp"

namespace {
bool initialized=false,sensors_started=false;
robot_control::ControlClock clock;
// 返回本板唯一发送层；首次构造不启动硬件、不发送电机帧。
robot::RobotCanService &CanService() {
  static robot::RobotCanService service(GetRobot(),true);return service;
}
}
// 按依赖顺序在前台创建/注册对象；提供的工厂参数和默认停用状态均保留。
void InitializeRobotRuntime() {
  if(initialized)return;
  auto *robot=GetRobot();robot->registerImu(GetImu());
  robot->registerCommunication(GetGimbalChassisComm());
  robot->registerAxis(static_cast<hello_world::motor::DaMiao *>(GetMotorYaw()),GetGimbal(),GetPidMotorYaw());
  robot->registerDrive(GetDriveController());
  hello_world::motor::Motor *wheels[4],*steers[4];
  for(int i=0;i<4;++i) {wheels[i]=GetMotorWheel(i);steers[i]=GetMotorSteer(i);}
  CanService().registerDrive(wheels,steers);
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
  initialized=true;
}
// 仅在对象注册后启动 CAN，供 CommTaskInit 的弱钩子调用，独立 DBUS 测试可不链接本模块。
extern "C" void RobotCommunicationInit() {
  if(initialized)GetRobot()->setTransportReady(CanService().init());
}
// 每次先走 MainTask 决策，再由 CommTask 调用本函数统一发帧；看门狗留给第十项。
extern "C" void RobotCommunicationTick() {
  if(initialized)CanService().tick();
}
// CPU 周期差使用无符号回绕；错过周期不补跑旧命令，异常 dt 由状态机拒绝。
void RunRobotControl() {
  if(!initialized)return;
  const float dt=clock.step(DWT->CYCCNT,HAL_GetTick(),SystemCoreClock,(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)!=0);
  GetRobot()->update(dt);
}
// main 的 USER CODE 主循环调用，BMI088 延时/超时都在可被控制中断抢占的前台执行。
extern "C" void RobotSensorsPoll() {
  if(!initialized)return;
  if(!sensors_started) {
    sensors_started=true;
    hello_world::imu::BMI088HWConfig hw;
    hw.hspi=&hspi1;hw.acc_cs_port=GPIOA;hw.acc_cs_pin=GPIO_PIN_4;
    hw.gyro_cs_port=GPIOB;hw.gyro_cs_pin=GPIO_PIN_0;
    GetRobot()->initializeImu(hw,robot_control::kChassisRotation,robot_control::kChassisImuConfirmed);
  }
  GetRobot()->pollSensors();
}
