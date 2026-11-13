#include "can.h"
#include "ins_motor.hpp"
#include "motor_feedback.hpp"
#include "tick.hpp"
#include "support.hpp"
CAN_HandleTypeDef hcan1{1},hcan2{2};CAN_RxHeaderTypeDef injected{};
uint8_t data[8]{};uint32_t now=10,notified=0;HAL_StatusTypeDef rx_status=HAL_OK;
hello_world::motor::Motor wheels[4],steers[4],yaw,pitch;
// 提供独立的可控毫秒数。
uint32_t hello_world::tick::GetTickMs() {return now;}
// 返回测试轮位对象，各自 ID1..4，便于检查路由没有索引错位。
hello_world::motor::Motor *GetMotorWheel(int i) {return &wheels[i];}
// 返回各自 ID1..4 的转向对象。
hello_world::motor::Motor *GetMotorSteer(int i) {return &steers[i];}
// 返回底盘独占轴记录对象。
hello_world::motor::Motor *GetMotorYaw() {return &yaw;}
// 返回云台独占轴记录对象。
hello_world::motor::Motor *GetMotorPitch() {return &pitch;}
// 将注入的帧交给真实回调；允许模拟 HAL 取帧失败。
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *,uint32_t,CAN_RxHeaderTypeDef *h,uint8_t *p) {
  *h=injected;memcpy(p,data,8);return rx_status;
}
// 检查未知/坏帧也经过接收通知恢复路径。
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *,uint32_t) {++notified;return HAL_OK;}
// 保留提供的基础函数可编译，测试不配置真实滤波器。
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *,CAN_FilterTypeDef *) {return HAL_OK;}
// 本测试只验证接收分发，不发送电机命令。
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *,CAN_TxHeaderTypeDef *,uint8_t *,uint32_t *) {return HAL_OK;}
// 不调用硬件错误处理路径。
void Error_Handler() {}
// 注入一个标准 8 字节帧，使用真实 CAN1/CAN2 分发函数。
void inject(uint32_t id,bool can1=false) {
  injected={id,CAN_ID_STD,CAN_RTR_DATA,8};
  if(can1)HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);else HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
}
// 覆盖整车十电机的唯一责任路由、格式过滤以及本地有效期更新。
extern "C" int run_tests() {
  for(int i=0;i<4;++i) {wheels[i].info.id=i+1;steers[i].info.id=i+1;}
  yaw.info.id=1;pitch.info.id=2;data[6]=data[7]=30;
#if defined(TEST_CHASSIS)
  for(int i=0;i<4;++i) {
    inject(0x205+i,true);CHECK(steers[i].decodes==1 && robot_control::MotorFeedbacks().fresh(i+4,now));
    inject(0x201+i);CHECK(wheels[i].decodes==1 && robot_control::MotorFeedbacks().fresh(i,now));
    inject(0x201+i,true);CHECK(wheels[i].decodes==1);
    inject(0x205+i);CHECK(steers[i].decodes==1);
  }
  data[0]=0x11;inject(0x11);CHECK(yaw.decodes==1 && pitch.decodes==0 && robot_control::MotorFeedbacks().axisEnabled(now));
  data[0]=0x12;inject(0x12);CHECK(yaw.decodes==1 && pitch.decodes==0);
  const uint32_t owned=0x11;
#else
  data[0]=0x12;inject(0x12);CHECK(pitch.decodes==1 && yaw.decodes==0 && robot_control::MotorFeedbacks().axisEnabled(now));
  data[0]=0x11;inject(0x11);CHECK(yaw.decodes==0 && pitch.decodes==1);
  for(int i=0;i<4;++i) {inject(0x201+i);inject(0x205+i,true);CHECK(wheels[i].decodes==0 && steers[i].decodes==0);}
  const uint32_t owned=0x12;
#endif
  const int before=yaw.decodes+pitch.decodes;now+=100;
  injected={owned,CAN_ID_STD,CAN_RTR_DATA,7};HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
  injected.DLC=8;injected.IDE=CAN_ID_EXT;HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
  injected.IDE=CAN_ID_STD;injected.RTR=CAN_RTR_REMOTE;HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
  inject(0x333);rx_status=HAL_ERROR;inject(owned);rx_status=HAL_OK;
  CHECK(yaw.decodes+pitch.decodes==before && !robot_control::MotorFeedbacks().axisEnabled(now));
  CHECK(notified>10);
  data[0]=static_cast<uint8_t>(0x80|(owned-0x10));inject(owned);
  CHECK(!robot_control::MotorFeedbacks().fresh(8,now));
  return 0;
}
