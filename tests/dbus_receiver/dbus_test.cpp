#include "ins_comm.hpp"
#include "comm_task.hpp"
#include "usart.h"
#include <cstddef>
#include <initializer_list>

// 在 ARM 模拟器中运行真实接收层和真实 DT7 解码器，仅替换 HAL 外设调用。
uint32_t test_mask = 0, now_ms = 0;
DMA_HandleTypeDef dma;
UART_HandleTypeDef huart3{&dma, 0, HAL_UART_RXEVENT_IDLE};
uint8_t *rx_buffer = nullptr;
uint16_t capacity = 0;
int starts = 0, aborts = 0;
HAL_StatusTypeDef start_result = HAL_OK, abort_result = HAL_OK;
uint32_t HAL_GetTick() { return now_ms; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *) { ++aborts; return abort_result; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *p, uint16_t size) {
  ++starts; rx_buffer = p; capacity = size; return start_result;
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
extern "C" void *memset(void *p, int v, std::size_t n) {
  auto *q = static_cast<uint8_t *>(p); for (std::size_t i=0;i<n;++i) q[i]=v; return p;
}
extern "C" void *memcpy(void *p, const void *s, std::size_t n) {
  auto *q=static_cast<uint8_t *>(p); const auto *r=static_cast<const uint8_t *>(s);
  for (std::size_t i=0;i<n;++i) q[i]=r[i]; return p;
}

void frame(uint8_t *p, uint16_t c0=1024, uint16_t c1=1024, uint16_t c2=1024,
           uint16_t c3=1024, uint16_t wheel=1024, uint8_t left=2, uint8_t right=2) {
  memset(p,0,18);
  const uint64_t bits=static_cast<uint64_t>(c0) | (static_cast<uint64_t>(c1)<<11) |
      (static_cast<uint64_t>(c2)<<22) | (static_cast<uint64_t>(c3)<<33) |
      (static_cast<uint64_t>(right)<<44) | (static_cast<uint64_t>(left)<<46);
  for(int i=0;i<6;++i) p[i]=bits>>(8*i);
  p[16]=wheel&255; p[17]=wheel>>8;
}
void feed(const uint8_t *p, uint16_t size=18, HAL_UART_RxEventTypeTypeDef event=HAL_UART_RXEVENT_IDLE) {
  memcpy(rx_buffer,p,size>18?18:size); huart3.event=event;
  HAL_UARTEx_RxEventCallback(&huart3,size);
}
#define CHECK(x) do { if (!(x)) return __LINE__; } while(0)
bool near(float a, float b) { return a-b < 0.00001f && b-a < 0.00001f; }

extern "C" int run_tests() {
  using hello_world::remote_control::DT7;
  DT7 snapshot;
  uint8_t data[18]; frame(data);
  CHECK(!ReadRemoteControl(snapshot));
  CommTaskInit(); CHECK(capacity==36 && starts==1 && test_mask==0);
  feed(data); CHECK(ReadRemoteControl(snapshot)); CHECK(snapshot.rc_lv()==0 && snapshot.rc_wheel()==0);
  CHECK(starts==2 && aborts==1);
  // 所有摇杆与拨轮边界均由真实解码器还原，键鼠字段同时保持一致。
  frame(data,364,1684,1684,364,1684,1,3); data[6]=17; data[12]=1; data[14]=1;
  feed(data); CHECK(ReadRemoteControl(snapshot));
  CHECK(near(snapshot.rc_rh(),-1) && near(snapshot.rc_rv(),1) && near(snapshot.rc_lh(),1) && near(snapshot.rc_lv(),-1));
  CHECK(near(snapshot.rc_wheel(),1) && snapshot.rc_l_switch()==1 && snapshot.rc_r_switch()==3);
  CHECK(snapshot.mouse_x()==17 && snapshot.mouse_l_btn() && snapshot.key_W());
  now_ms=99; CHECK(ReadRemoteControl(snapshot));
  now_ms=100; CHECK(!ReadRemoteControl(snapshot));
  CHECK(snapshot.rc_wheel()==0 && !snapshot.mouse_l_btn() && !snapshot.key_W());
  // 短帧、长帧、循环 DMA 满容量事件均不能沿用上一条有效输入。
  frame(data); feed(data); feed(data,17); CHECK(!ReadRemoteControl(snapshot));
  feed(data); feed(data,19); CHECK(!ReadRemoteControl(snapshot));
  feed(data); feed(data,36,HAL_UART_RXEVENT_TC); CHECK(!ReadRemoteControl(snapshot));
  feed(data); const int count=starts; feed(data,18,HAL_UART_RXEVENT_HT);
  CHECK(starts==count && ReadRemoteControl(snapshot));
  // 任一通道超界、拨轮高字节异常、拨杆为零或鼠标键异常均立即拒绝。
  for (int channel=0;channel<5;++channel) {
    for (uint16_t bad : {static_cast<uint16_t>(363),static_cast<uint16_t>(1685)}) {
      uint16_t c[5]={1024,1024,1024,1024,1024}; c[channel]=bad;
      frame(data,c[0],c[1],c[2],c[3],c[4]); feed(data); CHECK(!ReadRemoteControl(snapshot));
    }
  }
  frame(data); data[17]=255; feed(data); CHECK(!ReadRemoteControl(snapshot));
  frame(data,1024,1024,1024,1024,1024,0,2); feed(data); CHECK(!ReadRemoteControl(snapshot));
  frame(data,1024,1024,1024,1024,1024,2,0); feed(data); CHECK(!ReadRemoteControl(snapshot));
  frame(data); data[12]=2; feed(data); CHECK(!ReadRemoteControl(snapshot));
  frame(data); data[13]=2; feed(data); CHECK(!ReadRemoteControl(snapshot));
  // 不相关串口的事件不改变 DBUS 输入；串口错误立即失效并重启接收。
  frame(data); feed(data); UART_HandleTypeDef other{&dma,0,HAL_UART_RXEVENT_IDLE};
  const int before=starts; HAL_UARTEx_RxEventCallback(&other,18); HAL_UART_ErrorCallback(&other);
  CHECK(starts==before && ReadRemoteControl(snapshot));
  huart3.ErrorCode=1; HAL_UART_ErrorCallback(&huart3);
  CHECK(!ReadRemoteControl(snapshot) && huart3.ErrorCode==0);
  feed(data); huart3.ErrorCode=1; feed(data); CHECK(!ReadRemoteControl(snapshot));
  huart3.ErrorCode=0;
  // DMA 启动失败不得保留有效状态，下一次任务允许恢复，但不恢复旧指令。
  start_result=HAL_BUSY; feed(data); CHECK(!ReadRemoteControl(snapshot));
  start_result=HAL_OK; CommTask(); CHECK(!ReadRemoteControl(snapshot));
  feed(data); CHECK(ReadRemoteControl(snapshot));
  abort_result=HAL_TIMEOUT; feed(data); CHECK(!ReadRemoteControl(snapshot));
  abort_result=HAL_OK; CommTask(); feed(data); CHECK(ReadRemoteControl(snapshot));
  // 时间回绕以及原有中断屏蔽状态必须保持正确。
  now_ms=0xFFFFFFF0; feed(data); now_ms=0x30; CHECK(ReadRemoteControl(snapshot));
  now_ms=0x54; CHECK(!ReadRemoteControl(snapshot));
  test_mask=1; CHECK(!ReadRemoteControl(snapshot)); CHECK(test_mask==1); test_mask=0;
  // 断线后的任务清理也会更新骨架原有 raw getter 的缓存。
  now_ms=200; frame(data,1684); feed(data); now_ms=300; CommTask();
  CHECK(!ReadRemoteControl(snapshot) && GetRemoteControl()->rc_rh()==0);
  return 0;
}
