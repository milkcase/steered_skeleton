#include "debug_drive_test.cpp"
// 真实三页协议下仅重启底盘，云台保留旧会话/接收序号；完整右下和重新准备后必须恢复。
extern "C" int run_tests() {
  for(uint32_t phase=0;phase<3;++phase) {
  test_ms=0;Bench b;b.prepare();for(int i=0;i<120;++i)b.tick(1,0.5f);
  CHECK(b.chassis.output().control);
  b.chassis=Power(true,true);b.c=Comm(Comm::CodePart::Chassis,0x210,0x211);
  b.chassis_tx_phase=phase;
  b.chassis_clock_offset=test_ms; // 只把底盘本地毫秒数归零，云台时间和旧接收序号保持。
  b.chassis_imu=false;
  for(int i=0;i<2800;++i) {b.tick(2,0);CHECK(b.chassis.command()==0);}
  b.chassis_imu=true;
  for(int i=0;i<600;++i)b.tick(2,0);
  for(int i=0;i<350;++i)b.tick(3,0);
  for(int i=0;i<70;++i)b.tick(1,0);
  for(int i=0;i<100;++i)b.tick(1,0.5f);
  CHECK(b.chassis.output().control);
  }
  return 0;
}
