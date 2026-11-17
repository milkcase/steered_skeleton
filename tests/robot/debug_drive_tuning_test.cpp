#include "debug_drive_test.cpp"
// 验证持续调参、新协议隔离、长于500ms、回中再启动及所有真实故障仍停止。
extern "C" int run_tests() {
  CHECK(drive::kTuning && mode::kResumable && mode::kTarget==19 && mode::kProfile==0xD101);
  Bench b;b.prepare();
  for(int i=0;i<1100;++i)b.tick(1,0.5f);
  CHECK(b.chassis.output().control && b.chassis.command()>0);
  CHECK(std::fabs(b.chassis.command())<=drive::kCurrentA);
  b.angle=12.0f;b.tick(1,0.5f);CHECK(b.chassis.output().control);
  b.velocity=0.8f;b.tick(1,0.5f);CHECK(b.chassis.command()<0);
  b.velocity=0;
  for(int i=0;i<35;++i)b.tick(1,0);
  CHECK(!b.chassis.output().control && b.chassis.command()==0);
  for(int i=0;i<80;++i)b.tick(1,-0.5f);
  CHECK(b.chassis.output().control && b.chassis.command()<0 && b.chassis.diagnostics().run_count==2);
  auto bad=b.gs;bad.target=18;CHECK(!b.chassis.stillSafe(b.ci,bad));
  // 非有限反馈必须停机，并锁存具体样本失败位；右下及后续正常反馈不能覆盖首次失败。
  test_ms=0;Bench invalid;invalid.prepare();for(int i=0;i<80;++i)invalid.tick(1,0.5f);
  invalid.angle=std::numeric_limits<float>::quiet_NaN();invalid.tick(1,0.5f);
  const auto stopped=invalid.chassis.diagnostics();
  CHECK(!invalid.chassis.output().control && invalid.chassis.command()==0 && stopped.last_stop==Reason::Sample);
  CHECK(stopped.stop_check.count==1 && stopped.stop_check.sample_failures==4 && std::isnan(stopped.stop_check.actual_angle));
  invalid.angle=0;for(int i=0;i<30;++i)invalid.tick(2,0);
  CHECK(invalid.chassis.diagnostics().stop_check.sample_failures==4);
  for(int fault=0;fault<7;++fault) {
    test_ms=0;Bench f;f.prepare();for(int i=0;i<80;++i)f.tick(1,0.5f);
    CHECK(f.chassis.output().control);
    if(fault==0)f.radio=false;if(fault==1)f.feedback=false;if(fault==2)f.imu=false;
    if(fault==3)f.deliver=false;if(fault==4)f.other=false;if(fault==5)f.velocity=drive::kMaxActualSpeed+0.1f;
    for(int i=0;i<120;++i)f.tick(fault==6?2:1,0.5f);
    CHECK(!f.chassis.output().control && f.chassis.command()==0);
    // 条件恢复后仅偏杆不能自动恢复；真实故障必须重新准备。
    f.radio=f.feedback=f.imu=f.deliver=f.other=true;f.velocity=0;
    for(int i=0;i<40;++i)f.tick(1,0.5f);
    CHECK(!f.chassis.output().control);
  }
  return 0;
}
