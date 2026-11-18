#include "main_tuning_link.hpp"
#include "test_support.hpp"
using namespace main_tuning;
// 创建合法但不依赖任何实车调参宏的测试参数；增益包含自由小数。
Bundle sample() {return {{0.6f,0.6f,1.0f,0.5f,2.0f,5.125f,0,0,0,1.0f,0.23f,0.4f,0,0.4f,0.5f,0.5f,2.0f,-0.419f,0.515f,0.03f,1.0f}};}
// 使用真实线协议发完整组；时间按每页5ms推进，除非调用方明确模拟丢页。
bool group(Source &source,Sink &sink,uint32_t &now) {
  for(int i=0;i<Count+2;++i) {uint8_t frame[8];now+=5;if(!source.frame(frame) || !sink.receive(frame,now))return false;source.sent();}
  return true;
}
// 覆盖完整提交、ACK租约、坏帧/丢页/混包拒绝，以及底盘单独重启换参数。
extern "C" int run_tests() {
  CHECK(checksum("123456789",9)==0xcbf43926u);
  Source source;Sink sink;Bundle bundle;uint32_t hash,now=0;
  source.load(sample());CHECK(!source.ready(now));CHECK(!sink.ready(now));
  CHECK(group(source,sink,now));CHECK(sink.pending(bundle,hash,now));
  CHECK(bundle.value[PositionP]==5.125f);CHECK(hash==source.hash());
  uint8_t ack[8];CHECK(!sink.frame(ack,now)); // 完整接收但尚未安全应用，不能回确认。
  sink.applied(hash);CHECK(sink.frame(ack,now));CHECK(source.receive(ack,now));CHECK(source.ready(now));
  ack[3]^=1;CHECK(!source.receive(ack,now+50));CHECK(!source.ready(now+200));
  CHECK(!sink.ready(now+400));
  CHECK(group(source,sink,now));CHECK(sink.ready(now));CHECK(!sink.pending(bundle,hash,now)); // 同组只续租。
  // 新底盘启动，旧云台不重新构建；新组到来立刻撤销旧配置一致性，接收不等于自动应用。
  Source restarted;auto next=sample();next.value[PositionP]=8.375f;next.value[Vx]=0.8f;restarted.load(next);
  uint8_t frame[8];CHECK(restarted.frame(frame));CHECK(sink.receive(frame,++now));restarted.sent();
  CHECK(!sink.ready(now));CHECK(!sink.frame(ack,now));
  for(int i=0;i<Count+1;++i) {CHECK(restarted.frame(frame));CHECK(sink.receive(frame,++now));restarted.sent();}
  CHECK(sink.pending(bundle,hash,now));CHECK(bundle.value[PositionP]==8.375f);CHECK(!sink.ready(now));
  // 模拟外层右下急停应用后再确认，新底盘才可准备。
  sink.applied(hash);CHECK(sink.frame(ack,now));CHECK(restarted.receive(ack,now));CHECK(restarted.ready(now));
  Source broken;broken.load(sample());Sink clean;uint32_t t=0;
  CHECK(broken.frame(frame));frame[3]^=1;CHECK(!clean.receive(frame,++t));
  CHECK(broken.frame(frame));CHECK(clean.receive(frame,++t));broken.sent();broken.sent(); // 缺参数0页。
  CHECK(broken.frame(frame));CHECK(!clean.receive(frame,++t));CHECK(!clean.pending(bundle,hash,t));
  // 合法帧CRC也不能掩盖整组CRC不符、事务混入或超时。
  broken=Source{};broken.load(sample());clean=Sink{};
  CHECK(broken.frame(frame));CHECK(clean.receive(frame,++t));broken.sent();
  CHECK(broken.frame(frame));frame[3]^=1;frame[7]=frameCrc(frame);CHECK(clean.receive(frame,++t));broken.sent();
  for(int i=1;i<Count;++i) {CHECK(broken.frame(frame));CHECK(clean.receive(frame,++t));broken.sent();}
  CHECK(broken.frame(frame));CHECK(!clean.receive(frame,++t));CHECK(!clean.ready(t));
  broken=Source{};broken.load(sample());CHECK(broken.frame(frame));CHECK(clean.receive(frame,++t));broken.sent();
  CHECK(broken.frame(frame));CHECK(!clean.receive(frame,t+181));
  auto bad=sample();bad.value[Torque]=__builtin_nanf("");broken=Source{};broken.load(bad);CHECK(!broken.frame(frame));
  bad=sample();bad.value[TripSpeed]=bad.value[PositionSpeed];broken.load(bad);CHECK(!broken.frame(frame));
  bad=sample();bad.value[PitchLimitEnabled]=2;broken.load(bad);CHECK(!broken.frame(frame));
  bad=sample();bad.value[PitchLimitEnabled]=0;bad.value[PitchMin]=1;bad.value[PitchMax]=-1;
  broken.load(bad);CHECK(broken.frame(frame)); // 关闭后原上下限记录不限制位置，数值有限检查仍保留。
  // 旧20/21字段组分别缺少角度开关/重力前馈字段，不得提交或通过ACK续租。
  const int legacy_counts[2]={PitchLimitEnabled,GravityEnabled};
  for(int legacy_count:legacy_counts) {
    clean=Sink{};auto legacy=sample();const uint32_t legacy_hash=checksum(&legacy,legacy_count*4);
    frame[0]=0xd1;frame[1]=0xfe;frame[2]=1;putWord(frame+3,legacy_hash);frame[7]=frameCrc(frame);
    CHECK(clean.receive(frame,++t));
    for(int i=0;i<legacy_count;++i) {
      frame[1]=i;uint32_t word;memcpy(&word,&legacy.value[i],4);putWord(frame+3,word);frame[7]=frameCrc(frame);
      CHECK(clean.receive(frame,++t));
    }
    frame[1]=0xff;putWord(frame+3,legacy_hash);frame[7]=frameCrc(frame);CHECK(!clean.receive(frame,++t));
    CHECK(!clean.pending(bundle,hash,t));
    broken.load(sample());frame[0]=0xd2;frame[1]=legacy_count;frame[2]=1;putWord(frame+3,broken.hash());frame[7]=frameCrc(frame);
    CHECK(!broken.receive(frame,t) && !broken.ready(t));
  }
  return 0;
}
