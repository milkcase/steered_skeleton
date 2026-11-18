#include "pitch_feedback_link.hpp"
#include "test_support.hpp"
// 用不同上电时钟发送整组，验证接收方只用自己的时间计龄。
bool transfer(pitch_feedback::Link &source,pitch_feedback::Link &sink,uint32_t tx,uint32_t rx) {
  for(int i=0;i<pitch_feedback::kPages;++i) {
    uint8_t f[8];source.frame(f,tx+2*i);if(!sink.receive(f,rx+2*i))return false;source.sent();
  }
  return true;
}
// 验证完整同周期float、分页原子性、坏帧/缺页/超时/序号循环和非有限值拒绝。
extern "C" int run_tests() {
  pitch_feedback::Link source,sink;uint8_t f[8];
  CHECK(!sink.read(0).valid && sink.age(0)==0xffffffffu);
  pitch_feedback::Sample s{-0.234567f,1.234567f,-0.67845f,true};
  s.position_target=10000.123f;s.speed_target=-12.34567f;s.torque_command=1.23456f;
  s.feedforward=-0.23456f;s.pid_torque=1.46912f;s.position_error=10000.357f;
  s.control=true;s.torque_limited=true;s.feedforward_endpoint=true;
  source.publish(s,100);
  for(int i=0;i<pitch_feedback::kPages;++i) {
    source.frame(f,100+2*i);CHECK(sink.receive(f,5+2*i));source.sent();
    if(i<8)CHECK(!sink.read(5+2*i).valid && sink.count()==0);
    if(i==0)source.publish(0,0,0,false,101); // 发送中local改变不能污染已冻结组。
  }
  auto got=sink.read(21);
  CHECK(got.valid && got.angle==s.angle && got.speed==s.speed && got.torque==s.torque);
  CHECK(got.position_target==s.position_target && got.speed_target==s.speed_target && got.torque_command==s.torque_command);
  CHECK(got.feedforward==s.feedforward && got.pid_torque==s.pid_torque && got.position_error==s.position_error);
  CHECK(got.control && got.torque_limited && got.feedforward_endpoint && sink.count()==1 && sink.age(21)==16);
  CHECK(!sink.read(55).valid);
  source.publish(s,200);source.frame(f,200);f[3]^=1;
  CHECK(!sink.receive(f,60) && !sink.read(60).valid);
  CHECK(transfer(source,sink,200,61) && sink.read(77).valid);
  source.publish(s,300);source.frame(f,300);CHECK(sink.receive(f,80));source.sent();source.sent();
  source.frame(f,304);CHECK(!sink.receive(f,84) && !sink.read(84).valid); // 缺页。
  pitch_feedback::Link timed,receiver;timed.publish(s,0);timed.frame(f,0);CHECK(receiver.receive(f,0));timed.sent();
  timed.frame(f,2);CHECK(!receiver.receive(f,30) && receiver.count()==0); // 拼包超时。
  pitch_feedback::Link invalid,invalid_sink;
  invalid.publish(s,100);CHECK(transfer(invalid,invalid_sink,105,200) && !invalid_sink.read(216).valid); // 本地过期。
  s.torque=__builtin_nanf("");invalid.publish(s,200);
  CHECK(transfer(invalid,invalid_sink,200,300) && !invalid_sink.read(316).valid);
  pitch_feedback::Link loop,loop_sink;uint8_t old_first[8]{};
  s.torque=0;
  for(int i=0;i<270;++i) {
    const uint32_t now=1000+i*20;loop.publish(s,now);
    if(i==268)loop.frame(old_first,now);
    CHECK(transfer(loop,loop_sink,now,now+1373) && loop_sink.read(now+1389).valid);
  }
  CHECK(!loop_sink.receive(old_first,1000+269*20+1390)); // 新鲜窗口内倒序重放。
  CHECK(loop_sink.count()==270);
  return 0;
}
