#pragma once
#include "module_state.hpp"
#include "ins_motor.hpp"
namespace robot {
struct TestOutput {ModuleState state{};bool control=false;};
class Robot {
 public:
  TestOutput out{};bool transport=true,safe=true;
  // 提供当前动作状态，允许注入“非 Working 却有输出”的软件故障。
  const TestOutput &output() const {return out;}
  // 模拟异步急停/离线时最终检查失败。
  bool controlStillSafe() {return safe;}
  // 模拟真实统一停止语义，归零本板全部记录输入。
  void stop() {
    out={};yaw.input=pitch.input=0;
    for(int i=0;i<4;++i)wheels[i].input=steers[i].input=0;
  }
  // 锁住输出来响应看门狗故障，与普通急停停止区分。
  void setTransportReady(bool ready) {transport=ready;if(!ready)stop();}
};
}
