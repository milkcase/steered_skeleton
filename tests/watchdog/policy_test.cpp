#include "watchdog_supervisor.hpp"
#include "support.hpp"
using namespace robot_watchdog;
constexpr uint32_t hz=168000000,per_ms=168000;
// 以真实策略模拟一轮控制后通讯，只有策略发放许可才模拟 HAL 刷新成功。
bool cycle(Supervisor &s,uint32_t ms,bool foreground=true,bool safe=true) {
  const uint32_t t=ms*per_ms;
  if(foreground)s.foreground(t);
  if(!s.controlBegin(t,ms,hz,true) || !s.controlEnd(t+1000) || !s.commBegin(t+1100))return false;
  const bool feed=s.commEnd(t+2000,ms,safe,true);if(feed)s.refreshed(ms);return feed;
}
// 验证完整执行链、前台/时钟、刷新间隔及不可自行恢复的故障锁存；不模拟硬件复位外设。
extern "C" int run_tests() {
  Supervisor s;CHECK(s.init(0,0,hz,true));
  for(uint32_t ms=1;ms<=1000;++ms)cycle(s,ms);
  CHECK(s.fault()==Fault::None && s.feeds()==50 && s.controlCount()==1000 && s.commCount()==1000);
  Supervisor startup;startup.init(0,0,hz,true);CHECK(!cycle(startup,1,false));
  CHECK(startup.fault()==Fault::None);CHECK(cycle(startup,2,true));
  Supervisor stale;stale.init(0,0,hz,true);cycle(stale,1);
  for(uint32_t ms=2;ms<=101;++ms)cycle(stale,ms,false);
  CHECK(stale.fault()==Fault::ForegroundStale);const auto feeds=stale.feeds();
  CHECK(!cycle(stale,102,true) && stale.feeds()==feeds);
  Supervisor order;order.init(0,0,hz,true);CHECK(!order.commBegin(100));CHECK(order.fault()==Fault::NoControl);
  Supervisor nested;nested.init(0,0,hz,true);CHECK(nested.controlBegin(per_ms,1,hz,true));
  CHECK(!nested.controlBegin(2*per_ms,2,hz,true) && nested.fault()==Fault::Order);
  Supervisor no_comm;no_comm.init(0,0,hz,true);cycle(no_comm,1);
  CHECK(no_comm.controlBegin(2*per_ms,2,hz,true) && no_comm.controlEnd(2*per_ms+100));
  CHECK(!no_comm.controlBegin(3*per_ms,3,hz,true));
  Supervisor long_control;long_control.init(0,0,hz,true);long_control.controlBegin(per_ms,1,hz,true);
  CHECK(!long_control.controlEnd(5*per_ms) && long_control.fault()==Fault::PhaseOverrun);
  Supervisor long_combined;long_combined.init(0,0,hz,true);long_combined.foreground(per_ms);
  long_combined.controlBegin(per_ms,1,hz,true);CHECK(long_combined.controlEnd(3*per_ms));
  CHECK(long_combined.commBegin(3*per_ms));CHECK(!long_combined.commEnd(5*per_ms,1,true,true));
  CHECK(long_combined.fault()==Fault::PhaseOverrun);
  Supervisor unsafe;unsafe.init(0,0,hz,true);CHECK(!cycle(unsafe,1,true,false) && unsafe.fault()==Fault::UnsafeOutputs);
  Supervisor frozen;frozen.init(0,0,hz,true);cycle(frozen,1);
  for(uint32_t i=2;i<=4;++i) {
    frozen.foreground(i*per_ms);
    if(frozen.controlBegin(i*per_ms,1,hz,true)) {frozen.controlEnd(i*per_ms+100);frozen.commBegin(i*per_ms+200);frozen.commEnd(i*per_ms+300,1,true,true);}
  }
  CHECK(frozen.fault()==Fault::Clock);
  Supervisor stopped;CHECK(!stopped.init(0,0,hz,false) && stopped.fault()==Fault::Clock);
  Supervisor no_init;CHECK(!no_init.commBegin(0) && no_init.fault()==Fault::NotInitialized);
  Supervisor duplicate;duplicate.init(0,0,hz,true);cycle(duplicate,1);
  CHECK(!duplicate.commBegin(per_ms+3000) && duplicate.feeds()==1);
  Supervisor inhibited;inhibited.init(0,0,hz,true);inhibited.foreground(per_ms);
  inhibited.controlBegin(per_ms,1,hz,true);inhibited.controlEnd(per_ms+100);inhibited.commBegin(per_ms+200);
  CHECK(!inhibited.commEnd(per_ms+300,1,true,false) && inhibited.fault()==Fault::TestInhibit);
  Supervisor refresh_error;refresh_error.init(0,0,hz,true);refresh_error.refreshFailed();
  CHECK(!cycle(refresh_error,1) && refresh_error.fault()==Fault::RefreshFailed);
  Supervisor wrap;wrap.init(0xFFFF0000,0xFFFFFFFE,hz,true);wrap.foreground(0xFFFF1000);
  CHECK(wrap.controlBegin(0x19040,0xFFFFFFFF,hz,true));wrap.controlEnd(0x19140);wrap.commBegin(0x19240);
  CHECK(wrap.commEnd(0x19340,0xFFFFFFFF,true,true));wrap.refreshed(0xFFFFFFFF);
  CHECK(wrap.controlBegin(0x42080,0,hz,true));wrap.controlEnd(0x42180);wrap.commBegin(0x42280);
  CHECK(!wrap.commEnd(0x42380,0,true,true) && wrap.fault()==Fault::None);
  return 0;
}
