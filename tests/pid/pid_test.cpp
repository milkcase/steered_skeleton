#include "pid.hpp"
#include "pid_config.hpp"
#include "ins_pid.hpp"
#include <cstddef>
#include <limits>

// 模拟器测试只运行纯计算代码，不链接电机驱动或 HAL。
// 为 ARM 无操作系统测试提供内存填充实现，将 n 字节写成指定值，返回原地址。
extern "C" void *memset(void *p, int v, std::size_t n) {
  auto *q=static_cast<unsigned char *>(p); for(std::size_t i=0;i<n;++i) q[i]=v; return p;
}
// 为 ARM 无操作系统测试提供内存复制实现；测试缓冲区互不重叠，返回目标地址。
extern "C" void *memcpy(void *p, const void *s, std::size_t n) {
  auto *q=static_cast<unsigned char *>(p); const auto *r=static_cast<const unsigned char *>(s);
  for(std::size_t i=0;i<n;++i) q[i]=r[i]; return p;
}
#define CHECK(x) do { if (!(x)) return __LINE__; } while(0)
// 按给定绝对误差容差比较两个浮点数，避免精确相等判断受舍入误差影响。
bool near(float a,float b,float tolerance=0.0001f) {
  return a-b<tolerance && b-a<tolerance;
}
// 执行本文件全部断言；成功返回 0，失败返回断言所在行号，供 Python 模拟器报告。
extern "C" int run_tests() {
  using hello_world::pid::Pid;
  using Params=Pid::Params;
  Pid p;
  CHECK(p.update(100,0)==0 && p.lastUpdateOk());
  CHECK(p.configure({.kp=2,.output_limit=3}));
  CHECK(near(p.update(1,0),2));
  CHECK(near(p.update(-1,0),-2));
  CHECK(p.update(10,0)==3 && p.update(-10,0)==-3);

  // 积分用秒计算，测试不同 dt 下的累积以及正负积分限幅。
  CHECK(p.configure({.ki=2,.integral_limit=0.01f,.output_limit=10}));
  CHECK(near(p.updateError(1,0.001f),0.002f));
  CHECK(near(p.updateError(1,0.002f),0.006f));
  for(int i=0;i<20;++i) p.updateError(1);
  CHECK(near(p.integralTerm(),0.01f));
  for(int i=0;i<30;++i) p.updateError(-1);
  CHECK(near(p.integralTerm(),-0.01f));

  CHECK(p.configure({.kd=0.5f,.output_limit=1000}));
  CHECK(p.updateError(10)==0);  // 首帧没有可用的历史误差。
  CHECK(near(p.updateError(10.1f),50,0.01f));
  p.reset(); CHECK(p.output()==0 && p.integralTerm()==0 && !p.lastUpdateOk());
  CHECK(p.updateError(-100)==0);

  // P 项已经使输出饱和，积分不能继续加深饱和；反向积分则允许退出饱和。
  CHECK(p.configure({.kp=2,.ki=100,.integral_limit=10,.output_limit=1}));
  for(int i=0;i<100;++i) CHECK(p.updateError(1)==1);
  CHECK(p.integralTerm()==0);
  CHECK(p.updateError(-1)==-1 && p.integralTerm()==0);
  CHECK(p.configure({.ki=100,.integral_limit=10,.output_limit=1}));
  for(int i=0;i<9;++i) p.updateError(1);
  const float stored=p.integralTerm();
  p.updateError(10); CHECK(near(p.integralTerm(),stored));
  p.updateError(-1); CHECK(p.integralTerm()<stored);

  // 任一非有限参数、负限幅或不合法 dt 都必须得到零输出并清除历史。
  const float nan=std::numeric_limits<float>::quiet_NaN();
  const float inf=std::numeric_limits<float>::infinity();
  for(int i=0;i<7;++i) {
    Params cfg{.kp=1,.ki=1,.kd=1,.integral_limit=1,.output_limit=1};
    float *fields[]={&cfg.kp,&cfg.ki,&cfg.kd,&cfg.integral_limit,&cfg.output_limit,&cfg.max_dt_s,&cfg.error_period};
    *fields[i]=nan; CHECK(!p.configure(cfg)); CHECK(p.updateError(1)==0 && !p.lastUpdateOk());
    *fields[i]=inf; CHECK(!p.configure(cfg));
  }
  CHECK(!p.configure({.integral_limit=-1}));
  CHECK(!p.configure({.output_limit=-1}));
  CHECK(!p.configure({.max_dt_s=0}));
  CHECK(!p.configure({.error_period=-1}));
  CHECK(p.configure({.kp=1,.ki=1,.integral_limit=1,.output_limit=10}));
  const float bad_dt[]={0,-0.001f,0.011f,nan,inf};
  for(float dt:bad_dt) {
    p.updateError(1); CHECK(p.updateError(1,dt)==0 && !p.lastUpdateOk());
    CHECK(p.integralTerm()==0);
  }
  CHECK(p.update(nan,0)==0 && !p.lastUpdateOk());
  CHECK(p.update(0,inf)==0 && !p.lastUpdateOk());
  CHECK(p.updateError(inf)==0 && !p.lastUpdateOk());
  CHECK(p.configure({.kp=std::numeric_limits<float>::max(),.output_limit=1}));
  CHECK(p.updateError(2)==0 && !p.lastUpdateOk());
  CHECK(p.configure({.kp=1,.output_limit=1}));
  CHECK(p.update(std::numeric_limits<float>::max(),-std::numeric_limits<float>::max())==0);

  // 359°→1° 使用短角差；误差跨 ±π 时，微分也按周期连续处理。
  constexpr float pi=3.14159265358979323846f;
  CHECK(p.configure({.kp=1,.output_limit=10,.error_period=2*pi}));
  CHECK(near(p.update(pi/180,359*pi/180),2*pi/180));
  CHECK(near(p.update(359*pi/180,pi/180),-2*pi/180));
  CHECK(p.configure({.kd=1,.output_limit=10000,.error_period=2*pi}));
  CHECK(p.updateError(pi-0.01f)==0);
  CHECK(near(p.updateError(-pi+0.01f),20,0.001f));

  // 零输出上限保持禁用；重新配置必须清掉积分和微分历史。
  CHECK(p.configure({.kp=1,.ki=1,.kd=1,.integral_limit=10,.output_limit=0}));
  CHECK(p.updateError(1)==0 && p.updateError(-2)==0 && p.integralTerm()==0);
  CHECK(p.configure({.ki=1,.integral_limit=1,.output_limit=1}));
  p.updateError(1); CHECK(p.integralTerm()>0);
  CHECK(p.configure({.kd=1,.output_limit=1}));
  CHECK(p.integralTerm()==0 && p.updateError(10)==0);

#if defined(TEST_CHASSIS)
  CHECK(GetPidMotorWheel(-1)==nullptr && GetPidMotorWheel(4)==nullptr);
  CHECK(GetPidMotorSteer(-1)==nullptr && GetPidMotorSteer(4)==nullptr);
  for(int i=0;i<4;++i) {
    CHECK(GetPidMotorWheel(i)!=nullptr && GetPidMotorSteer(i)!=nullptr);
    CHECK(GetPidMotorWheel(i)==GetPidMotorWheel(i));
    CHECK(GetPidMotorWheel(i)->updateError(100)==0);
    CHECK(GetPidMotorSteer(i)->updateError(1)==0);
    for(int j=i+1;j<4;++j) CHECK(GetPidMotorWheel(i)!=GetPidMotorWheel(j));
  }
  CHECK(GetPidMotorYaw()!=nullptr && GetPidMotorYaw()->updateError(1)==0);
  auto *wheel0=GetPidMotorWheel(0); auto *wheel1=GetPidMotorWheel(1);
  CHECK(wheel0->configure({.ki=1,.integral_limit=1,.output_limit=1}));
  wheel0->updateError(1); CHECK(wheel0->integralTerm()>0 && wheel1->integralTerm()==0);
#else
  CHECK(GetPidMotorYaw()==nullptr);
  CHECK(GetPidMotorPitch()!=nullptr && GetPidMotorPitch()==GetPidMotorPitch());
  CHECK(GetPidMotorPitch()->updateError(1)==0);
#endif

  // 纯数学仿真：用一阶对象检查闭环误差会下降，不将此结果当作实车整定。
  CHECK(p.configure({.kp=2,.ki=5,.integral_limit=1,.output_limit=2}));
  float plant=0;
  for(int i=0;i<5000;++i) {
    const float u=p.update(1,plant);
    CHECK(p.lastUpdateOk() && u<=2 && u>=-2);
    plant+=0.001f*(-plant+u);
  }
  CHECK(near(plant,1,0.01f));
  return 0;
}
