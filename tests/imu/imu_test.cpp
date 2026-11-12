#include "imu_device.hpp"
#include <cstddef>
#include <limits>

uint32_t test_now=0,sensor_clock=0;
int selected_chip=-1,fail_transaction=0,transactions=0;
uint8_t acc_regs[64]{},gyro_regs[64]{};
SPI_HandleTypeDef spi;
GPIO_TypeDef acc_port,gyro_port;
hello_world::imu::BMI088ErrState hello_world::imu::test_init_error=hello_world::imu::kBMI088ErrStateNoErr;
// 裸测试不申请堆内存；提供删除空桩，使原有虚析构函数的表项能够链接。
void operator delete(void *) noexcept {}
// 为编译器生成的带大小删除表项提供空桩，测试对象均在栈上构造。
void operator delete(void *,std::size_t) noexcept {}
// 为裸 ARM 测试提供内存填充，返回原目标地址。
extern "C" void *memset(void *p,int v,std::size_t n) {
  auto *q=static_cast<uint8_t *>(p);for(std::size_t i=0;i<n;++i)q[i]=v;return p;
}
// 为裸 ARM 测试提供非重叠内存复制，返回目标地址。
extern "C" void *memcpy(void *p,const void *s,std::size_t n) {
  auto *a=static_cast<uint8_t *>(p);auto *b=static_cast<const uint8_t *>(s);
  for(std::size_t i=0;i<n;++i)a[i]=b[i];return p;
}
// 提供可控微秒数，模拟周期、过期及时间回绕。
uint64_t hello_world::tick::GetTickUs() { return test_now; }
// 根据片选记录当前访问的芯片，未被选中时不应有 SPI 数据读取。
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint32_t,GPIO_PinState state) {
  selected_chip=state==GPIO_PIN_SET?-1:(port==&acc_port?0:1);
}
// 模拟 BMI088 两个芯片的寄存器读取，支持在任一次传输注入错误。
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *,uint8_t *tx,uint8_t *rx,uint16_t n,uint32_t) {
  ++transactions;
  if(selected_chip<0 || transactions==fail_transaction)return HAL_ERROR;
  auto *regs=selected_chip==0?acc_regs:gyro_regs;
  const int offset=selected_chip==0?2:1,address=tx[0]&0x7F;
  for(int i=0;i<n;++i)rx[i]=i<offset?0:regs[address+i-offset];
  return HAL_OK;
}
// 把测试有符号原始采样值按低字节在前写入两个寄存器。
void put16(uint8_t *p,int v) { p[0]=v&255;p[1]=(v>>8)&255; }
// 生成静止原始数据与独立的传感器时钟；不使用真实 SPI。
void sample(bool advance=true) {
  memset(acc_regs,0,sizeof(acc_regs));memset(gyro_regs,0,sizeof(gyro_regs));
  acc_regs[0]=0x1E;gyro_regs[0]=0x0F;put16(acc_regs+0x16,10923);
  put16(gyro_regs+2,10);put16(gyro_regs+4,-8);put16(gyro_regs+6,5);
  if(advance)sensor_clock=(sensor_clock+26)&0xFFFFFF;
  acc_regs[0x18]=sensor_clock&255;acc_regs[0x19]=(sensor_clock>>8)&255;acc_regs[0x1A]=(sensor_clock>>16)&255;
}
// 比较测试浮点结果，允许数值积分和原有快速倒平方根带来的小误差。
bool near(float a,float b,float tolerance=0.002f) { return std::fabs(a-b)<tolerance; }
#define CHECK(x) do {if(!(x))return __LINE__;}while(0)

// 测试静止校准、真实 Mahony 解算、姿态变化和故障保护；返回 0 或失败行号。
extern "C" int run_tests() {
  using namespace hello_world::imu;
  ImuEstimator e;
  ImuEstimator::Params cfg;cfg.calibration_samples=20;cfg.alignment_samples=10;
  float acc[3]={0,0,9.80665f},gyro[3]={0.01f,-0.02f,0.005f};
  uint32_t t=1000;
  CHECK(!e.ready(t) && e.roll(t)==0);
  CHECK(e.begin(cfg));
  for(int i=0;i<19;++i,t+=1000)CHECK(e.pushSample(acc,gyro,t));
  CHECK(e.state()==ImuEstimator::State::Calibrating && !e.ready(t));
  e.pushSample(acc,gyro,t);t+=1000;CHECK(e.state()==ImuEstimator::State::Aligning);
  CHECK(near(e.bias(0),0.01f,0.00001f) && near(e.bias(1),-0.02f,0.00001f));
  for(int i=0;i<10;++i,t+=1000)e.pushSample(acc,gyro,t);
  CHECK(e.ready(t) && near(e.roll(t),0) && near(e.pitch(t),0) && near(e.yaw(t),0));
  CHECK(!e.ready(t+5000) && e.yaw(t+5000)==0);
  // 偏置扣除后绕 z 轴转动 1 rad/s，约 1 秒后相对 yaw 应为 1 rad。
  gyro[2]+=1;
  for(int i=0;i<1000;++i,t+=1000)CHECK(e.pushSample(acc,gyro,t));
  CHECK(e.ready(t) && near(e.yaw(t),1.0f,0.015f));
  CHECK(e.recalibrate() && !e.ready(t));
  gyro[2]=0.005f;
  for(int i=0;i<10;++i,t+=1000)e.pushSample(acc,gyro,t);
  const auto count=e.calibrationCount();CHECK(!e.pushSample(acc,gyro,t-1000));CHECK(e.calibrationCount()==count);
  gyro[0]=1;e.pushSample(acc,gyro,t);t+=1000;CHECK(e.calibrationCount()==0);
  gyro[0]=0.01f;
  // 非静止和高方差样本不能完成标定。
  CHECK(e.recalibrate());
  for(int i=0;i<20;++i,t+=1000) {gyro[0]=i%2?0.05f:-0.05f;e.pushSample(acc,gyro,t);}
  CHECK(e.state()==ImuEstimator::State::Calibrating && e.calibrationCount()==0);
  // 初始有倾角时，最终输出仍应以该静止起始姿态为零。
  gyro[0]=0;gyro[1]=0;gyro[2]=0;acc[0]=0;acc[1]=4.903325f;acc[2]=8.492808f;
  CHECK(e.recalibrate());for(int i=0;i<30;++i,t+=1000)e.pushSample(acc,gyro,t);
  CHECK(e.ready(t) && near(e.roll(t),0) && near(e.pitch(t),0));
  CHECK(!e.pushSample(acc,gyro,t+10000) && e.state()==ImuEstimator::State::Fault);
  CHECK(!e.pushSample(acc,gyro,t+11000));
  CHECK(e.recalibrate());gyro[0]=std::numeric_limits<float>::quiet_NaN();
  CHECK(!e.pushSample(acc,gyro,t) && !e.ready(t));gyro[0]=0;
  auto invalid=cfg;invalid.calibration_samples=1;CHECK(!e.begin(invalid));
  invalid=cfg;invalid.mahony_kp=std::numeric_limits<float>::infinity();CHECK(!e.begin(invalid));
  CHECK(e.begin(cfg));acc[0]=acc[1]=acc[2]=0;CHECK(!e.pushSample(acc,gyro,t));
  acc[2]=9.80665f;
  // 微秒计数自然回绕时，采样间隔及有效期仍应正确。
  CHECK(e.begin(cfg));t=0xFFFFD000;
  for(int i=0;i<30;++i,t+=1000)e.pushSample(acc,gyro,t);
  CHECK(e.ready(t));

  // 默认配置确实需要 2000 个静止样本和 500 个对齐样本，不能提前就绪。
  CHECK(e.begin(ImuEstimator::Params{}));
  t=1000;
  for(int i=0;i<2499;++i,t+=1000)CHECK(e.pushSample(acc,gyro,t));
  CHECK(!e.ready(t));CHECK(e.pushSample(acc,gyro,t));CHECK(e.ready(t));
  // 对齐阶段再次晃动必须退回校准；加速度方向方差过大也不能完成校准。
  CHECK(e.begin(cfg));t+=1000;
  for(int i=0;i<20;++i,t+=1000)e.pushSample(acc,gyro,t);
  CHECK(e.state()==ImuEstimator::State::Aligning);
  gyro[0]=1;e.pushSample(acc,gyro,t);t+=1000;gyro[0]=0;
  CHECK(e.state()==ImuEstimator::State::Calibrating && !e.ready(t));
  for(int i=0;i<20;++i,t+=1000) {acc[0]=i%2?0.4f:-0.4f;e.pushSample(acc,gyro,t);}
  CHECK(e.calibrationCount()==0 && !e.ready(t));acc[0]=0;

  Imu imu;
  BMI088HWConfig hw{&spi,&acc_port,16,&gyro_port,1};
  const float rotation[9]={1,0,0,0,1,0,0,0,1};
  CHECK(!imu.isReady() && !imu.update());
  sample();CHECK(imu.init(hw,rotation,cfg));
  for(int i=0;i<30;++i) {test_now+=1000;sample();CHECK(imu.update());}
  CHECK(imu.isReady() && near(imu.yaw(),0));
  test_now+=1000;sample(false);CHECK(!imu.update());
  test_now+=5000;sample(false);CHECK(!imu.update() && !imu.isReady());
  CHECK(imu.state()==ImuEstimator::State::Fault);
  // 每一个实际读取事务发生失败都必须立即被识别，不能被后续成功掩盖。
  for(int step=1;step<=5;++step) {
    sample();CHECK(imu.init(hw,rotation,cfg));
    fail_transaction=transactions+step;test_now+=1000;sample();
    CHECK(!imu.update() && imu.state()==ImuEstimator::State::Fault && !imu.isReady());
    fail_transaction=0;
  }
  sample();CHECK(imu.init(hw,rotation,cfg));test_now+=1000;sample();acc_regs[0]=0;
  CHECK(!imu.update() && !imu.isReady());
  sample();CHECK(imu.init(hw,rotation,cfg));test_now+=1000;sample();acc_regs[0x22]=127;acc_regs[0x23]=224;
  CHECK(!imu.update() && !imu.isReady());
  test_init_error=kTestInitError;sample();CHECK(!imu.init(hw,rotation,cfg));test_init_error=kBMI088ErrStateNoErr;
  float bad[9]={1,0,0,0,1,0,0,0,-1};CHECK(!imu.init(hw,bad,cfg));
  CHECK(!imu.init(hw,nullptr,cfg));hw.hspi=nullptr;CHECK(!imu.init(hw,rotation,cfg));
  return 0;
}
