#include "imu_device.hpp"
#include "robot_control_config.hpp"
#include <cstddef>
#include <limits>

uint32_t test_now=0,sensor_clock=0;
int selected_chip=-1,fail_transaction=0,transactions=0;
// 覆盖 0x00～0x7F，故障诊断会读取 0x7C/0x7D，不能沿用只覆盖数据区的缓冲区。
uint8_t acc_regs[128]{},gyro_regs[128]{};
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
  // 诊断只能读取寄存器；拒绝写命令或越界访问，避免测试桩掩盖非法事务。
  if((tx[0]&0x80)==0 || n<offset || address+n-offset>128)return HAL_ERROR;
  for(int i=0;i<n;++i)rx[i]=i<offset?0:regs[address+i-offset];
  return HAL_OK;
}
// 把测试有符号原始采样值按低字节在前写入两个寄存器。
void put16(uint8_t *p,int v) { p[0]=v&255;p[1]=(v>>8)&255; }
// 生成静止原始数据与独立的传感器时钟；不使用真实 SPI。
void sample(bool advance=true) {
  memset(acc_regs,0,sizeof(acc_regs));memset(gyro_regs,0,sizeof(gyro_regs));
  acc_regs[0]=0x1E;gyro_regs[0]=0x0F;put16(acc_regs+0x16,10923);
  // 默认样本表示已有新加速度数据；专门的启动测试会覆盖为未就绪状态。
  acc_regs[0x03]=0x80;
  put16(gyro_regs+2,10);put16(gyro_regs+4,-8);put16(gyro_regs+6,5);
  if(advance)sensor_clock=(sensor_clock+26)&0xFFFFFF;
  acc_regs[0x18]=sensor_clock&255;acc_regs[0x19]=(sensor_clock>>8)&255;acc_regs[0x1A]=(sensor_clock>>16)&255;
}
// 比较测试浮点结果，允许数值积分和原有快速倒平方根带来的小误差。
bool near(float a,float b,float tolerance=0.002f) { return std::fabs(a-b)<tolerance; }
#define CHECK(x) do {if(!(x))return __LINE__;}while(0)

// 为原有采样/温度回归准备已完成首次 DRDY 的状态，不提前累计校准。
// 仅测试代码推进模拟时钟；用一次无效温度避免准备阶段吞掉第一份校准样本。
bool initForSampling(hello_world::imu::Imu &imu,const hello_world::imu::BMI088HWConfig &hw,
                     const float rotation[9],const hello_world::imu::ImuEstimator::Params &cfg={}) {
  if(!imu.init(hw,rotation,cfg))return false;
  test_now+=5000;
  const uint8_t old_msb=acc_regs[0x22],old_lsb=acc_regs[0x23];acc_regs[0x22]=0x80;
  const bool updated=imu.update();acc_regs[0x22]=old_msb;acc_regs[0x23]=old_lsb;
  return !updated && imu.state()==hello_world::imu::ImuEstimator::State::Calibrating &&
      imu.calibrationCount()==0;
}

// 按现场三种静态姿态检查实际底盘配置，再用真实封装验证静止校准及角速度轴向。
int checkChassisInstallation() {
  using namespace hello_world::imu;
  const float *rotation=robot_control::kChassisRotation;
  const float sensor[3][3]={{-0.061f,-0.251f,-9.714f},{1.891f,-0.125f,-9.619f},{0.175f,-2.990f,-9.481f}};
  const float body[3][3]={{-0.061f,0.251f,9.714f},{1.891f,0.125f,9.619f},{0.175f,2.990f,9.481f}};
  for(int pose=0;pose<3;++pose)for(int row=0;row<3;++row) {
    float transformed=0;for(int col=0;col<3;++col)transformed+=rotation[3*row+col]*sensor[pose][col];
    CHECK(near(transformed,body[pose][row],0.00001f));
  }
  // 本次只改底盘，云台仍保持原单位矩阵，避免把未核对的安装方向一并放行。
  // 上一行为底盘适配时的阶段说明；云台本轮已单独授权修改，此处核对底盘矩阵保持原值。
  const float chassis_expected[9]={1,0,0,0,-1,0,0,0,-1};
  for(int i=0;i<9;++i)CHECK(robot_control::kChassisRotation[i]==chassis_expected[i]);
  Imu imu;BMI088HWConfig hw{&spi,&acc_port,16,&gyro_port,1};
  ImuEstimator::Params cfg;cfg.calibration_samples=20;cfg.alignment_samples=10;
  sample();CHECK(initForSampling(imu,hw,rotation,cfg));
  for(int i=0;i<30;++i) {
    test_now+=1000;sample();put16(acc_regs+0x16,-10923);
    CHECK(imu.update());
  }
  CHECK(imu.isReady() && near(imu.roll(),0) && near(imu.pitch(),0) && near(imu.yaw(),0));
  // 默认模拟角速度原始值为 [10,-8,5]，底盘变换后的零漂必须为 [10,8,-5] 的同单位值。
  const float gyro_scale=1000.0f*0.01745329251994329577f/32768.0f;
  CHECK(near(imu.gyroBias(0),10*gyro_scale,0.000001f));
  CHECK(near(imu.gyroBias(1),8*gyro_scale,0.000001f));
  CHECK(near(imu.gyroBias(2),-5*gyro_scale,0.000001f));
  return 0;
}

// 用云台水平/抬头实测验证实际生产矩阵，再检查真实封装的角速度轴交换和正向 yaw 解算。
int checkGimbalInstallation() {
  using namespace hello_world::imu;
  const float *rotation=robot_control::kGimbalRotation;
  const float sensor[2][3]={{-0.20290859f,0.03501520f,-9.75667095f},{-0.29682847f,3.7241807f,-9.06803894f}};
  const float body[2][3]={{0.03501520f,-0.20290859f,9.75667095f},{3.7241807f,-0.29682847f,9.06803894f}};
  for(int pose=0;pose<2;++pose)for(int row=0;row<3;++row) {
    float transformed=0;for(int col=0;col<3;++col)transformed+=rotation[3*row+col]*sensor[pose][col];
    CHECK(near(transformed,body[pose][row],0.00001f));
  }
  Imu imu;BMI088HWConfig hw{&spi,&acc_port,16,&gyro_port,1};
  ImuEstimator::Params cfg;cfg.calibration_samples=20;cfg.alignment_samples=10;
  sample();CHECK(initForSampling(imu,hw,rotation,cfg));
  for(int i=0;i<30;++i) {
    test_now+=1000;sample();put16(acc_regs+0x16,-10923);CHECK(imu.update());
  }
  CHECK(imu.isReady() && near(imu.roll(),0) && near(imu.pitch(),0) && near(imu.yaw(),0));
  // 模拟传感器角速度原始值 [10,-8,5] 经云台矩阵后必须为 [-8,10,-5] 的同单位值。
  const float gyro_scale=1000.0f*0.01745329251994329577f/32768.0f;
  CHECK(near(imu.gyroBias(0),-8*gyro_scale,0.000001f));
  CHECK(near(imu.gyroBias(1),10*gyro_scale,0.000001f));
  CHECK(near(imu.gyroBias(2),-5*gyro_scale,0.000001f));
  // 传感器Z负向转动约1rad/s，变换后云台Z应正向；真实 Mahony 积分一秒后 yaw 约为+1rad。
  for(int i=0;i<1000;++i) {
    test_now+=1000;sample();put16(acc_regs+0x16,-10923);put16(gyro_regs+6,5-1877);CHECK(imu.update());
  }
  CHECK(imu.isReady() && near(imu.yaw(),1.0f,0.015f));
  return 0;
}

// 直接测试生产初始化入口，验证静默、首次就绪、超时与故障，不能依赖准备辅助函数。
int checkAccelStartupGate() {
  using namespace hello_world::imu;
  Imu imu;BMI088HWConfig hw{&spi,&acc_port,16,&gyro_port,1};
  const float rotation[9]={1,0,0,0,1,0,0,0,1};
  ImuEstimator::Params cfg;cfg.calibration_samples=20;cfg.alignment_samples=10;
  test_now=0;sample();CHECK(imu.init(hw,rotation,cfg));int before=transactions;
  // 5ms 静默期间任何调用均不读 SPI，不自旋、不计样本、不就绪。
  for(test_now=0;test_now<5000;test_now+=1000) {
    CHECK(!imu.update() && transactions==before && !imu.isReady());
    CHECK(imu.state()==ImuEstimator::State::Calibrating && imu.calibrationCount()==0);
  }
  test_now=4999;CHECK(!imu.update() && transactions==before);
  // 即使传感器时钟在变、温度有效，DRDY=0 的全零数据也不能送入估计器。
  for(test_now=5000;test_now<19000;test_now+=1000) {
    sample();put16(acc_regs+0x16,0);acc_regs[0x03]=0x10;before=transactions;
    CHECK(!imu.update() && transactions==before+1);
    CHECK(imu.state()==ImuEstimator::State::Calibrating && imu.calibrationCount()==0 && !imu.isReady());
  }
  // 第一次 DRDY 到来才读数据并计第一份校准样本；之后仍须完成全部校准和对齐。
  test_now=19000;sample();before=transactions;
  CHECK(imu.update() && transactions==before+6 && imu.calibrationCount()==1 && !imu.isReady());
  for(int i=1;i<30;++i) {test_now+=1000;sample();before=transactions;CHECK(imu.update() && transactions==before+5);}
  CHECK(imu.isReady());
  // 进入正常采样后再次全零，不能借启动宽限跳过原量程检查。
  test_now+=1000;sample();put16(acc_regs+0x16,0);
  CHECK(!imu.update() && imu.state()==ImuEstimator::State::Fault && !imu.isReady());
  // 初次就绪即给出坏数据，同样必须故障，不能以状态位代替数据质量。
  test_now=0;sample();CHECK(imu.init(hw,rotation,cfg));test_now=5000;sample();put16(acc_regs+0x16,0);
  CHECK(!imu.update() && imu.state()==ImuEstimator::State::Fault);
  // 到 20ms 截止点，即使 DRDY 才变为1也不能接受；故障后不重复读寄存器。
  test_now=0;sample();CHECK(imu.init(hw,rotation,cfg));test_now=19999;sample();acc_regs[0x03]=0x10;
  CHECK(!imu.update() && imu.state()==ImuEstimator::State::Calibrating);
  test_now=20000;sample();before=transactions;CHECK(!imu.update() && transactions==before);
  CHECK(imu.state()==ImuEstimator::State::Fault && !imu.recalibrate());
  test_now+=1000;CHECK(!imu.update() && transactions==before);
  // 初次门控增加的状态读取及其后的原5次读取，任一次SPI失败都必须故障。
  for(int step=1;step<=6;++step) {
    sample();CHECK(imu.init(hw,rotation,cfg));test_now+=5000;sample();fail_transaction=transactions+step;
    CHECK(!imu.update() && imu.state()==ImuEstimator::State::Fault && !imu.isReady());fail_transaction=0;
  }
  sample();CHECK(imu.init(hw,rotation,cfg));test_now+=5000;sample();acc_regs[0]=0;
  CHECK(!imu.update() && imu.state()==ImuEstimator::State::Fault);
  // 计数自然回绕不改变首次就绪等待的时限。
  test_now=0xFFFFF000u;const auto start=test_now;sample();CHECK(imu.init(hw,rotation,cfg));
  test_now=start+4999u;before=transactions;CHECK(!imu.update() && transactions==before);
  test_now=start+19999u;sample();acc_regs[0x03]=0x10;CHECK(!imu.update());
  test_now=start+20000u;CHECK(!imu.update() && imu.state()==ImuEstimator::State::Fault);
  return 0;
}

// 检查故障原始字节与寄存器快照、失败掩码和只读一次，不改变健康采样或故障锁存。
int checkAccelFaultDiagnostics() {
  using namespace hello_world::imu;
  Imu imu;BMI088HWConfig hw{&spi,&acc_port,16,&gyro_port,1};
  const float rotation[9]={1,0,0,0,1,0,0,0,1};
  sample();CHECK(initForSampling(imu,hw,rotation));
  CHECK(!imu.accelFaultDiagnostics().captured && imu.accelFaultDiagnostics().read_mask==0);
  test_now+=1000;sample();int before=transactions;
  CHECK(imu.update() && transactions==before+5 && !imu.accelFaultDiagnostics().captured);
  // 模拟现场三轴原始加速度为零；原数据与配置在故障后应分别完整保存。
  test_now+=1000;sample();put16(acc_regs+0x16,0);
  acc_regs[0x02]=0x04;acc_regs[0x03]=0x80;acc_regs[0x40]=0x8C;acc_regs[0x41]=0;
  acc_regs[0x7C]=0;acc_regs[0x7D]=4;
  uint8_t expected[9];for(int i=0;i<9;++i)expected[i]=acc_regs[0x12+i];
  before=transactions;CHECK(!imu.update() && imu.state()==ImuEstimator::State::Fault && !imu.isReady());
  CHECK(transactions==before+8 && selected_chip==-1);
  auto d=imu.accelFaultDiagnostics();CHECK(d.captured && d.read_mask==7);
  for(int i=0;i<9;++i)CHECK(d.raw[i]==expected[i]);
  CHECK(d.status[0]==4 && d.status[1]==0x80 && d.config[0]==0x8C && d.config[1]==0);
  CHECK(d.power[0]==0 && d.power[1]==4);
  // 修改副本、改变外设及再次调用不能覆盖首份故障记录，也不会重复读取或恢复就绪。
  d.power[1]=0;sample();test_now+=1000;before=transactions;
  CHECK(!imu.update() && transactions==before && !imu.isReady());
  CHECK(imu.accelFaultDiagnostics().power[1]==4);
  // 三组诊断读取任意一组失败，都继续完成其余尝试，并用掩码区分失败和合法零值。
  for(int failed=0;failed<3;++failed) {
    sample();CHECK(initForSampling(imu,hw,rotation));
    CHECK(!imu.accelFaultDiagnostics().captured && imu.accelFaultDiagnostics().read_mask==0);
    test_now+=1000;sample();put16(acc_regs+0x16,0);
    acc_regs[0x02]=4;acc_regs[0x03]=0x80;acc_regs[0x40]=0x8C;acc_regs[0x7D]=4;
    before=transactions;fail_transaction=transactions+6+failed;
    CHECK(!imu.update() && imu.state()==ImuEstimator::State::Fault && !imu.isReady());
    fail_transaction=0;d=imu.accelFaultDiagnostics();
    CHECK(d.captured && d.read_mask==(7^(1<<failed)) && transactions==before+8);
    if(failed!=0)CHECK(d.status[0]==4 && d.status[1]==0x80);
    if(failed!=1)CHECK(d.config[0]==0x8C);
    if(failed!=2)CHECK(d.power[1]==4);
    before=transactions;CHECK(!imu.update() && transactions==before);
  }
  // SPI 在原采样阶段失败时，没有完整惯性样本，不伪造故障原始字节快照。
  sample();CHECK(initForSampling(imu,hw,rotation));test_now+=1000;sample();
  fail_transaction=transactions+3;CHECK(!imu.update());fail_transaction=0;
  CHECK(imu.state()==ImuEstimator::State::Fault && !imu.accelFaultDiagnostics().captured);
  return 0;
}

// 验证诊断保留真实输入、首份故障样本锁存及只读副本，不改变原就绪和故障阈值。
int checkSampleDiagnostics() {
  using namespace hello_world::imu;
  ImuEstimator e;ImuEstimator::Params cfg;
  float acc[3]={0,0,9.80665f},gyro[3]={0.01f,-0.02f,0.005f};
  CHECK(e.begin(cfg));auto d=e.sampleDiagnostics();
  CHECK(std::isnan(d.acc_norm) && std::isnan(d.gyro_norm));
  CHECK(e.pushSample(acc,gyro,1000) && e.calibrationCount()==1 && !e.ready(1000));
  d=e.sampleDiagnostics();
  for(int i=0;i<3;++i)CHECK(d.acc[i]==acc[i] && d.gyro[i]==gyro[i]);
  CHECK(near(d.acc_norm,9.80665f) && near(d.gyro_norm,std::sqrt(0.000525f)));
  // 调用方改写副本不能影响内部记录或校准结果。
  d.acc[2]=100;d.acc_norm=100;CHECK(near(e.sampleDiagnostics().acc[2],9.80665f));
  CHECK(e.calibrationCount()==1 && e.state()==ImuEstimator::State::Calibrating);
  // 重现当前现场症状：零加速度在第一份样本即故障，诊断仍能读到原始零值和角速度。
  CHECK(e.begin(cfg));acc[2]=0;
  CHECK(!e.pushSample(acc,gyro,2000) && e.state()==ImuEstimator::State::Fault && e.calibrationCount()==0);
  d=e.sampleDiagnostics();CHECK(d.acc_norm==0 && d.acc[2]==0 && d.gyro[1]==gyro[1]);
  acc[2]=9.80665f;CHECK(!e.pushSample(acc,gyro,3000));
  CHECK(e.sampleDiagnostics().acc_norm==0 && !e.ready(3000));
  // 有限但超限的加速度和角速度仍分别被原阈值拒绝，且可看见具体超限值。
  CHECK(e.begin(cfg));acc[2]=36;CHECK(!e.pushSample(acc,gyro,4000));
  CHECK(e.sampleDiagnostics().acc_norm==36 && e.state()==ImuEstimator::State::Fault);
  CHECK(e.begin(cfg));acc[2]=9.80665f;gyro[0]=18;
  CHECK(!e.pushSample(acc,gyro,5000) && e.sampleDiagnostics().gyro[0]==18);
  CHECK(e.sampleDiagnostics().gyro_norm>17 && e.state()==ImuEstimator::State::Fault);
  // 非有限值及空指针不会被诊断记录修复或解引用；未计算的模长明确为 NaN。
  CHECK(e.begin(cfg));gyro[0]=std::numeric_limits<float>::quiet_NaN();
  CHECK(!e.pushSample(acc,gyro,6000) && std::isnan(e.sampleDiagnostics().gyro[0]));
  CHECK(std::isnan(e.sampleDiagnostics().acc_norm) && std::isnan(e.sampleDiagnostics().gyro_norm));
  CHECK(e.begin(cfg));gyro[0]=std::numeric_limits<float>::infinity();
  CHECK(!e.pushSample(acc,gyro,7000) && std::isinf(e.sampleDiagnostics().gyro[0]));
  CHECK(e.begin(cfg));gyro[0]=0;
  CHECK(!e.pushSample(nullptr,gyro,8000) && std::isnan(e.sampleDiagnostics().acc[0]));
  CHECK(e.begin(cfg));CHECK(!e.pushSample(acc,nullptr,9000));
  CHECK(std::isnan(e.sampleDiagnostics().gyro[0]));
  // 35m/s² 和 17rad/s 的原边界仍可通过量程检查，但不满足静止校准条件。
  CHECK(e.begin(cfg));acc[2]=35;gyro[0]=17;gyro[1]=gyro[2]=0;
  CHECK(e.pushSample(acc,gyro,10000) && e.state()==ImuEstimator::State::Calibrating && e.calibrationCount()==0);
  CHECK(e.sampleDiagnostics().acc_norm==35 && e.sampleDiagnostics().gyro_norm==17);
  // 显式重新开始清除故障记录，随后仍须按原样本数校准，不直接进入 Ready。
  CHECK(e.recalibrate());d=e.sampleDiagnostics();
  CHECK(d.acc[2]==0 && std::isnan(d.acc_norm) && !e.ready(11000));
  return 0;
}

// 测试静止校准、真实 Mahony 解算、姿态变化和故障保护；返回 0 或失败行号。
extern "C" int run_tests() {
  const int gimbal_installation_result=checkGimbalInstallation();if(gimbal_installation_result!=0)return gimbal_installation_result;
  const int installation_result=checkChassisInstallation();if(installation_result!=0)return installation_result;
  const int startup_result=checkAccelStartupGate();if(startup_result!=0)return startup_result;
  const int accel_diagnostics_result=checkAccelFaultDiagnostics();if(accel_diagnostics_result!=0)return accel_diagnostics_result;
  const int diagnostics_result=checkSampleDiagnostics();if(diagnostics_result!=0)return diagnostics_result;
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
  sample();CHECK(initForSampling(imu,hw,rotation,cfg));
  for(int i=0;i<30;++i) {test_now+=1000;sample();CHECK(imu.update());}
  CHECK(imu.isReady() && near(imu.yaw(),0));
  test_now+=1000;sample(false);CHECK(!imu.update());
  test_now+=5000;sample(false);CHECK(!imu.update() && !imu.isReady());
  CHECK(imu.state()==ImuEstimator::State::Fault);
  // 每一个实际读取事务发生失败都必须立即被识别，不能被后续成功掩盖。
  for(int step=1;step<=5;++step) {
    sample();CHECK(initForSampling(imu,hw,rotation,cfg));
    fail_transaction=transactions+step;test_now+=1000;sample();
    CHECK(!imu.update() && imu.state()==ImuEstimator::State::Fault && !imu.isReady());
    fail_transaction=0;
  }
  sample();CHECK(initForSampling(imu,hw,rotation,cfg));test_now+=1000;sample();acc_regs[0]=0;
  CHECK(!imu.update() && !imu.isReady());
  sample();CHECK(initForSampling(imu,hw,rotation,cfg));test_now+=1000;sample();acc_regs[0x22]=127;acc_regs[0x23]=224;
  CHECK(!imu.update() && !imu.isReady());
  // 冷启动连续 1.28 秒返回无效温度码时，不阻塞、不累计校准，也不提前就绪。
  Imu cold;test_now=0;sample();CHECK(initForSampling(cold,hw,rotation,cfg));
  for(int i=0;i<1280;++i) {
    test_now+=1000;sample();acc_regs[0x22]=0x80;
    const auto before_time=test_now;const auto before_transactions=transactions;
    CHECK(!cold.update() && !cold.isReady());
    CHECK(cold.state()==ImuEstimator::State::Calibrating && cold.calibrationCount()==0);
    CHECK(test_now==before_time && transactions==before_transactions+5);
  }
  // 第一份有效温度到来后才从样本 1 开始，仍须完整完成校准与对齐。
  test_now+=1000;sample();CHECK(cold.update() && cold.calibrationCount()==1 && !cold.isReady());
  for(int i=1;i<30;++i) {test_now+=1000;sample();CHECK(cold.update());}
  CHECK(cold.isReady() && near(cold.temperature(),23));
  // 已经正常后再次无效，即使仍在初始化后的 1500ms 内，也必须立即故障。
  test_now+=1000;sample();acc_regs[0x22]=0x80;
  CHECK(!cold.update() && cold.state()==ImuEstimator::State::Fault && !cold.isReady());
  // 故障锁存，温度恢复也不能自行恢复；必须显式重新初始化。
  test_now+=1000;sample();CHECK(!cold.update() && !cold.recalibrate());
  // 有效温度后的人工重校准不重启温度宽限，避免反复宽限掩盖读数故障。
  sample();CHECK(initForSampling(cold,hw,rotation,cfg));test_now+=1000;sample();CHECK(cold.update());
  CHECK(cold.recalibrate());test_now+=1000;sample();acc_regs[0x22]=0x80;
  CHECK(!cold.update() && cold.state()==ImuEstimator::State::Fault);
  // 无效温度最多等 1500ms：截止前仍等待，到截止点必须故障，不能无限重试。
  test_now=0;sample();CHECK(initForSampling(cold,hw,rotation,cfg));
  test_now=1499999;sample();acc_regs[0x22]=0x80;
  CHECK(!cold.update() && cold.state()==ImuEstimator::State::Calibrating);
  test_now=1500000;sample();acc_regs[0x22]=0x80;
  CHECK(!cold.update() && cold.state()==ImuEstimator::State::Fault && !cold.isReady());
  // 截止前的有效温度可开始校准，截止点才第一次出现的有效温度不得复活过期等待。
  test_now=0;sample();CHECK(initForSampling(cold,hw,rotation,cfg));
  test_now=1499999;sample();CHECK(cold.update() && cold.calibrationCount()==1);
  test_now=0;sample();CHECK(initForSampling(cold,hw,rotation,cfg));
  test_now=1500000;sample();CHECK(!cold.update() && cold.state()==ImuEstimator::State::Fault);
  // 微秒计数回绕不能延长或缩短启动期限。
  test_now=0xFFFFF000u;const auto start=test_now;sample();CHECK(initForSampling(cold,hw,rotation,cfg));
  test_now=start+1499999u;sample();acc_regs[0x22]=0x80;
  CHECK(!cold.update() && cold.state()==ImuEstimator::State::Calibrating);
  test_now=start+1500000u;sample();CHECK(!cold.update() && cold.state()==ImuEstimator::State::Fault);
  // 启动等待期间，坏芯片 ID、SPI 失败和其他越界温度仍须立即故障。
  for(int step=1;step<=5;++step) {
    sample();CHECK(initForSampling(cold,hw,rotation,cfg));test_now+=1000;sample();acc_regs[0x22]=0x80;
    fail_transaction=transactions+step;
    CHECK(!cold.update() && cold.state()==ImuEstimator::State::Fault);fail_transaction=0;
  }
  sample();CHECK(initForSampling(cold,hw,rotation,cfg));test_now+=1000;sample();acc_regs[0x22]=0x80;gyro_regs[0]=0;
  CHECK(!cold.update() && cold.state()==ImuEstimator::State::Fault);
  sample();CHECK(initForSampling(cold,hw,rotation,cfg));test_now+=1000;sample();acc_regs[0x22]=0x7F;acc_regs[0x23]=0xE0;
  CHECK(!cold.update() && cold.state()==ImuEstimator::State::Fault);
  // 等待温度时仍监督传感器时钟；停止更新达到原 5ms 期限就必须故障。
  sample();CHECK(initForSampling(cold,hw,rotation,cfg));test_now+=1000;sample();acc_regs[0x22]=0x80;
  CHECK(!cold.update() && cold.state()==ImuEstimator::State::Calibrating);
  test_now+=5000;sample(false);acc_regs[0x22]=0x80;
  CHECK(!cold.update() && cold.state()==ImuEstimator::State::Fault);
  test_init_error=kTestInitError;sample();CHECK(!initForSampling(imu,hw,rotation,cfg));test_init_error=kBMI088ErrStateNoErr;
  float bad[9]={1,0,0,0,1,0,0,0,-1};CHECK(!imu.init(hw,bad,cfg));
  CHECK(!imu.init(hw,nullptr,cfg));hw.hspi=nullptr;CHECK(!initForSampling(imu,hw,rotation,cfg));
  return 0;
}
