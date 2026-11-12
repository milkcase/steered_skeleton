#ifndef STEERED_IMU_DEVICE_HPP_
#define STEERED_IMU_DEVICE_HPP_
#include "BMI088.hpp"
#include "imu_estimator.hpp"
#include "tick.hpp"

namespace hello_world::imu {
class Imu {
 public:
  // 仅创建本板对象，不访问尚未初始化的 SPI/GPIO，也不自动触发校准或电机动作。
  Imu() = default;

  // 在非中断初始化阶段绑定硬件和安装旋转矩阵，配置原有 BMI088，再启动分步校准。
  // rotation 将传感器轴转到 x前/y左/z上；调用方必须显式提供，不猜测安装方向。
  bool init(const BMI088HWConfig &hardware, const float rotation[9],
            const ImuEstimator::Params &params=ImuEstimator::Params{}, bool self_test=false) {
    initialized_=false; sensor_timed_=false; hardware_=hardware;
    if (__get_IPSR()!=0 || hardware.hspi==nullptr || hardware.acc_cs_port==nullptr ||
        hardware.gyro_cs_port==nullptr || hardware.acc_cs_pin==0 || hardware.gyro_cs_pin==0 ||
        !validRotation(rotation) || !estimator_.begin(params)) return estimator_.fault();
    for(int i=0;i<9;++i) rotation_[i]=rotation[i];
    params_=params;
    // 固定使用提供驱动的默认量程，后面的字节到物理量转换必须与其保持一致。
    device_.init(hardware,rotation,BMI088Config{});
    device_error_=device_.imuInit(self_test);
    if(device_error_!=kBMI088ErrStateNoErr || !checkIds()) return estimator_.fault();
    initialized_=true;
    return true;
  }

  // 按真实当前微秒时间读取一次硬件并推进校准/姿态；同一调度周期只调用一次。
  // 任一传输失败立即撤销就绪；故障后必须显式调用 init 重新检查硬件。
  bool update() {
    if(!initialized_ || estimator_.state()==ImuEstimator::State::Fault) return false;
    const uint32_t now=nowUs();
    uint8_t acc_raw[9]{},gyro_raw[6]{},temp_raw[2]{};
    if(!checkIds() || !readRegisters(true,0x12,acc_raw,9) ||
        !readRegisters(false,0x02,gyro_raw,6) || !readRegisters(true,0x22,temp_raw,2))
      return estimator_.fault();
    const uint32_t sensor_time=acc_raw[6] | (acc_raw[7]<<8) | (acc_raw[8]<<16);
    if(sensor_timed_ && sensor_time==last_sensor_time_) {
      if(now-last_sensor_host_us_>=params_.stale_us) return estimator_.fault();
      return false;  // 不是新传感器样本，不累计校准次数。
    }
    sensor_timed_=true; last_sensor_time_=sensor_time; last_sensor_host_us_=now;
    float acc_sensor[3],gyro_sensor[3],acc_body[3]{},gyro_body[3]{};
    for(int i=0;i<3;++i) {
      acc_sensor[i]=decodeSigned(acc_raw+2*i)*(3.0f*9.80665f/32768.0f);
      gyro_sensor[i]=decodeSigned(gyro_raw+2*i)*(1000.0f*0.01745329251994329577f/32768.0f);
    }
    for(int row=0;row<3;++row) for(int col=0;col<3;++col) {
      acc_body[row]+=rotation_[3*row+col]*acc_sensor[col];
      gyro_body[row]+=rotation_[3*row+col]*gyro_sensor[col];
    }
    int t=(temp_raw[0]<<3) | (temp_raw[1]>>5); if(t>1023)t-=2048;
    temperature_=23.0f+0.125f*t;
    if(temperature_< -40 || temperature_>85) return estimator_.fault();
    return estimator_.pushSample(acc_body,gyro_body,now);
  }

  // 人工确认重新静止后可清零漂重新采样；硬件故障必须用 init 而非此接口恢复。
  bool recalibrate() {
    if(!initialized_ || estimator_.state()==ImuEstimator::State::Fault) return false;
    sensor_timed_=false;
    return estimator_.recalibrate();
  }
  // 查询校准、姿态对齐和新鲜度是否全部满足；上层进入 Working 必须以此为必要条件。
  bool isReady() const { return initialized_ && estimator_.ready(nowUs()); }
  // 输出相对起始姿态的 roll，单位 rad；不可用时返回零，须结合 isReady 使用。
  float roll() const { return estimator_.roll(nowUs()); }
  // 输出相对起始姿态的 pitch，单位 rad；不可用时返回零，须结合 isReady 使用。
  float pitch() const { return estimator_.pitch(nowUs()); }
  // 输出相对起始姿态的 yaw，单位 rad；不可用时返回零，六轴 IMU 不提供绝对航向。
  float yaw() const { return estimator_.yaw(nowUs()); }
  // 返回最近有效温度，单位摄氏度；未就绪时返回零，不作为姿态许可判断。
  float temperature() const { return isReady()?temperature_:0; }
  // 返回指定轴已估计的角速度零漂，单位 rad/s；用于观察校准，不代表已经就绪。
  float gyroBias(int axis) const { return estimator_.bias(axis); }
  // 返回当前 IMU 阶段，便于调试自检、静止校准、对齐和故障。
  ImuEstimator::State state() const { return estimator_.state(); }
  // 返回原驱动芯片初始化错误码；运行时 SPI 失败通过 Fault 状态表示。
  BMI088ErrState deviceError() const { return device_error_; }
  // 返回连续静止校准采样进度；不是采样总调用次数。
  uint32_t calibrationCount() const { return estimator_.calibrationCount(); }

 private:
  // 获取当前低 32 位微秒数；时间差使用无符号运算以处理自然回绕。
  static uint32_t nowUs() { return static_cast<uint32_t>(tick::GetTickUs()); }
  // 将两个低字节在前的原始字节转成有符号 16 位数，避免依赖无符号强转符号行为。
  static int decodeSigned(const uint8_t *p) {
    const int v=p[0] | (p[1]<<8);return v>=32768?v-65536:v;
  }
  // 检查安装矩阵是否为有限、正交且行列式为 +1 的三维旋转，拒绝反射/缩放矩阵。
  static bool validRotation(const float *r) {
    if(r==nullptr)return false;
    for(int i=0;i<9;++i)if(!std::isfinite(r[i]))return false;
    for(int a=0;a<3;++a)for(int b=0;b<3;++b) {
      float dot=0;for(int c=0;c<3;++c)dot+=r[a*3+c]*r[b*3+c];
      if(std::fabs(dot-(a==b?1.0f:0.0f))>0.001f)return false;
    }
    const float det=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
    return std::fabs(det-1.0f)<0.001f;
  }
  // 带状态检查的 SPI 连续读；加速度计包含一个额外 dummy 字节，任一 HAL 失败即返回 false。
  bool readRegisters(bool accelerometer, uint8_t address, uint8_t *out, uint16_t length) {
    if(length==0 || length>9)return false;
    uint8_t tx[11]{},rx[11]{};tx[0]=address|0x80;
    const uint16_t offset=accelerometer?2:1;
    auto *port=accelerometer?hardware_.acc_cs_port:hardware_.gyro_cs_port;
    const uint32_t pin=accelerometer?hardware_.acc_cs_pin:hardware_.gyro_cs_pin;
    HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET);
    const bool ok=HAL_SPI_TransmitReceive(hardware_.hspi,tx,rx,length+offset,2)==HAL_OK;
    HAL_GPIO_WritePin(port,pin,GPIO_PIN_SET);
    if(!ok)return false;
    for(uint16_t i=0;i<length;++i)out[i]=rx[i+offset];
    return true;
  }
  // 每次采样检查两个芯片 ID，避免断线后把全零或全 1 字节当作传感器数据。
  bool checkIds() {
    uint8_t acc=0,gyro=0;
    return readRegisters(true,0,&acc,1) && readRegisters(false,0,&gyro,1) && acc==0x1E && gyro==0x0F;
  }
  BMI088 device_{};
  BMI088HWConfig hardware_{};
  ImuEstimator estimator_{};
  ImuEstimator::Params params_{};
  BMI088ErrState device_error_=kBMI088ErrStateNoErr;
  bool initialized_=false,sensor_timed_=false;
  uint32_t last_sensor_time_=0,last_sensor_host_us_=0;
  float rotation_[9]{},temperature_=0;
};
}  // namespace hello_world::imu
#endif
