#pragma once
namespace hello_world::imu {
struct BMI088HWConfig {};
// 仅替代 SPI 采样；真实 IMU 的算法与超时已在第四项测试，本项模拟其就绪/失效结果。
class Imu {
 public:
  bool ready=true;float heading=0;
  // 记录测试配置，不访问 SPI/GPIO。
  bool init(const BMI088HWConfig &,const float *) {return true;}
  // 返回测试采样结果，允许注入前台停采。
  bool update() {return ready;}
  // 提供校准完成标志供真实 RobotRuntime 判断。
  bool isReady() const {return ready;}
  // 提供本板相对航向，单位 rad。
  float yaw() const {return heading;}
};
}
