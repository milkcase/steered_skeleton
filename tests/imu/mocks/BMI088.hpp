#pragma once
#include <cstdint>
struct SPI_HandleTypeDef {};
struct GPIO_TypeDef {};
enum GPIO_PinState { GPIO_PIN_RESET,GPIO_PIN_SET };
enum HAL_StatusTypeDef { HAL_OK,HAL_ERROR };
// 测试固定处于主线程；真实封装会拒绝在中断中执行芯片初始化。
inline uint32_t __get_IPSR() { return 0; }
void HAL_GPIO_WritePin(GPIO_TypeDef *,uint32_t,GPIO_PinState);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *,uint8_t *,uint8_t *,uint16_t,uint32_t);
namespace hello_world::imu {
enum BMI088ErrState { kBMI088ErrStateNoErr, kTestInitError };
extern BMI088ErrState test_init_error;
struct BMI088Config {};
struct BMI088HWConfig {
  SPI_HandleTypeDef *hspi=nullptr;
  GPIO_TypeDef *acc_cs_port=nullptr;
  uint32_t acc_cs_pin=0;
  GPIO_TypeDef *gyro_cs_port=nullptr;
  uint32_t gyro_cs_pin=0;
};
class BMI088 {
 public:
  // 测试替身不配置芯片；真实构建仍调用原有 BMI088 初始化方法。
  void init(const BMI088HWConfig &,const float *,const BMI088Config &) {}
  // 返回可注入的芯片初始化错误，检查封装是否正确拒绝继续运行。
  BMI088ErrState imuInit(bool) const { return test_init_error; }
};
}
