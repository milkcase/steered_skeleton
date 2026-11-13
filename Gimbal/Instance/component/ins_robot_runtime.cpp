#include "robot.hpp"
// 独立编译顶层状态机单例，原模块工厂不必依赖 SPI/HAL；每板仍只有一个实例。
robot::Robot *GetRegisteredRobotInstance() {
  static robot::Robot instance;
  return &instance;
}
