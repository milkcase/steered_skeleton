#pragma once
struct TestMotor {
  float input=0;
  // 返回模拟缓存输入，供真实板级安全检查使用。
  float getInput() const {return input;}
};
extern TestMotor wheels[4],steers[4],yaw,pitch;
// 返回测试驱动轮，检查底盘九台本地输入而非只检查云台轴。
TestMotor *GetMotorWheel(int);
// 返回测试舵电机。
TestMotor *GetMotorSteer(int);
// 返回测试底盘 yaw。
TestMotor *GetMotorYaw();
// 返回测试云台 pitch。
TestMotor *GetMotorPitch();
