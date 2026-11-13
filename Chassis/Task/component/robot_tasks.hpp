#ifndef STEERED_ROBOT_TASKS_HPP_
#define STEERED_ROBOT_TASKS_HPP_
// 在启动控制中断前注册本板对象，前台 IMU 初始化另行进行。
void InitializeRobotRuntime();
// 将一次 TIM6 调度转给 Robot，并用 CPU 周期计数检查真实 dt。
void RunRobotControl();
#endif
