#include "ins_robot.hpp"
#include "gimbal.hpp"
#include "gimbal_control_config.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"

// 返回底盘板的 yaw 位置环，绑定本板 J4310 和 yaw PID；不创建 pitch 的重复控制对象。
robot::Gimbal *GetGimbal() {
  static robot::Gimbal gimbal(robot::Gimbal::Axis::Yaw,
      static_cast<hello_world::motor::DaMiao *>(GetMotorYaw()),GetPidMotorYaw(),gimbal_control_config::kYaw);
  return &gimbal;
}
