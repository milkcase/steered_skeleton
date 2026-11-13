#include "ins_drive.hpp"
#include "ins_robot.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "ins_chassis_iksolver.hpp"
// 通过本板 getter 绑定轮组对象；未确认 ID 的空指针原样保留，令自检失败而非猜测设备。
robot::DriveController *GetDriveController() {
  hello_world::motor::Motor *wheels[4],*steers[4];
  hello_world::pid::Pid *wp[4],*sp[4];
  for(int i=0;i<4;++i) {wheels[i]=GetMotorWheel(i);steers[i]=GetMotorSteer(i);wp[i]=GetPidMotorWheel(i);sp[i]=GetPidMotorSteer(i);}
  static robot::DriveController drive(GetChassis(),wheels,steers,wp,sp,GetChassisIkSolver());
  return &drive;
}
