#pragma once
#include "chassis_iksolver.hpp"
#include "steer_debug_safety.hpp"
#include "motor_debug_preview.hpp"
#include "vehicle_motor_config.hpp"
namespace robot {
// 只计算四舵及四轮目标；唯一发送层只允许左前驱动，不能据此使能其余M3508。
class VectorDriveDebug {
 public:
  // 注册本地四舵并创建独立双环，沿用已整定的转向数值，不发送硬件命令。
  void bind(hello_world::motor::Motor *const motors[4]) {
    for(int i=0;i<4;++i) {motors_[i]=motors[i];position_[i].configure(robot_debug::steer_safety::positionParams());speed_[i].configure(robot_debug::direction::speedParams());}
    hello_world::chassis_ik_solver::ChassisIkSolver::Limits limits;
    limits.max_wheel_speed_rad_s=robot_debug::drive::kMaxTargetSpeed;
    limits.max_steer_rate_rad_s=robot_debug::steer_safety::kTargetRateRadS;
    limits.max_dt_s=0.003f;limits.project_drive_speed=true;solver_.configure(limits);
  }
  // 只检查本地真实反馈和各舵物理限幅，不读取Ozone镜像。
  bool safe(uint32_t now) const {
    remote_input::InterruptLock lock;now=hello_world::tick::GetTickMs();
    for(int i=0;i<4;++i) {
      auto *m=motors_[i];
      if(m==nullptr || m->motor_info().id!=i+1 || m->get_input_type()!=hello_world::motor::InputType::kCurr ||
          m->motor_info().curr_input_lim<robot_debug::direction::kCurrentA ||
          !robot_control::MotorFeedbacks().fresh(i+4,now) || !std::isfinite(m->angle()) || !std::isfinite(m->vel()) ||
          std::fabs(m->vel())>robot_debug::steer_safety::kActualSpeedLimit)return false;
    }
    return solver_.configured();
  }
  // 幅度变为车速、方向变为云台系向量，再复用整车坐标变换/几何逆解和180度等效解。
  bool update(float direction,float magnitude,float yaw,float dt,float &left_speed) {
    left_speed=0;
    if(!safe(hello_world::tick::GetTickMs()) || !std::isfinite(magnitude) || magnitude<0 || magnitude>1)return false;
    float measured[4];for(int i=0;i<4;++i)measured[i]=motors_[i]->angle();
    const float v=magnitude*robot_debug::drive::kMaxTargetSpeed*vehicle_config::kWheelRadiusM;
    const ChassisMotion command{v*std::cos(direction),v*std::sin(direction),0};
    if(!solver_.solve(command,yaw,measured,dt))return false;
    const auto &r=solver_.result();
    for(int i=0;i<4;++i) {
      const float target_speed=position_[i].update(r.wheels[i].steer_rad,measured[i],dt);
      commands_[i]=speed_[i].update(target_speed,motors_[i]->vel(),dt);
      if(!position_[i].lastUpdateOk() || !speed_[i].lastUpdateOk() || !std::isfinite(commands_[i]))return false;
      robot_debug::LoopDiagnostics d{1,0,r.wheels[i].steer_rad,measured[i],
          hello_world::chassis_ik_solver::ChassisIkSolver::wrapAngle(r.wheels[i].steer_rad-measured[i]),
          target_speed,motors_[i]->vel(),target_speed-motors_[i]->vel(),commands_[i]};
      robot_debug::publishLoops(robot_four_steer_diagnostics[i],d);
      robot_vector_wheel_targets[i]=r.wheels[i].speed_rad_s;
    }
    left_speed=r.wheels[0].speed_rad_s;return true;
  }
  // 发送前复查后合并四舵电流，任一槽编码失败就清整帧；不发送报文。
  bool frame(uint8_t data[8]) {
    for(int i=0;i<8;++i)data[i]=0;
    for(int i=0;i<4;++i) {
      if(!std::isfinite(commands_[i]) || std::fabs(commands_[i])>robot_debug::direction::kCurrentA ||
          motors_[i]->setInput(commands_[i])!=hello_world::motor::Status::kOk || !motors_[i]->encode(data)) {
        for(int k=0;k<8;++k)data[k]=0;
        return false;
      }
      const int bits=(data[2*i]<<8)|data[2*i+1],raw=bits>=32768?bits-65536:bits;
      if(std::abs(raw)>robot_debug::direction::kRawCurrentLimit) {for(int k=0;k<8;++k)data[k]=0;return false;}
    }
    return true;
  }
  // 失效/正常回中清所有私有PID和逆解历史，真实电机输入由统一停止层归零。
  void stop() {
    solver_.reset();for(int i=0;i<4;++i) {position_[i].reset();speed_[i].reset();commands_[i]=0;
      robot_four_steer_diagnostics[i].active=0;robot_vector_wheel_targets[i]=0;}
  }
 private:
  hello_world::chassis_ik_solver::ChassisIkSolver solver_;
  hello_world::motor::Motor *motors_[4]{};
  hello_world::pid::Pid position_[4],speed_[4];float commands_[4]{};
};
}
