#ifndef STEERED_DRIVE_CONTROLLER_HPP_
#define STEERED_DRIVE_CONTROLLER_HPP_
#include "robot_runtime.hpp"
#include "chassis.hpp"
#include "chassis_ik_config.hpp"
#include "pid_config.hpp"
#include "vector_drive_debug.hpp"
namespace robot {
class DriveController : public DrivePort {
 public:
  // 注册本地解算器、轮/舵电机及各自 PID，构造仅写零输入。
  DriveController(Chassis *chassis,hello_world::motor::Motor *const wheels[4],
      hello_world::motor::Motor *const steers[4],hello_world::pid::Pid *const wheel_pid[4],
      hello_world::pid::Pid *const steer_pid[4],Chassis::Solver *solver,
      const hello_world::pid::Pid::Params *steer_speed_params=pid_config::kSteerSpeed) : chassis_(chassis),solver_(solver) {
    for(int i=0;i<4;++i) {wheels_[i]=wheels[i];steers_[i]=steers[i];wp_[i]=wheel_pid[i];sp_[i]=steer_pid[i];}
    if(robot_debug::drive::kVector)vector_.bind(steers);
    for(int i=0;i<4;++i)if(steer_speed_params!=nullptr)ss_[i].configure(steer_speed_params[i]);
    stop();
  }
  // 八台电机必须全配置、有新鲜反馈且 PID 单位/限幅匹配；不允许只放行一个轮组。
  bool ready(uint32_t now) const override {
    if(chassis_==nullptr || solver_==nullptr || !solver_->configured())return false;
    const auto &limits=solver_->limits();
    if(limits.max_wheel_speed_rad_s<=0 || limits.max_steer_rate_rad_s<=0)return false;
    for(int i=0;i<4;++i) {
      if(wheels_[i]==nullptr || steers_[i]==nullptr || wp_[i]==nullptr || sp_[i]==nullptr ||
          !robot_control::MotorFeedbacks().fresh(i,now) || !robot_control::MotorFeedbacks().fresh(i+4,now) ||
          !std::isfinite(wheels_[i]->vel()) || !std::isfinite(steers_[i]->angle()) || !std::isfinite(steers_[i]->vel()) ||
          wheels_[i]->get_input_type()!=hello_world::motor::InputType::kRaw ||
          steers_[i]->get_input_type()!=hello_world::motor::InputType::kCurr ||
          !pidReady(wp_[i],wheels_[i]->motor_info().raw_input_lim,0) ||
          !pidReady(sp_[i],limits.max_steer_rate_rad_s,6.283185307179586f) ||
          !pidReady(&ss_[i],steers_[i]->motor_info().curr_input_lim,0))return false;
    }
    return true;
  }
  // 使用第六项逆解得到轮轴rad/s和舵角rad；转向为位置/速度双环，驱动为单速度环。
  // 转向外环输出rad/s、内环输出A；驱动轮速度环输出C620原始报文单位。
  bool update(const ChassisMotion &command,float delta,const ModuleState &state,float dt) override {
    if(!ready(hello_world::tick::GetTickMs())) {stop();return false;}
    float angles[4],speeds[4],steer_speeds[4];
    {
      remote_input::InterruptLock lock;
      for(int i=0;i<4;++i) {angles[i]=steers_[i]->angle();speeds[i]=wheels_[i]->vel();steer_speeds[i]=steers_[i]->vel();}
    }
    if(!chassis_->updateTargets(command,delta,angles,state,true,dt)) {stop();return false;}
    const auto &targets=chassis_->targets();
    for(int i=0;i<4;++i) {
      // 舵角外环输出rad/s，舵速内环输出A；M3508仅运行轮速环，仍输出C620原始协议单位。
      const float target_speed=sp_[i]->update(targets.wheels[i].steer_rad,angles[i],dt);
      const float steer=ss_[i].update(target_speed,steer_speeds[i],dt);
      const float wheel=wp_[i]->update(targets.wheels[i].speed_rad_s,speeds[i],dt);
      if(!sp_[i]->lastUpdateOk() || !ss_[i].lastUpdateOk() || !wp_[i]->lastUpdateOk() ||
          steers_[i]->setInput(steer)!=hello_world::motor::Status::kOk ||
          wheels_[i]->setInput(wheel)!=hello_world::motor::Status::kOk) {stop();return false;}
    }
    return true;
  }
  // 所有失效和状态复位都实际执行八台 setInput(0)，清除积分与逆解历史。
  void stop() override {
    if(chassis_!=nullptr)chassis_->stop();
    for(int i=0;i<4;++i) {
      if(wheels_[i]!=nullptr)wheels_[i]->setInput(0);
      if(steers_[i]!=nullptr)steers_[i]->setInput(0);
      if(wp_[i]!=nullptr)wp_[i]->reset();
      if(sp_[i]!=nullptr)sp_[i]->reset();
      ss_[i].reset();
    }
  }
 private:
  VectorDriveDebug vector_;
 public:
  // 委托组合调试逆解/四舵独立双环，不使能其余驱动。
  bool vectorUpdate(float angle,float magnitude,float yaw,float dt,float &speed) override {return vector_.update(angle,magnitude,yaw,dt,speed);}
  // 复查本板四舵条件。
  bool vectorSafe(uint32_t now) const override {return vector_.safe(now);}
  // 只生成四舵当前帧。
  bool vectorFrame(uint8_t *data) override {return vector_.frame(data);}
  // 清组合调试私有状态。
  void vectorStop() override {vector_.stop();}
 private:
  // 正增益、有限输出及正确周期是闭环放行的必要条件，不依赖驱动静默截断。
  static bool pidReady(const hello_world::pid::Pid *pid,float limit,float period) {
    const auto &p=pid->params();
    return pid->configured() && p.kp>0 && p.ki>=0 && p.kd>=0 &&
        p.output_limit>0 && p.output_limit<=limit && std::fabs(p.error_period-period)<0.00001f;
  }
  Chassis *chassis_;Chassis::Solver *solver_;
  hello_world::motor::Motor *wheels_[4]{},*steers_[4]{};
  hello_world::pid::Pid *wp_[4]{},*sp_[4]{};
  hello_world::pid::Pid ss_[4]; // 四个速度内环独立保存I/D状态。
};
}
#endif
