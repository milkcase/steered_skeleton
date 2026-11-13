#include "gimbal_controller.hpp"
#include "gimbal_control_config.hpp"
#include <limits>
#include <cstddef>
using robot::Gimbal;
using hello_world::pid::Pid;
using hello_world::motor::DaMiao;
constexpr float pi=3.14159265358979323846f;
// 提供裸 ARM 测试内存填充实现，不访问外设。
extern "C" void *memset(void *p,int v,std::size_t n) {
  auto *q=static_cast<unsigned char *>(p);for(std::size_t i=0;i<n;++i)q[i]=v;return p;
}
// 提供裸 ARM 测试的非重叠内存复制实现。
extern "C" void *memcpy(void *p,const void *s,std::size_t n) {
  auto *q=static_cast<unsigned char *>(p);const auto *a=static_cast<const unsigned char *>(s);
  for(std::size_t i=0;i<n;++i)q[i]=a[i];return p;
}
// 比较浮点结果，容许正常舍入误差。
bool near(float a,float b,float tolerance=0.0001f) { return std::fabs(a-b)<tolerance; }
// 仅供数学测试的动作条件，不写入生产配置，也不声称实际设备已就绪。
Gimbal::Safety safe() { return {true,true,true,true,true,true}; }
// 仅供测试构造已工作的描述；真实状态机在第九项根据真实输入迁移。
robot::ModuleState working() {
  return {robot::LifeState::Working,robot::ControlSource::RemoteControl,
      robot::ChassisMode::Separate,robot::GimbalMode::JointPosition};
}
// 仅供模拟测试选择非零限制与已知 pitch 范围，不能用于实车。
Gimbal::Params limits() { return {3,1000,true,-0.5f,0.5f,0.05f,0.01f}; }
// 按轴配置真实 PID，角度回绕仅用于 yaw，输出单位为 N·m。
bool configurePid(Pid &pid,bool yaw,float kp=2,float ki=0,float kd=0) {
  return pid.configure({kp,ki,kd,1,3,0.01f,yaw?2*pi:0});
}
#define CHECK(value) do {if(!(value))return __LINE__;}while(0)

// 验证实际位置环与 PID，电机仅用记录型替身；成功返回零，失败返回行号。
extern "C" int run_tests() {
  DaMiao motor;Pid pid;configurePid(pid,true);
  Gimbal yaw(Gimbal::Axis::Yaw,&motor,&pid,limits());
  auto state=working();auto safety=safe();
  motor.position=359*pi/180;
  CHECK(yaw.update({pi/180,0},state,safety));
  CHECK(near(yaw.result().error_rad,2*pi/180) && motor.input>0 && near(motor.input,4*pi/180));
  yaw.stop();motor.position=pi/180;
  CHECK(yaw.update({359*pi/180,0},state,safety));
  CHECK(near(yaw.result().error_rad,-2*pi/180) && motor.input<0);
  yaw.stop();motor.position=0;
  CHECK(yaw.update({10*pi+0.2f,0},state,safety));CHECK(near(yaw.result().error_rad,0.2f));

  // 每个许可缺失时均清除旧输出、积分、目标与 MIT 残留刚度。
  for(int i=0;i<6;++i) {
    safety=safe();CHECK(yaw.update({0.2f,0},state,safety));CHECK(motor.input>0);
    motor.mit_position=motor.mit_velocity=motor.mit_kp=motor.mit_kd=1;
    bool *flags[]={&safety.remote_valid,&safety.command_valid,&safety.emergency_released,
        &safety.imu_ready,&safety.feedback_valid,&safety.peer_online};
    *flags[i]=false;CHECK(!yaw.update({0.2f,0},state,safety));
    CHECK(motor.input==0 && !yaw.result().valid && pid.integralTerm()==0);
    CHECK(motor.mit_position==0 && motor.mit_velocity==0 && motor.mit_kp==0 && motor.mit_kd==0);
  }
  safety=safe();
  state.life=robot::LifeState::Dead;CHECK(!yaw.update({0.2f,0},state,safety));
  state.life=robot::LifeState::Resurrection;CHECK(!yaw.update({0.2f,0},state,safety));
  state=working();state.source=robot::ControlSource::None;CHECK(!yaw.update({0.2f,0},state,safety));
  state=working();state.gimbal_mode=robot::GimbalMode::Disabled;CHECK(!yaw.update({0.2f,0},state,safety));
  state=working();state.chassis_mode=static_cast<robot::ChassisMode>(255);
  CHECK(!yaw.update({0.2f,0},state,safety));state=working();
  motor.enabled=false;CHECK(!yaw.update({0.2f,0},state,safety) && !motor.enabled);motor.enabled=true;
  motor.input_type=hello_world::motor::InputType::kRaw;CHECK(!yaw.update({0.2f,0},state,safety));
  motor.input_type=hello_world::motor::InputType::kTorq;
  motor.info.torq_input_lim=2;CHECK(!yaw.update({0.2f,0},state,safety));motor.info.torq_input_lim=7;
  motor.reject_nonzero=true;CHECK(!yaw.update({0.2f,0},state,safety) && motor.input==0);motor.reject_nonzero=false;

  // 首周期、停机恢复均从反馈起步，目标跳变被限制为 rate×dt。
  auto params=limits();params.max_target_rate_rad_s=1;
  CHECK(yaw.configure(params));motor.position=0;
  CHECK(yaw.update({1,0},state,safety));CHECK(near(yaw.result().target_rad,0.001f));
  CHECK(yaw.update({1,0},state,safety));CHECK(near(yaw.result().target_rad,0.002f));
  yaw.stop();motor.position=0.3f;
  CHECK(yaw.update({1,0},state,safety));CHECK(near(yaw.result().target_rad,0.301f));
  CHECK(yaw.configure(limits()));

  // pitch 线性限位、边界向外力矩阻断、越界后失效，不绕回另一侧。
  DaMiao pitch_motor;Pid pitch_pid;configurePid(pitch_pid,false);
  Gimbal pitch(Gimbal::Axis::Pitch,&pitch_motor,&pitch_pid,limits());
  CHECK(pitch.update({0,2},state,safety));
  CHECK(near(pitch.result().target_rad,0.5f) && pitch.result().limited && pitch_motor.input>0);
  pitch.stop();CHECK(pitch.update({0,-2},state,safety));
  CHECK(near(pitch.result().target_rad,-0.5f) && pitch_motor.input<0);
  pitch_motor.position=0.49f;pitch.stop();CHECK(pitch.update({0,2},state,safety));
  CHECK(pitch_motor.input==0 && pitch.result().limited && pitch_pid.integralTerm()==0);
  CHECK(pitch.update({0,-0.5f},state,safety) && pitch_motor.input<0);
  pitch_motor.position=-0.49f;pitch.stop();CHECK(pitch.update({0,-2},state,safety));CHECK(pitch_motor.input==0);
  pitch_motor.position=0.501f;CHECK(!pitch.update({0,0},state,safety));CHECK(pitch_motor.input==0);
  pitch_motor.position=0;params=limits();params.pitch_limits_confirmed=false;
  pitch.configure(params);CHECK(!pitch.update({0,0.1f},state,safety));
  params=limits();params.pitch_min_rad=-pi;CHECK(!pitch.configure(params));
  params=limits();params.pitch_guard_rad=0.5f;CHECK(!pitch.configure(params));
  params=limits();params.max_torque_nm=-1;CHECK(!pitch.configure(params));
  CHECK(pitch.configure(limits()));

  // 时间、目标、反馈、PID 参数错误均不得保留上次力矩。
  const float nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
  CHECK(!yaw.update({nan,0},state,safety));CHECK(!pitch.update({0,inf},state,safety));
  motor.position=nan;CHECK(!yaw.update({0.1f,0},state,safety));motor.position=0;
  CHECK(!yaw.update({0.1f,0},state,safety,0));CHECK(!yaw.update({0.1f,0},state,safety,0.011f));
  CHECK(!yaw.update({0.1f,0},state,safety,nan));
  CHECK(!yaw.update({0.1f,0},state,safety,-0.001f));CHECK(!yaw.update({0.1f,0},state,safety,inf));
  pid.configure({.kp=-1,.output_limit=3,.error_period=2*pi});CHECK(!yaw.update({0.1f,0},state,safety));
  pid.configure({.kp=2,.output_limit=4,.error_period=2*pi});CHECK(!yaw.update({0.1f,0},state,safety));
  pid.configure({.kp=2,.output_limit=3});CHECK(!yaw.update({0.1f,0},state,safety));
  configurePid(pid,true);Gimbal missing(Gimbal::Axis::Yaw,nullptr,&pid,limits());
  CHECK(!missing.update({0.1f,0},state,safety));
  Gimbal no_pid(Gimbal::Axis::Yaw,&motor,nullptr,limits());CHECK(!no_pid.update({0.1f,0},state,safety));
  Gimbal disabled(Gimbal::Axis::Yaw,&motor,&pid,gimbal_control_config::kYaw);
  CHECK(!disabled.update({0.1f,0},state,safety) && motor.input==0);

  // 状态模式切换清积分；正常连续调用则保持本回路自己的积分历史。
  configurePid(pid,true,0,1);yaw.configure(limits());
  for(int i=0;i<10;++i)CHECK(yaw.update({0.3f,0},state,safety));
  const float old_integral=pid.integralTerm();CHECK(old_integral>0);
  state.chassis_mode=robot::ChassisMode::Follow;CHECK(yaw.update({0.3f,0},state,safety));
  CHECK(pid.integralTerm()<old_integral);

  // 两轴分别在有阻尼的数学对象上运行闭环，检查收敛和力矩限幅；非实车整定参数。
  for(int axis=0;axis<2;++axis) {
    DaMiao plant;Pid controller;configurePid(controller,axis==0,8,0,4);
    params=limits();params.max_target_rate_rad_s=1;
    Gimbal loop(axis==0?Gimbal::Axis::Yaw:Gimbal::Axis::Pitch,&plant,&controller,params);
    float velocity=0,peak=0;const float goal=axis==0?0.4f:0.2f;
    for(int i=0;i<5000;++i) {
      CHECK(loop.update(axis==0?robot::GimbalMotion{goal,0}:robot::GimbalMotion{0,goal},state,safety));
      CHECK(std::fabs(plant.input)<=3);
      velocity+=0.001f*(plant.input-2*velocity);plant.position+=0.001f*velocity;
      if(plant.position>peak)peak=plant.position;
    }
    CHECK(near(plant.position,goal,0.003f));CHECK(peak<goal+0.01f);
  }
  return 0;
}
