#include "chassis_iksolver.hpp"
#include "vehicle_motor_config.hpp"
#include <cmath>
#include <limits>

namespace hello_world::chassis_ik_solver {
namespace {
constexpr float kPi=3.14159265358979323846f;
constexpr float kTwoPi=2*kPi;
// 将有限值限制在正负上限内，用于每周期舵角步长。
float clamp(float value,float limit) {
  if(value>limit)return limit;
  if(value< -limit)return -limit;
  return value;
}
}  // namespace

// 构造时执行同一套配置检查，不创建任何硬件对象。
ChassisIkSolver::ChassisIkSolver(const Limits &limits) { configure(limits); }

// 检查限幅与周期参数，重新配置会清空旧目标，防止参数切换时沿用历史。
bool ChassisIkSolver::configure(const Limits &limits) {
  reset();limits_=limits;
  configured_=std::isfinite(limits.max_wheel_speed_rad_s) && limits.max_wheel_speed_rad_s>=0 &&
      std::isfinite(limits.max_steer_rate_rad_s) && limits.max_steer_rate_rad_s>=0 &&
      std::isfinite(limits.alignment_tolerance_rad) && limits.alignment_tolerance_rad>=0 &&
      limits.alignment_tolerance_rad<kPi/2 && std::isfinite(limits.zero_speed_m_s) &&
      limits.zero_speed_m_s>=0 && std::isfinite(limits.max_dt_s) && limits.max_dt_s>0;
  return configured_;
}

// 同时清除有效标志、驱动速度、舵角目标与步长历史；无效舵角不能作为回中指令。
void ChassisIkSolver::reset() {
  result_={};has_history_=false;
  for(float &angle:previous_steer_)angle=0;
}

// 解算失败统一撤销结果，调用方应归零电机输入而非继续使用旧目标。
bool ChassisIkSolver::fail() { reset();return false; }

// 先取模再转到最短角差区间，避免跨 ±π 时走大圈。
float ChassisIkSolver::wrapAngle(float angle_rad) {
  if(!std::isfinite(angle_rad))return std::numeric_limits<float>::quiet_NaN();
  float angle=std::fmod(angle_rad,kTwoPi);
  if(angle>=kPi)angle-=kTwoPi;
  if(angle< -kPi)angle+=kTwoPi;
  return angle;
}

// 使用任务书原式 R(-delta)：云台系指令转换为车体系，不能在未转换前叠加轮组力臂。
bool ChassisIkSolver::gimbalToBody(const robot::ChassisMotion &command,float delta_rad,BodyMotion &body) {
  body={};
  if(!robot::isFinite(command) || !std::isfinite(delta_rad))return false;
  const float delta=wrapAngle(delta_rad),c=std::cos(delta),s=std::sin(delta);
  body.vx=command.vx*c+command.vy*s;
  body.vy=-command.vx*s+command.vy*c;
  body.w=command.w;
  if(!std::isfinite(body.vx) || !std::isfinite(body.vy)) {body={};return false;}
  return true;
}

// 完成坐标变换、轮组逆解、等效解选取、统一轮速缩放和舵角变化率限制。
bool ChassisIkSolver::solve(const robot::ChassisMotion &command,float delta_rad,
                           const float current_steer_rad[4],float dt_s) {
  if(!configured_ || current_steer_rad==nullptr || !std::isfinite(dt_s) || dt_s<=0 ||
      dt_s>limits_.max_dt_s)return fail();
  BodyMotion body;
  if(!gimbalToBody(command,delta_rad,body))return fail();
  float measured[4],goals[4],speeds[4]{};
  for(int i=0;i<4;++i) {
    if(!std::isfinite(current_steer_rad[i]))return fail();
    measured[i]=wrapAngle(current_steer_rad[i]);
  }
  const float max_step=limits_.max_steer_rate_rad_s*dt_s;
  if(!std::isfinite(max_step))return fail();
  Result next;
  // 尚未整定任一动力上限时，保持当前/上一目标角，所有驱动轮速度为零。
  const bool enabled=limits_.max_wheel_speed_rad_s>0 && limits_.max_steer_rate_rad_s>0;
  bool all_aligned=enabled;
  float peak=0;
  for(int i=0;i<4;++i) {
    const auto &position=vehicle_config::kWheelPositions[i];
    // v_i = v_center + w×r_i；x前/y左/z上时旋转贡献为 (-w*y, +w*x)。
    const float vx=body.vx-body.w*position.y_m;
    const float vy=body.vy+body.w*position.x_m;
    if(!std::isfinite(vx) || !std::isfinite(vy))return fail();
    const float linear_speed=std::hypot(vx,vy);
    if(!std::isfinite(linear_speed))return fail();
    const float previous=has_history_?previous_steer_[i]:measured[i];
    goals[i]=previous;
    if(enabled && linear_speed>limits_.zero_speed_m_s) {
      goals[i]=wrapAngle(std::atan2(vy,vx));
      speeds[i]=linear_speed/vehicle_config::kWheelRadiusM;
      if(!std::isfinite(speeds[i]))return fail();
      // 比较的是实际反馈，不是旧目标；超过 π/2 才翻转，等于时保留原解。
      if(std::fabs(wrapAngle(goals[i]-measured[i]))>kPi/2) {
        goals[i]=wrapAngle(goals[i]+kPi);speeds[i]=-speeds[i];
      }
      if(std::fabs(wrapAngle(goals[i]-measured[i]))>limits_.alignment_tolerance_rad)
        all_aligned=false;
    }
    const float step=clamp(wrapAngle(goals[i]-previous),max_step);
    next.wheels[i].steer_rad=enabled?wrapAngle(previous+step):previous;
    if(!std::isfinite(next.wheels[i].steer_rad))return fail();
    // 实际反馈和限速后的舵角命令都需接近最终目标，避免历史命令尚未赶上就驱动。
    if(enabled && linear_speed>limits_.zero_speed_m_s &&
        std::fabs(wrapAngle(goals[i]-next.wheels[i].steer_rad))>limits_.alignment_tolerance_rad)
      all_aligned=false;
    if(std::fabs(speeds[i])>peak)peak=std::fabs(speeds[i]);
  }
  // 整体等比例缩放四轮速度，避免各轮单独截断而改变理想运动的速度比例。
  const float scale=peak>limits_.max_wheel_speed_rad_s?limits_.max_wheel_speed_rad_s/peak:1.0f;
  for(int i=0;i<4;++i) {
    // 任一运动轮组未对准最终舵角，四个驱动轮统一暂缓，转向目标仍可逐步接近。
    next.wheels[i].speed_rad_s=all_aligned?speeds[i]*scale:0.0f;
    if(!std::isfinite(next.wheels[i].speed_rad_s))return fail();
    previous_steer_[i]=next.wheels[i].steer_rad;
  }
  has_history_=true;next.drive_aligned=all_aligned;next.valid=true;result_=next;
  return true;
}
}  // namespace hello_world::chassis_ik_solver
