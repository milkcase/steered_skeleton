#ifndef STEERED_CHASSIS_IKSOLVER_HPP_
#define STEERED_CHASSIS_IKSOLVER_HPP_
#include "module_state.hpp"

namespace hello_world::chassis_ik_solver {
class ChassisIkSolver {
 public:
  struct Limits {
    float max_wheel_speed_rad_s = 0.0f;  // 轮轴角速度上限，非电机转子 rpm；默认不放行。
    float max_steer_rate_rad_s = 0.0f;   // 相邻两周期舵角目标的最大变化率；默认不放行。
    float alignment_tolerance_rad = 0.10f;  // 驱动前允许的实际舵角与最终目标误差。
    float zero_speed_m_s = 0.0001f;      // 零速死区，避免 atan2(0,0) 导致舵角乱跳。
    float max_dt_s = 0.010f;            // 异常长周期清历史，不将旧目标继续使用。
  };
  struct BodyMotion {
    float vx = 0.0f;  // 车体系 x前，m/s；与云台系指令使用不同类型。
    float vy = 0.0f;  // 车体系 y左，m/s。
    float w = 0.0f;   // z上右手系角速度，rad/s。
  };
  struct WheelTarget {
    float speed_rad_s = 0.0f;  // 轮轴有符号角速度，后续由速度环转为电机输入。
    float steer_rad = 0.0f;    // 标定后舵角，归一化至 [-π,π)。
  };
  struct Result {
    WheelTarget wheels[4]{};  // 左前、左后、右后、右前。
    bool drive_aligned = false;  // 所有运动轮组均已对准；不等同于动作许可。
    bool valid = false;          // false 时调用方禁止使用角度目标，必须按失效归零处理。
  };

  // 创建默认停用的纯计算器，不绑定电机、CAN、遥控器或 IMU。
  ChassisIkSolver() = default;
  // 按限幅参数构造计算器；非法参数将使后续解算失败且结果无效。
  explicit ChassisIkSolver(const Limits &limits);
  // 配置限幅并清除历史；任一动力上限为零时仅输出保持角度和零轮速。
  bool configure(const Limits &limits);
  // 清空历史舵角与结果；恢复时从新反馈重新起步，不能沿用故障前的目标。
  void reset();
  // 输入云台系速度、文档定义的 delta、四个有效舵角反馈及周期，生成目标而不发送。
  bool solve(const robot::ChassisMotion &command, float delta_rad,
             const float current_steer_rad[4], float dt_s = 0.001f);
  // 返回最近结果的只读引用；只有 valid=true 时才可将其作为闭环目标。
  const Result &result() const { return result_; }
  // 返回当前限幅配置供整定记录，不能通过此引用直接修改参数。
  const Limits &limits() const { return limits_; }
  // 查询配置是否通过数值检查；合法零上限仍然是停用状态。
  bool configured() const { return configured_; }
  // 按考核说明 3.2 原式旋转云台系平移量，角速度保持不变；非法输入输出零并失败。
  static bool gimbalToBody(const robot::ChassisMotion &command, float delta_rad, BodyMotion &body);
  // 将有限角度归一化到 [-π,π)，用于最短差；非有限输入返回 NaN，由调用方拒绝。
  static float wrapAngle(float angle_rad);

 private:
  // 清空全部历史并返回失败，集中处理输入、时间和计算异常。
  bool fail();
  Limits limits_{};
  Result result_{};
  float previous_steer_[4]{};
  bool has_history_ = false;
  bool configured_ = true;
};
}  // namespace hello_world::chassis_ik_solver
#endif
