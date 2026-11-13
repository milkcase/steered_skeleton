#ifndef STEERED_CHASSIS_IK_CONFIG_HPP_
#define STEERED_CHASSIS_IK_CONFIG_HPP_
#include "chassis_iksolver.hpp"
namespace chassis_ik_config {
// 仅保存未放行的限幅默认值。轮速/舵角速率须在安全链具备后按实车确认，不能照搬测试参数。
// 几何量固定读取已确认的 vehicle_motor_config.hpp，本文件不重复维护尺寸。
inline constexpr hello_world::chassis_ik_solver::ChassisIkSolver::Limits kLimits{};
}
#endif
