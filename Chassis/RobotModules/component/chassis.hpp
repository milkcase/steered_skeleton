/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "chassis_iksolver.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】声明底盘运动模块类 Chassis（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Chassis 类，负责把底盘速度指令 (vx, vy, w) 逆解为每个轮 / 舵的控制量：
 *      舵向轮运动学逆解（4 个 GM6020 舵电机 + 4 个 M3508 轮电机），输出每个舵的
 *      目标转角与每个轮的轮速（或电流 / 转矩指令）。
 *   2. 提供接口让 Robot 下发目标速度、读取解算结果。
 *
 * 约束 / 提示：
 *   - 几何参数（轮半径、轮距、轮心距、轮位坐标）见考核说明文档。
 *   - 逆解公式需自己推导（舵向轮注意舵角多解与轮速符号翻转），网上「舵向轮 / 麦克纳姆轮逆运动学」资料可参考。
 * ==========================================================================*/
namespace robot {
class Chassis {
 public:
  using Solver=hello_world::chassis_ik_solver::ChassisIkSolver;
  // 绑定纯数学逆解器；空指针对象保持不可用，不初始化或使能电机。
  explicit Chassis(Solver *solver=nullptr) : solver_(solver) {}
  // 在状态及输入有效性通过后更新轮组目标；此处不生成模式 w，也不下发电机命令。
  bool updateTargets(const ChassisMotion &command,float delta_rad,const float steer_feedback[4],
                     const ModuleState &state,bool inputs_valid,float dt_s=0.001f);
  // 撤销目标有效标志并清除逆解历史；实际 setInput(0) 由后续 Robot 最外层执行。
  void stop();
  // 返回本周期目标；valid=false 时不可把其中的零舵角当成回中指令。
  const Solver::Result &targets() const { return targets_; }
 private:
  Solver *solver_=nullptr;
  Solver::Result targets_{};
};
}
#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
