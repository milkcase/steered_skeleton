/**
 *******************************************************************************
 * @file      :chassis.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"
#include "robot_control_config.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】实现 chassis.hpp 中声明的底盘运动学。
 *
 * 你要做：
 *   1. 逆解：输入 (vx, vy, w)，输出每个舵的目标转角与每个轮的轮速 / 电流指令。
 *   2. 舵向轮需处理舵角连续性：当目标舵角与当前舵角相差超过 π/2 时，选择等效的
 *      另一组解（舵角反向、轮速取反），避免舵机大角度回摆。
 *   3. 对解算结果做限幅，避免超速。
 *
 * 约束 / 提示：
 *   - 解算结果会经电机组件的 encode() 下发，注意输入量的类型（电流 / 转矩）。
 * ==========================================================================*/

// 未处于有效遥控 Working 模式、输入失效或解算失败时撤销全部目标，不能沿用上一周期。
bool robot::Chassis::updateTargets(const ChassisMotion &command,float delta_rad,const float steer_feedback[4],
                                  const ModuleState &state,bool inputs_valid,float dt_s) {
  if(solver_==nullptr || !inputs_valid || !isValid(state) || state.life!=LifeState::Working ||
      state.source!=ControlSource::RemoteControl || state.chassis_mode==ChassisMode::Disabled) {
    stop();return false;
  }
  const bool spin_mode=main_tuning::enabled && state.chassis_mode==ChassisMode::Spin;
  const float wheel_limit=spin_mode?robot_control::spinWheelTargetLimit(command.w):0.0f;
  // 偏角只叠加到平移坐标变换，后续逆解仍用原平移幅度和原自转速度计算各轮目标。
  constexpr float offset_deg=ROBOT_SPIN_TRANSLATION_OFFSET_DEG;
  static_assert(offset_deg>=-180.0f && offset_deg<=180.0f,"小陀螺平移补偿角必须在 -180 至 +180 度内");
  constexpr float offset_rad=offset_deg*0.017453292519943295f;
  const float translation_delta=spin_mode?delta_rad+offset_rad:delta_rad;
  if(!solver_->solve(command,translation_delta,steer_feedback,dt_s,wheel_limit)) { stop();return false; }
  targets_=solver_->result();
  return true;
}

// 清空内存中的目标与速率历史，不访问电机；后续输出层仍必须独立执行失效归零。
void robot::Chassis::stop() {
  targets_={};
  if(solver_!=nullptr)solver_->reset();
}
