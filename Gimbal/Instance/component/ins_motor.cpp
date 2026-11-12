/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
#include "vehicle_motor_config.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
// 云台板不负责 yaw 执行，固定返回空指针，避免在共享 CAN2 上重复控制。
hello_world::motor::Motor *GetMotorYaw() {
  /* TODO【新人任务】:
   * 返回云台 yaw 关节电机单例（DM-J4310）。
   *
   * 你要做：
   *   1. 静态定义 yaw 电机对象（CreateMotor(kDM_J4310, id, opt) 或直接构造 DM_J4310）。
   *   2. 配置 id / 方向 / 减速比 / 角度偏移，并确保使能（auto_enable / enable()），
   *      达妙电机失能状态下不会反馈数据。
   *   3. 返回其地址。
   *
   * 约束 / 提示：
   *   - 设备类见 inc/motor_DaMiao.hpp（DM_J4310），工厂 MotorType::kDM_J4310。
   *   - yaw 关节注意 360° 连续旋转的角度过零处理（MIT 位置控制内部不做多圈累计，
   *     见 motor_DaMiao.hpp 注释）。
   */
  // 实车 yaw 接底盘板 CAN2：本板不创建第二个对象，不允许跨板直接调用。
  // 后续通过板间 CAN 交换 yaw 目标与反馈；本板 getter 暂保持不可用。
  return nullptr;
}

// 返回云台板负责的 pitch 电机单例；按 J4310 零力矩初始化，关闭自动使能。
hello_world::motor::Motor *GetMotorPitch() {
  /* TODO【新人任务】:
   * 返回云台 pitch 关节电机单例（DM-J4310），做法同 GetMotorYaw()。
   * pitch 关节需用反馈角度做限位，防止堵转 / 撞结构。
   */
  const auto &cfg = vehicle_config::kPitch;
  if (!cfg.configured || cfg.model != vehicle_config::Model::DM_J4310 ||
      cfg.id < 1 || cfg.id > 0x0F ||
      cfg.board != vehicle_config::Board::Gimbal ||
      cfg.bus != vehicle_config::CanBus::Can2) return nullptr;
  static hello_world::motor::DM_J4310 motor;
  static bool initialized = false;
  if (!initialized) {
    hello_world::motor::OptionalParams opt;
    opt.input_type = hello_world::motor::InputType::kTorq;
    opt.dir = cfg.reversed ? hello_world::motor::kDirRev : hello_world::motor::kDirFwd;
    opt.angle_offset = cfg.offset_rad;
    opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
    // 初始化仅准备零力矩，禁止自动使能；pitch 机械限位留在后续位置环实现。
    motor.init(cfg.id, opt, false, false);
    motor.setInput(0.0f);
    initialized = true;
  }
  return &motor;
}

/* Private function definitions ----------------------------------------------*/
