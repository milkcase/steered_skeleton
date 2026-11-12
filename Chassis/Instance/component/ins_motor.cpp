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
hello_world::motor::Motor *GetMotorWheel(int idx) {
  /* TODO【新人任务】:
   * 返回第 idx 号轮电机（M3508）单例。
   *
   * 你要做：
   *   1. 静态定义 4 个轮电机对象并存入数组（可用 hello_world::motor::CreateMotor
   *      (hello_world::motor::MotorType::kM3508, id, opt) 工厂，或直接构造 M3508）。
   *   2. 校验 idx ∈ [0,4)，越界时用 HW_ASSERT 或直接断言。
   *   3. 返回对应元素的地址。
   *   4. 每个电机的 CAN id、方向、减速比、角度偏移按实机接线配置（见考核说明文档）。
   *
   * 约束 / 提示：
   *   - 电机设备类已提供：inc/motor_M3508.hpp（M3508）、inc/motor.hpp（CreateMotor 工厂、
   *     OptionalParams 配置项）。
   *   - 电机按「左前 / 左后 / 右后 / 右前」命名，id 与方向见实机接线图。
   */
  // 先检查下标和实物确认标记，未确认时不创建可用实例。
  if (idx < 0 || idx >= 4) return nullptr;
  const auto &cfg = vehicle_config::kWheels[idx];
  if (!cfg.configured || cfg.id < 1 || cfg.id > 4 ||
      cfg.model != vehicle_config::Model::M3508 ||
      cfg.board != vehicle_config::Board::Chassis ||
      cfg.bus != vehicle_config::CanBus::Can2) return nullptr;
  // 同一组设备 ID 不得重复；重复配置不能参与反馈分发。
  for (int i = 0; i < 4; ++i) {
    if (i != idx && vehicle_config::kWheels[i].configured &&
        vehicle_config::kWheels[i].id == cfg.id) return nullptr;
  }
  static hello_world::motor::M3508 motors[4];
  static bool initialized[4] = {};
  if (!initialized[idx]) {
    hello_world::motor::OptionalParams opt;
    opt.dir = cfg.reversed ? hello_world::motor::kDirRev : hello_world::motor::kDirFwd;
    opt.angle_offset = cfg.offset_rad;
    // 保留驱动内置减速比；仅初始化反馈与零输入，不使电机转动。
    motors[idx].init(cfg.id, opt);
    motors[idx].setInput(0.0f);
    initialized[idx] = true;
  }
  return &motors[idx];
}

hello_world::motor::Motor *GetMotorSteer(int idx) {
  /* TODO【新人任务】:
   * 返回第 idx 号舵电机（GM6020）单例，做法同 GetMotorWheel()。
   * 舵电机控制的是轮子的转向角（舵向轮底盘），与轮电机（驱动轮速）配合完成运动；
   * 设备类见 inc/motor_GM6020.hpp，工厂 MotorType::kGM6020。
   */
  if (idx < 0 || idx >= 4) return nullptr;
  const auto &cfg = vehicle_config::kSteers[idx];
  if (!cfg.configured || cfg.id < 1 || cfg.id > 4 ||
      cfg.model != vehicle_config::Model::GM6020 ||
      cfg.board != vehicle_config::Board::Chassis ||
      cfg.bus != vehicle_config::CanBus::Can1) return nullptr;
  for (int i = 0; i < 4; ++i) {
    if (i != idx && vehicle_config::kSteers[i].configured &&
        vehicle_config::kSteers[i].id == cfg.id) return nullptr;
  }
  static hello_world::motor::GM6020 motors[4];
  static bool initialized[4] = {};
  if (!initialized[idx]) {
    hello_world::motor::OptionalParams opt;
    opt.dir = cfg.reversed ? hello_world::motor::kDirRev : hello_world::motor::kDirFwd;
    opt.angle_offset = cfg.offset_rad;
    // GM6020 使用默认电压输入，舵角统一归一化到 [-π, π)。
    opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
    motors[idx].init(cfg.id, opt);
    motors[idx].setInput(0.0f);
    initialized[idx] = true;
  }
  return &motors[idx];
}

hello_world::motor::Motor *GetMotorYaw() {
  const auto &cfg = vehicle_config::kYaw;
  // 未知型号保持不可用，避免按错误的速度、力矩量程编码。
  if (!cfg.configured || cfg.model != vehicle_config::Model::DM_J4310 ||
      cfg.id < 1 || cfg.id > 0x0F ||
      cfg.board != vehicle_config::Board::Chassis ||
      cfg.bus != vehicle_config::CanBus::Can2) return nullptr;
  static hello_world::motor::DM_J4310 motor;
  static bool initialized = false;
  if (!initialized) {
    hello_world::motor::OptionalParams opt;
    opt.input_type = hello_world::motor::InputType::kTorq;
    opt.dir = cfg.reversed ? hello_world::motor::kDirRev : hello_world::motor::kDirFwd;
    opt.angle_offset = cfg.offset_rad;
    opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
    // 禁止自动使能和自动切换模式；后续由通过安全检查的状态机决定使能。
    motor.init(cfg.id, opt, false, false);
    motor.setInput(0.0f);
    initialized = true;
  }
  return &motor;
}

/* Private function definitions ----------------------------------------------*/
