/**
 *******************************************************************************
 * @file      : motor_GM6020.cpp
 * @brief     : 大疆 GM6020 电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *  V1.2.0      2024-12-13      Jinletian       1. 增加电流控制模式
 *  V2.0.0      2026-09-18      Hello World     1. 改装版：去通信耦合，暴露纯字节接口
 *******************************************************************************
 * @attention :
 *  1. 请先查看 motor_base.hpp 中的注意事项
 *  2. 电机 ID 范围为 1~7，其中 1~4 为同一条发送报文，5~7 为同一条发送报文
 *  3. 电压控制使用 InputType::kRaw 输入方式，电流控制使用 InputType::kCurr 输入方式
 *  4. raw2x 与 x2raw 方法只适用于反馈报文中的转换关系，无法用于控制报文
 *  5. 可额外获得的数据为电机温度 temp
 *  6. V1.2.0版本组件适配1.0.11.2版本固件，使用前请先升级固件版本
 *  7. 如需使用电流控制，需在上位机参数设置选项中打开电流环开关，此时不能使用电压输入方式
 *  8. 改装版中 decode/encode 为纯字节接口：新人需在自己的 CAN 层根据反馈 ID 判断报文
 *     归属后再调用 decode，并根据 GetTxId 将 encode 出的 8 字节填入对应 CAN 帧发出
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_GM6020.hpp"

#include "assert.hpp"

namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

const MotorBaseInfo kGM6020MotorBaseInfo{
    .raw_input_lim = 25000,
    .torq_input_lim = kInvalidValue,  ///* 无效
    .curr_input_lim = 3.0,
    .torq_const = 0.741f,
    .redu_rat = 1.0f,
    .angle_rat = 2 * PI / 8191,
    .vel_rat = 2 * PI / 60,
    .curr_rat = 3.0f / 16384,
    .torq_rat = kInvalidValue,
    .cross_0_value = 8191U,
    .raw_mapping_type = RawMappingType::kCurr,
};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

GM6020::GM6020(uint8_t id, const OptionalParams& opt) : Motor()
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(1 <= id && id <= 7, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev,
            "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kRaw ||
                opt.input_type == InputType::kCurr,
            "Error input type: %d", opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0,
            "Error external reduction ration: %f", opt.ex_redu_rat);
  HW_ASSERT(opt.max_raw_input_lim > 0,
            "Error max raw input limit: %f", opt.max_raw_input_lim);
#pragma endregion

  motor_info_ = kGM6020MotorBaseInfo;
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_raw_input = motor_info_.raw_input_lim;
  if (opt.max_raw_input_lim != std::numeric_limits<float>::max()) {
    max_raw_input = std::min(max_raw_input, opt.max_raw_input_lim);
  }
  float max_curr_input = motor_info_.curr_input_lim;
  if (opt.max_curr_input_lim != std::numeric_limits<float>::max()) {
    max_curr_input = std::min(max_curr_input, opt.max_curr_input_lim);
  }

  motor_info_.raw_input_lim = max_raw_input;
  motor_info_.torq_input_lim = kInvalidValue;
  motor_info_.curr_input_lim = max_curr_input;
}

GM6020& GM6020::operator=(const GM6020& other)
{
  if (this != &other) {
    Motor::operator=(other);

    temp_ = other.temp_;
  }

  return *this;
}

GM6020::GM6020(GM6020&& other) : Motor(std::move(other))
{
  temp_ = other.temp_;
}

GM6020& GM6020::operator=(GM6020&& other)
{
  if (this != &other) {
    Motor::operator=(std::move(other));

    temp_ = other.temp_;
  }

  return *this;
}

bool GM6020::decode(const uint8_t data[8])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data, "data is nullptr");
#pragma endregion

  uint16_t raw_angle = static_cast<uint16_t>((data[0] << 8) | data[1]);

  /* 判断是否旋转了一圈 */
  float round = round_;
  float delta_angle = raw_angle - last_raw_angle_;
  if (fabsf(delta_angle) > motor_info_.cross_0_value * kCross0ValueThres) {
    delta_angle < 0 ? round++ : round--;

    /* 避免圈数溢出 */
    if (motor_info_.angle_range != AngleRange::kNegInfToPosInf) {
      round = fmodf(round, motor_info_.redu_rat);
    }
  }

  float actual_ang =
      static_cast<float>(motor_info_.dir) *
      (raw_angle * motor_info_.angle_rat + round * 2 * PI) /
      motor_info_.redu_rat;
  float raw = static_cast<float>(motor_info_.dir) *
              static_cast<int16_t>((data[4] << 8) | data[5]);

  /* 统一对电机状态赋值 */
  round_ = round;
  actual_angle_ = normAngle(actual_ang);
  angle_ = normAngle(actual_ang - motor_info_.angle_offset);
  vel_raw_ = static_cast<float>(motor_info_.dir) *
             static_cast<int16_t>((data[2] << 8) | data[3]) *
             motor_info_.vel_rat / motor_info_.redu_rat;
  vel_ = calcVel();
  torq_ = raw2Torq(raw);
  curr_ = raw2Curr(raw);
  temp_ = data[6];

  last_raw_angle_ = raw_angle;

  return true;
}

bool GM6020::encode(uint8_t data[8])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data, "data is nullptr");
#pragma endregion

  uint8_t index = (motor_info_.id - 1) % 4;
  int16_t input = 0;

  if (motor_info_.input_type == InputType::kRaw) {
    input = hello_world::Bound(
        static_cast<float>(motor_info_.dir) * raw_input_,
        motor_info_.raw_input_lim, -motor_info_.raw_input_lim);
  } else if (motor_info_.input_type == InputType::kCurr) {
    float curr_input = hello_world::Bound(
        static_cast<float>(motor_info_.dir) * curr_input_,
        motor_info_.curr_input_lim, -motor_info_.curr_input_lim);
    input = curr2Raw(curr_input);
  }

  data[2 * index] = input >> 8;
  data[2 * index + 1] = input;

  return true;
}

Status GM6020::setInput(float input)
{
  switch (motor_info_.input_type) {
    case InputType::kRaw:
      raw_input_ = hello_world::Bound(
          input, motor_info_.raw_input_lim, -motor_info_.raw_input_lim);
      if (fabsf(raw_input_) < fabsf(input)) {
        return Status::kInputValueOverflow;
      } else {
        return Status::kOk;
      }
      break;
    case InputType::kCurr:
      curr_input_ = hello_world::Bound(
          input, motor_info_.curr_input_lim, -motor_info_.curr_input_lim);
      raw_input_ = curr2Raw(curr_input_);
      if (fabsf(curr_input_) < fabsf(input)) {
        return Status::kInputValueOverflow;
      } else {
        return Status::kOk;
      }
      break;
    default:
      return Status::kInputTypeError;
  }
}

Status GM6020::set_input_type(InputType input_type)
{
  if (input_type == InputType::kRaw || input_type == InputType::kCurr) {
    motor_info_.input_type = input_type;
    return Status::kOk;
  } else {
    return Status::kInputTypeError;
  }
}

void GM6020::init(uint8_t id, const OptionalParams& opt)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(1 <= id && id <= 7, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev,
            "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kRaw ||
                opt.input_type == InputType::kCurr,
            "Error input type: %d", opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0,
            "Error external reduction ration: %f", opt.ex_redu_rat);
  HW_ASSERT(opt.max_raw_input_lim > 0,
            "Error max raw input limit: %f", opt.max_raw_input_lim);
#pragma endregion

  motor_info_ = kGM6020MotorBaseInfo;
  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_raw_input = motor_info_.raw_input_lim;
  if (opt.max_raw_input_lim != std::numeric_limits<float>::max()) {
    max_raw_input = std::min(max_raw_input, opt.max_raw_input_lim);
  }
  float max_curr_input = motor_info_.curr_input_lim;
  if (opt.max_curr_input_lim != std::numeric_limits<float>::max()) {
    max_curr_input = std::min(max_curr_input, opt.max_curr_input_lim);
  }

  motor_info_.raw_input_lim = max_raw_input;
  motor_info_.torq_input_lim = kInvalidValue;
  motor_info_.curr_input_lim = max_curr_input;

  angle_ = 0.0f;
  vel_ = 0.0f;
  vel_raw_ = 0.0f;
  torq_ = 0.0f;
  curr_ = 0.0f;
  round_ = 0.0f;
  actual_angle_ = 0.0f;

  raw_input_ = 0;
  torq_input_ = 0;
  curr_input_ = 0;

  temp_ = 0;
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world
