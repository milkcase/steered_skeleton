/**
 *******************************************************************************
 * @file      : motor_GM6020.hpp
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
 *     归属后再调用 decode，并根据 GetTxId 将 encode 出的 8 字节填入对应 CAN 帧发出。
 *     注意：同一条发送报文下的多个电机（1~4 或 5~7）共用一个 8 字节缓冲区分别 encode
 *     后，再以 GetTxId 得到的共享 ID 一次性发出
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_GM6020_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_GM6020_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

class GM6020 : public Motor
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  GM6020(void) = default;
  /**
   * @brief       GM6020 初始化
   * @param        id: 电机 ID，1~7
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit GM6020(
      uint8_t id, const OptionalParams& opt = OptionalParams());
  GM6020(const GM6020&) = default;
  GM6020& operator=(const GM6020& other);
  GM6020(GM6020&& other);
  GM6020& operator=(GM6020&& other);

  virtual ~GM6020(void) = default;

  /* 重载方法 */

  /**
   * @brief       将电调发回的 8 字节 CAN 反馈报文进行解包
   * @param        data: 电调发回的 8 字节 CAN 报文
   * @retval       是否解包成功
   * @note        新人需先在自己的 CAN 层根据反馈 ID 判断报文归属，再调用此接口
   */
  virtual bool decode(const uint8_t data[8]) override;

  /**
   * @brief       将要发给电调的期望输入编码为 8 字节 CAN 控制报文
   * @param        data: 输出的 8 字节 CAN 报文缓冲区
   * @retval       是否编码成功
   * @note        新人需根据该电机所属发送 ID（见 GetTxId）将 data 填入 CAN 帧发出
   */
  virtual bool encode(uint8_t data[8]) override;

  /**
   * @brief       设定发给电调的期望值
   * @param        input: 发给电调的期望值
   * @retval       设定状态，可能的返回值有：
   *   @arg        Status::kOk: 设定成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   *   @arg        Status::kInputValueOverflow: 设定值超出范围
   * @note        1. 期望值的物理意义与电机当前的输入类型有关，可使用 get_input_type
   *              方法查看
   *              2. 设定的期望值会自动被限制到允许的范围内，当前实际的设定值可以通过
   *              getInput 方法查看
   */
  virtual Status setInput(float input) override;

  /**
   * @brief       设置点击的输入类型
   * @param        input_type: 期望输入类型，可选值为：
   *   @arg        InputType::kRaw: 原始报文输入
   *   @arg        InputType::kCurr: 转子端电流输入
   * @retval       设置状态，可能的返回值有：
   *   @arg        Status::kOk: 设置成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   * @note        None
   */
  virtual Status set_input_type(InputType input_type) override;

  /* 配置方法 */

  /**
   * @brief       GM6020 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~7
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams());

  /* 数据修改与获取 */

  uint8_t temp(void) const { return temp_; }

  /**
   * @brief       根据 id 获取对应的发送 ID
   * @param        id: 电机 ID，范围 1~7
   * @param        input_type: 电机输入类型
   * @retval       对应的发送 ID
   * @note        1~4 为同一条发送报文，5~7 为同一条发送报文
   */
  static uint32_t GetTxId(uint8_t id, InputType input_type)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 7, "Error id: %d", id);
    HW_ASSERT(input_type == InputType::kRaw ||
                  input_type == InputType::kCurr,
              "Error input type: %d", input_type);
#pragma endregion
    if (input_type == InputType::kRaw) {
      return id <= 4 ? kRawTx1_4_ : kRawTx5_7_;
    } else {
      return id <= 4 ? kCurrTx1_4_ : kCurrTx5_7_;
    }
  }

  /**
   * @brief       根据 id 获取对应的接收 ID
   * @param        id: 电机 ID，范围 1~7
   * @retval       对应的接收 ID
   * @note        None
   */
  static uint32_t GetRxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 7, "Error id: %d", id);
#pragma endregion

    return kRx0_ + id;
  }

 private:
  /* 电机状态 */

  uint8_t temp_ = 0;  ///* 电机温度

  static constexpr uint32_t kRawTx1_4_ = 0x1FF;
  static constexpr uint32_t kRawTx5_7_ = 0x2FF;
  static constexpr uint32_t kCurrTx1_4_ = 0x1FE;
  static constexpr uint32_t kCurrTx5_7_ = 0x2FE;
  static constexpr uint32_t kRx0_ = 0x204;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_GM6020_HPP_ */
