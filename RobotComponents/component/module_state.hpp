/**
 *******************************************************************************
 * @file      :module_state.hpp
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
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】定义底盘板与云台板共享的状态类型与枚举（namespace robot）。
 *
 * 本文件会被底盘板、云台板同时编译，只能放「纯类型 / 枚举 / 内联函数」，
 * 严禁依赖任何具体硬件（电机 / CAN / HAL），否则另一块板编不过。
 *
 * 你要做：
 *   1. 生命状态枚举：至少区分「上电未就绪 / 恢复自检中 / 正常工作中」三种，
 *      供 RobotModules 的状态机在 Dead / Resurrection / Working 三态间迁移使用。
 *   2. 手动控制源枚举：遥控器 / 键鼠（如有），说明当前控制指令来源。
 *   3. 底盘工作模式枚举：分离 / 随动 / 小陀螺；云台工作模式枚举（关节空间控 yaw、pitch）。
 *   4. 底盘运动状态结构（平移速度 vx、vy + 旋转角速度 w，单位自定并全工程一致）与
 *      云台运动状态结构（yaw、pitch），并给出加减、数乘等常用运算。
 *
 * 约束 / 提示：
 *   - 这些类型会被 RobotModules 状态机、RobotComponents 双板通讯帧、Instance 接线
 *     三处共同使用，字段命名 / 枚举值必须三处一致——动手前先画一张字段对照表。
 *   - 只需覆盖本骨架用到的状态（裁判系统 / 摩擦轮 / 超电相关状态本骨架用不到，可省略）
 * ==========================================================================*/
namespace robot {

// 显式固定枚举宽度和值，供两板状态机与后续协议共用；数值本身不代表动作许可。
enum class LifeState : std::uint8_t {
  Dead = 0,          // 上电未就绪、急停或故障时的安全生命状态。
  Resurrection = 1,  // 恢复自检阶段，尚不具备 Working 的控制许可。
  Working = 2,       // 正常工作阶段；仍必须同时满足遥控、档位与设备在线等条件。
};

enum class ControlSource : std::uint8_t {
  None = 0,           // 没有有效控制源，默认不得产生动作。
  RemoteControl = 1,  // 本次仅实现 DT7；其余数值保留，不能当成合法控制源。
};

enum class ChassisMode : std::uint8_t {
  Disabled = 0,  // 无有效模式，作为上电和复位的默认值。
  Separate = 1,  // 分离：vx/vy/w 来自遥控映射，不自动回正或自旋。
  Follow = 2,    // 跟随：vx/vy 来自遥控，w 由底盘朝向跟随云台的闭环产生。
  Spin = 3,      // 小陀螺：有效档位内受限自旋，平移方向仍以云台为基准。
};

enum class GimbalMode : std::uint8_t {
  Disabled = 0,       // 无有效模式，默认不生成关节动作。
  JointPosition = 1,  // yaw/pitch 关节空间位置控制，实际闭环在第七项实现。
};

// 两板共享的状态描述；这里只储存状态，不执行状态迁移、不解码遥控器也不控制硬件。
struct ModuleState {
  LifeState life = LifeState::Dead;
  ControlSource source = ControlSource::None;
  ChassisMode chassis_mode = ChassisMode::Disabled;
  GimbalMode gimbal_mode = GimbalMode::Disabled;
};

// 整车运动指令：vx/vy 始终在云台坐标系，x前、y左，单位 m/s。
// w 为车体绕竖直轴的角速度，右手系逆时针为正，单位 rad/s。
// 第六项必须先把平移指令转到车体系，不能把本结构直接当轮速或电机输入。
struct ChassisMotion {
  float vx = 0.0f;
  float vy = 0.0f;
  float w = 0.0f;

  // 将另一份同坐标系、同单位的运动量逐分量累加到自身，返回自身引用。
  constexpr ChassisMotion &operator+=(const ChassisMotion &other) {
    vx += other.vx; vy += other.vy; w += other.w;
    return *this;
  }
  // 从自身逐分量减去另一份同坐标系运动量，返回自身引用。
  constexpr ChassisMotion &operator-=(const ChassisMotion &other) {
    vx -= other.vx; vy -= other.vy; w -= other.w;
    return *this;
  }
  // 用无量纲系数缩放三个分量，返回自身引用；本运算不实施物理限速。
  constexpr ChassisMotion &operator*=(float scale) {
    vx *= scale; vy *= scale; w *= scale;
    return *this;
  }
};

// 云台关节量：yaw/pitch 单位均为 rad，正方向由关节方向标定与右手约定统一。
// 可以描述目标、反馈或角度增量，但调用处必须明确语义；不在纯类型中执行回绕或机械限位。
struct GimbalMotion {
  float yaw = 0.0f;
  float pitch = 0.0f;

  // 对两个关节量逐分量相加并写回自身，返回自身引用。
  constexpr GimbalMotion &operator+=(const GimbalMotion &other) {
    yaw += other.yaw; pitch += other.pitch;
    return *this;
  }
  // 对两个关节量逐分量相减并写回自身；角度最短差由后续控制层计算。
  constexpr GimbalMotion &operator-=(const GimbalMotion &other) {
    yaw -= other.yaw; pitch -= other.pitch;
    return *this;
  }
  // 用无量纲系数缩放两轴角度，返回自身引用；不执行 pitch 限位。
  constexpr GimbalMotion &operator*=(float scale) {
    yaw *= scale; pitch *= scale;
    return *this;
  }
};

// 返回两份同坐标系底盘运动量之和，不修改输入对象。
constexpr ChassisMotion operator+(ChassisMotion left, const ChassisMotion &right) {
  return left += right;
}
// 返回两份同坐标系底盘运动量之差，不修改输入对象。
constexpr ChassisMotion operator-(ChassisMotion left, const ChassisMotion &right) {
  return left -= right;
}
// 返回底盘运动量的相反数，不修改输入对象。
constexpr ChassisMotion operator-(ChassisMotion value) { return value *= -1.0f; }
// 返回底盘运动量乘以无量纲系数的结果，不修改输入对象。
constexpr ChassisMotion operator*(ChassisMotion value, float scale) { return value *= scale; }
// 支持系数在左侧的底盘运动量数乘，与右侧数乘含义一致。
constexpr ChassisMotion operator*(float scale, ChassisMotion value) { return value *= scale; }

// 返回两份云台关节量之和，不修改输入对象。
constexpr GimbalMotion operator+(GimbalMotion left, const GimbalMotion &right) {
  return left += right;
}
// 返回两份云台关节量之差，不修改输入对象；不自动归一化角度。
constexpr GimbalMotion operator-(GimbalMotion left, const GimbalMotion &right) {
  return left -= right;
}
// 返回两轴角度的相反数，不修改输入对象。
constexpr GimbalMotion operator-(GimbalMotion value) { return value *= -1.0f; }
// 返回两轴角度乘以无量纲系数的结果，不修改输入对象。
constexpr GimbalMotion operator*(GimbalMotion value, float scale) { return value *= scale; }
// 支持系数在左侧的云台关节量数乘，不修改输入对象。
constexpr GimbalMotion operator*(float scale, GimbalMotion value) { return value *= scale; }

// 检查生命状态是否为已定义值，拒绝错误解码或显式强转产生的未知值。
constexpr bool isValid(LifeState value) {
  return value == LifeState::Dead || value == LifeState::Resurrection || value == LifeState::Working;
}
// 检查控制源是否在本次实现范围内；预留数值一律视为无效。
constexpr bool isValid(ControlSource value) {
  return value == ControlSource::None || value == ControlSource::RemoteControl;
}
// 检查底盘模式是否已定义；Disabled 是合法的安全描述，不表示允许运动。
constexpr bool isValid(ChassisMode value) {
  return value == ChassisMode::Disabled || value == ChassisMode::Separate ||
      value == ChassisMode::Follow || value == ChassisMode::Spin;
}
// 检查云台模式是否已定义，不判断遥控数据、生命状态或设备在线情况。
constexpr bool isValid(GimbalMode value) {
  return value == GimbalMode::Disabled || value == GimbalMode::JointPosition;
}
// 检查状态描述中的枚举是否合法；合法性不等于完整的安全动作许可。
constexpr bool isValid(const ModuleState &value) {
  return isValid(value.life) && isValid(value.source) &&
      isValid(value.chassis_mode) && isValid(value.gimbal_mode);
}
// 检查三个运动分量是否均为有限数；运算溢出、NaN 和 Inf 均返回 false。
inline bool isFinite(const ChassisMotion &value) {
  return std::isfinite(value.vx) && std::isfinite(value.vy) && std::isfinite(value.w);
}
// 检查两轴角度是否均为有限数；不在此函数内清零或掩盖错误输入。
inline bool isFinite(const GimbalMotion &value) {
  return std::isfinite(value.yaw) && std::isfinite(value.pitch);
}

}  // namespace robot
#endif /* MODULE_STATE_HPP_ */
