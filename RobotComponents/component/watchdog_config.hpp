/*
 * @Author: milkcase choose68@163.com
 * @Date: 2026-11-13 10:43:57
 * @LastEditors: milkcase choose68@163.com
 * @LastEditTime: 2026-11-13 15:17:49
 * @FilePath: \Chassisd:\RM\steered_project\steered_skeleton\RobotComponents\component\watchdog_config.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef STEERED_WATCHDOG_CONFIG_HPP_
#define STEERED_WATCHDOG_CONFIG_HPP_
#include <cstdint>
// 正常固件必须保持 1；仅在电机动力电源断开时，用 0 构建“停止喂狗”验收固件。
// 停喂狗配置同时锁住本板输出，不通过持续运动或危险死循环演示复位。
#ifndef ROBOT_WATCHDOG_REFRESH_ENABLED
#define ROBOT_WATCHDOG_REFRESH_ENABLED 1
#endif
namespace robot_watchdog {
inline constexpr bool kRefreshEnabled=ROBOT_WATCHDOG_REFRESH_ENABLED!=0;
inline constexpr uint32_t kFeedIntervalMs=20;
inline constexpr uint32_t kForegroundTimeoutMs=100;
inline constexpr uint32_t kPhaseBudgetMs=3;
}
#endif
