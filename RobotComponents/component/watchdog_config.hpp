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
