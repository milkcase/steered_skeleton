// 提供的 tick.cpp 只有选择后端才生成实现；当前工程未定义后端，直接链接会缺少时间函数。
// 仅在未选择后端时编译已有 HAL 实现，不改 tick.cpp/CMake，不复制或改写时间算法。
// 若以后全工程定义 USE_HAL_TICK，本适配文件为空，由原 tick.cpp 正常提供唯一实现。
#if !defined(USE_HAL_TICK) && !defined(USE_UNIX_TICK)
#define USE_HAL_TICK
#include "tick.cpp"
#endif
