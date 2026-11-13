// 每板仅编译一次公共实现，板级辅助函数决定本地电机和总线，不跨固件共享运行数据。
#include "watchdog_board.hpp"
#include "watchdog_task_impl.hpp"
