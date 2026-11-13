#pragma once
#include "dbus_uart.hpp"
// 为真实 CommTask 提供测试接收对象。
remote_input::Receiver *GetRemoteControlReceiver();
