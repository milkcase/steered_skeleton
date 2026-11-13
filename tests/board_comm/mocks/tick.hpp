#pragma once
#include <cstdint>
namespace hello_world::tick {
// 测试可控的毫秒时钟，供真实协议代码检查拼包和离线超时。
uint32_t GetTickMs();
}
