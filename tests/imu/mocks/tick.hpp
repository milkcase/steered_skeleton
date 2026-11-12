#pragma once
#include <cstdint>
namespace hello_world::tick {
// 返回测试可控的微秒时钟，接口与生产代码一致。
uint64_t GetTickUs();
}
