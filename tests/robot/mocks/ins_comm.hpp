#pragma once
#include "DT7.hpp"
// 测试提供合法 DT7 快照，可注入断遥控；生产版本仍使用第二项有效期检查。
bool ReadRemoteControl(hello_world::remote_control::DT7 &snapshot);
