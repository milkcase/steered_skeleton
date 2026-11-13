#pragma once
// 测试 CAN 初始化只记录成功，不访问硬件。
bool StartBoardCommunication();
// 测试单页发送仍调用真实协议 encode，帧写入 HAL 记录桩。
bool SendBoardCommunication();
