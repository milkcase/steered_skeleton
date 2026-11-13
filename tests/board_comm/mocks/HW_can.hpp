#pragma once
#include "can.h"
extern "C" {
// 声明真实基础 CAN 滤波入口，仅用于将接收源文件纳入测试链接。
void CanFilter_Init(CAN_HandleTypeDef *);
// 声明真实基础 CAN 发送入口，协议测试不会将其用于电机输出。
void CAN_Send_Msg(CAN_HandleTypeDef *,uint8_t *,uint32_t,uint8_t);
}
