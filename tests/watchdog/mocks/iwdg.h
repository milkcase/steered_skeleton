#pragma once
#include "platform.hpp"
struct IWDG_HandleTypeDef {};
extern IWDG_HandleTypeDef hiwdg;
// 捕获刷新时机和注入 HAL 刷新失败，不模拟物理 IWDG 计数或复位。
extern "C" HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *);
