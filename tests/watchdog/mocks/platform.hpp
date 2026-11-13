#pragma once
#include <cstdint>
struct DWT_Type {uint32_t CYCCNT=0,CTRL=1;};
struct RCC_Type {uint32_t CSR=0;};
extern DWT_Type test_dwt;
extern RCC_Type test_rcc;
extern uint32_t SystemCoreClock,test_ms,irq_mask,aborts,stage,uart_inits;
#define DWT (&test_dwt)
#define RCC (&test_rcc)
constexpr uint32_t DWT_CTRL_CYCCNTENA_Msk=1,RCC_FLAG_IWDGRST=0x20000000,RESET=0;
#define __HAL_RCC_GET_FLAG(flag) ((test_rcc.CSR & (flag))!=0)
#define __HAL_RCC_CLEAR_RESET_FLAGS() (test_rcc.CSR=0)
enum HAL_StatusTypeDef {HAL_OK,HAL_ERROR};
// 返回测试毫秒数，不使用真实 SysTick。
extern "C" uint32_t HAL_GetTick();
