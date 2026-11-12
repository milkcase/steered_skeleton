#pragma once
#include <cstdint>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum HAL_UART_RxEventTypeTypeDef { HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_IDLE };
struct DMA_HandleTypeDef {};
struct UART_HandleTypeDef { DMA_HandleTypeDef *hdmarx; uint32_t ErrorCode; HAL_UART_RxEventTypeTypeDef event; };
constexpr uint32_t HAL_UART_ERROR_NONE = 0, DMA_IT_HT = 1;
extern uint32_t test_mask;
inline uint32_t __get_PRIMASK() { return test_mask; }
inline void __disable_irq() { test_mask = 1; }
inline void __set_PRIMASK(uint32_t x) { test_mask = x; }
inline void __HAL_DMA_DISABLE_IT(DMA_HandleTypeDef *, uint32_t) {}
inline void __HAL_UART_CLEAR_PEFLAG(UART_HandleTypeDef *u) { u->ErrorCode = 0; }
uint32_t HAL_GetTick();
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
inline HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *u) { return u->event; }
