#pragma once
#include <cstdint>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum HAL_UART_RxEventTypeTypeDef { HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_IDLE };
struct DMA_HandleTypeDef {};
struct UART_HandleTypeDef { DMA_HandleTypeDef *hdmarx; uint32_t ErrorCode; HAL_UART_RxEventTypeTypeDef event; };
constexpr uint32_t HAL_UART_ERROR_NONE = 0, DMA_IT_HT = 1;
extern uint32_t test_mask;
// 返回模拟中断屏蔽位，供临界区保存原状态。
inline uint32_t __get_PRIMASK() { return test_mask; }
// 设置模拟屏蔽位；不会操作宿主电脑的中断。
inline void __disable_irq() { test_mask = 1; }
// 恢复模拟屏蔽位，用于检查嵌套临界区退出后状态是否正确。
inline void __set_PRIMASK(uint32_t x) { test_mask = x; }
// 半传输通知关闭操作的空桩；模拟器中不需要配置 DMA 寄存器。
inline void __HAL_DMA_DISABLE_IT(DMA_HandleTypeDef *, uint32_t) {}
// 模拟清除 UART 错误字段，供错误恢复测试观察。
inline void __HAL_UART_CLEAR_PEFLAG(UART_HandleTypeDef *u) { u->ErrorCode = 0; }
// 声明可控的测试时钟，具体实现位于测试源文件。
uint32_t HAL_GetTick();
// 声明模拟 DMA 停止接口，允许测试注入超时或成功结果。
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
// 声明模拟 DMA 启动接口，允许测试记录缓冲区并注入启动失败。
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
// 读取预设的模拟接收事件类型，供真实接收适配器区分半传输、满容量与空闲。
inline HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *u) { return u->event; }
