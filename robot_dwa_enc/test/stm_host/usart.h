/* host stubs of the HAL used by WheelOdometry (plain C, like the real usart.h) */
#pragma once
#include <stdint.h>
#include <string.h>
typedef struct { char tx[1024]; int n; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;
extern uint32_t g_tick;
static inline uint32_t HAL_GetTick(void) { return g_tick; }
static inline int HAL_UART_Transmit(UART_HandleTypeDef* h, const uint8_t* p, int n, int t)
{ (void)t; memcpy(h->tx + h->n, p, n); h->n += n; h->tx[h->n] = 0; return 0; }
