#pragma once
#include <stdint.h>
typedef struct { uint32_t Instance; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *data, uint16_t len, uint32_t to);
int HAL_UART_Receive_DMA(UART_HandleTypeDef *h, uint8_t *data, uint16_t len);
uint32_t hal_dma_counter(DMA_HandleTypeDef *h);
#define __HAL_DMA_GET_COUNTER(h) hal_dma_counter(h)
