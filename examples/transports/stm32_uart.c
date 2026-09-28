/*
 * STM32, HAL. UART with DMA receive into a circular buffer.
 *
 * A flight controller's telemetry port is a UART, and the reason to use DMA here is that
 * the SDK wants whatever has arrived, whenever it arrived. There is no framing to respect
 * at this layer: hand it bytes and it finds the frames.
 */

#include <string.h>

#include "ardudeck.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

#define RX_RING 512

static uint8_t  rx[RX_RING];
static uint16_t rx_read;

/*
 * Transmit blocks. A telemetry frame is at most 280 bytes and a 57600 baud port clears
 * that in 50 ms, which is too long to sit inside a control loop. If yours is tight, queue
 * here and drain from a lower priority task instead.
 */
static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100);
}

static uint32_t now_ms(void) { return HAL_GetTick(); }

void ardudeck_link_begin(ad_config_t *cfg) {
  cfg->send = sink;
  cfg->now_ms = now_ms;
  rx_read = 0;
  HAL_UART_Receive_DMA(&huart2, rx, RX_RING);
  ardudeck_begin(cfg);
}

/*
 * Call this from your main loop, as often as you can. It hands the SDK everything the DMA
 * has put in the ring since last time, in at most two pieces.
 */
void ardudeck_link_poll(void) {
  const uint16_t write = (uint16_t)(RX_RING - __HAL_DMA_GET_COUNTER(huart2.hdmarx));

  if (write != rx_read) {
    if (write > rx_read) {
      ardudeck_receive(&rx[rx_read], (size_t)(write - rx_read));
    } else {
      ardudeck_receive(&rx[rx_read], (size_t)(RX_RING - rx_read));
      if (write > 0) ardudeck_receive(&rx[0], (size_t)write);
    }
    rx_read = write;
  }

  ardudeck_tick(now_ms());
}
