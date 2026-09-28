/*
 * Zephyr, UART with the interrupt driven API.
 *
 * Bytes arrive in an ISR and are put in a ring buffer; the SDK is fed from a thread.
 * Calling into the SDK from the ISR would work today and break the first time a callback
 * does anything real, so it is worth keeping the two apart from the start.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include "ardudeck.h"

static const struct device *uart;
RING_BUF_DECLARE(rx_ring, 512);

static void isr(const struct device *dev, void *user) {
  (void)user;
  uint8_t byte;
  while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
    if (uart_fifo_read(dev, &byte, 1) == 1) {
      /* A full ring drops bytes, which is correct: the SDK resynchronises on the next
         frame, and blocking in an ISR would be worse. */
      ring_buf_put(&rx_ring, &byte, 1);
    }
  }
}

static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  for (size_t i = 0; i < len; i++) uart_poll_out(uart, buf[i]);
}

static uint32_t now_ms(void) { return k_uptime_get_32(); }

void ardudeck_thread(void *a, void *b, void *c) {
  (void)b; (void)c;
  ad_config_t *cfg = (ad_config_t *)a;

  uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
  uart_irq_callback_user_data_set(uart, isr, NULL);
  uart_irq_rx_enable(uart);

  cfg->send = sink;
  cfg->now_ms = now_ms;
  ardudeck_begin(cfg);

  for (;;) {
    uint8_t buf[128];
    uint32_t n;
    while ((n = ring_buf_get(&rx_ring, buf, sizeof buf)) > 0) {
      ardudeck_receive(buf, (size_t)n);
    }

    /* Feed your state here. */

    ardudeck_tick(now_ms());
    k_msleep(20);
  }
}
