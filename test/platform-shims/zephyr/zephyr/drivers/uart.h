#pragma once
#include <stdint.h>
#include <zephyr/device.h>
typedef void (*uart_irq_callback_user_data_t)(const struct device *dev, void *user);
int  uart_irq_update(const struct device *dev);
int  uart_irq_rx_ready(const struct device *dev);
int  uart_fifo_read(const struct device *dev, uint8_t *out, int len);
void uart_irq_callback_user_data_set(const struct device *dev,
                                     uart_irq_callback_user_data_t cb, void *user);
void uart_irq_rx_enable(const struct device *dev);
void uart_poll_out(const struct device *dev, uint8_t byte);
