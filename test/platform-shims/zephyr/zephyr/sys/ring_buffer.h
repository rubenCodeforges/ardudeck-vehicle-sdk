#pragma once
#include <stdint.h>
struct ring_buf { int unused; };
#define RING_BUF_DECLARE(name, size) static struct ring_buf name
uint32_t ring_buf_put(struct ring_buf *rb, const uint8_t *data, uint32_t len);
uint32_t ring_buf_get(struct ring_buf *rb, uint8_t *out, uint32_t len);
