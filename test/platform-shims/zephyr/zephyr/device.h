#pragma once
struct device { int unused; };
extern const struct device *zephyr_shim_device;
#define DT_CHOSEN(x) 0
#define DEVICE_DT_GET(x) (zephyr_shim_device)
