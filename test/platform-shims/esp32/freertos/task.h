#pragma once
#include "freertos/FreeRTOS.h"
typedef void (*TaskFunction_t)(void *);
int xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack,
                            void *arg, unsigned prio, TaskHandle_t *out, int core);
