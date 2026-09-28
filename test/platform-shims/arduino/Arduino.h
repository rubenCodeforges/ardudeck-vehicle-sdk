#pragma once
#include <stdint.h>
#include <stddef.h>
unsigned long millis(void);
void delay(unsigned long ms);
class HardwareSerial {
 public:
  void begin(long baud);
  int available(void);
  int read(void);
  size_t write(const uint8_t *buf, size_t len);
};
extern HardwareSerial Serial1;
