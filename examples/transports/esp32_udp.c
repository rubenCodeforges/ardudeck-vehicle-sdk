/*
 * ESP32, ESP-IDF. UDP on the access point the board already runs.
 *
 * The common shape: the board is its own AP, a phone or laptop joins it, and nothing
 * needs an address typed in because the vehicle answers whoever spoke to it.
 *
 * Add `ardudeck` to REQUIRES in your component's CMakeLists.txt, or this file will not
 * find the header.
 */

#include <string.h>

#include "ardudeck.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static int                sock = -1;
static struct sockaddr_in peer;
static bool               have_peer = false;

static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  if (!have_peer || sock < 0) return;
  sendto(sock, buf, len, 0, (struct sockaddr *)&peer, sizeof peer);
}

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/*
 * One task owns every SDK call, which is how you satisfy "not thread safe" without a
 * mutex. If your firmware already serves a web app, consider driving the SDK from that
 * task instead: the settings your callbacks write are the settings that app writes.
 */
static void ardudeck_task(void *arg) {
  const ad_config_t *cfg = (const ad_config_t *)arg;

  sock = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof addr);
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(14550);
  bind(sock, (struct sockaddr *)&addr, sizeof addr);

  /* 20 ms rather than blocking, so the tick keeps running when nobody is talking. */
  struct timeval tv = {0, 20000};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

  ardudeck_begin(cfg);

  uint8_t buf[512];
  for (;;) {
    struct sockaddr_in from;
    socklen_t from_len = sizeof from;
    int n = recvfrom(sock, buf, sizeof buf, 0, (struct sockaddr *)&from, &from_len);
    if (n > 0) {
      peer = from;
      have_peer = true;
      ardudeck_receive(buf, (size_t)n);
    }

    /* Feed your state here: position, attitude, status, battery. */

    ardudeck_tick(now_ms());
  }
}

void ardudeck_start(ad_config_t *cfg) {
  cfg->send = sink;
  cfg->now_ms = now_ms;
  /* 6 KB is comfortable with missions and parameters; 4 KB is enough for telemetry. */
  xTaskCreatePinnedToCore(ardudeck_task, "ardudeck", 6144, cfg, 4, NULL, 0);
}
