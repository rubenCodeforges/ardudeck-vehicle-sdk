/*
 * Linux, POSIX sockets. A companion computer, or your desktop against SITL.
 *
 * This is the one to start with: it builds and runs on the machine you are reading
 * this on, so you can point ArduDeck at your firmware logic before any hardware exists.
 *
 *   cc -o vehicle linux_udp.c your_vehicle.c ../../src/ad_*.c ../../src/ardudeck.c -I../../include
 *   ./vehicle                       then connect ArduDeck to UDP 14550
 */

#include <arpa/inet.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "ardudeck.h"

static int      sock = -1;
static struct sockaddr_in peer;
static bool     have_peer = false;

/*
 * Answer whoever spoke to us, rather than broadcasting at a fixed address. A ground
 * station may be on any address on the network, and this way nothing needs configuring
 * at either end.
 */
static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  if (!have_peer) return;
  sendto(sock, buf, len, 0, (struct sockaddr *)&peer, sizeof peer);
}

static uint32_t now_ms(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

int transport_open(uint16_t port) {
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) return -1;

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof addr);
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
  if (bind(sock, (struct sockaddr *)&addr, sizeof addr) < 0) return -1;

  /* A short timeout rather than none, so the loop below keeps ticking on a quiet link. */
  struct timeval tv = {0, 20000};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  return 0;
}

/* Call this as fast as you like. It blocks for at most 20 ms. */
void transport_poll(void) {
  uint8_t buf[512];
  struct sockaddr_in from;
  socklen_t from_len = sizeof from;
  ssize_t n = recvfrom(sock, buf, sizeof buf, 0, (struct sockaddr *)&from, &from_len);
  if (n > 0) {
    peer = from;
    have_peer = true;
    ardudeck_receive(buf, (size_t)n);
  }
}

void transport_wire(ad_config_t *cfg) {
  cfg->send = sink;
  cfg->now_ms = now_ms;
}
