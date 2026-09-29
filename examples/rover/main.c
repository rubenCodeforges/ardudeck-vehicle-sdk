/*
 * The main loop, and a UDP socket. Run it and connect ArduDeck to UDP 14550.
 *
 * The two halves are visible here: rover_step() is the firmware doing its job, and the
 * three ardudeck_link_* calls are the entire integration.
 */
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "ardudeck_link.h"
#include "vehicle.h"

static int                sock = -1;
static struct sockaddr_in peer;
static bool               have_peer = false;
static int                gcs_port = 14550;

/*
 * Answer whoever spoke to us. Until somebody does, send to the ground station's port on
 * loopback, so a tool that only listens still sees the vehicle appear.
 */
void transport_send(const uint8_t *buf, size_t len) {
  struct sockaddr_in to;
  if (have_peer) {
    to = peer;
  } else {
    memset(&to, 0, sizeof to);
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    to.sin_port = htons((uint16_t)gcs_port);
  }
  sendto(sock, buf, len, 0, (struct sockaddr *)&to, sizeof to);
}

int main(int argc, char **argv) {
  int port = 14550;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
  }

  gcs_port = port;
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  int on = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);

  /* The ground station owns `port`, so the vehicle listens one above it. */
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof addr);
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons((uint16_t)(port + 1));
  if (bind(sock, (struct sockaddr *)&addr, sizeof addr) < 0) {
    fprintf(stderr, "cannot bind %d\n", port);
    return 1;
  }
  struct timeval tv = {0, 20000};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

  rover_init();
  ardudeck_link_begin();
  printf("rover running. Connect ArduDeck to UDP %d, or run:\n"
         "  ardudeck-conform --udp %d\n", port, port);

  for (;;) {
    uint8_t buf[512];
    struct sockaddr_in from;
    socklen_t from_len = sizeof from;
    ssize_t n = recvfrom(sock, buf, sizeof buf, 0, (struct sockaddr *)&from, &from_len);
    if (n > 0) {
      peer = from;
      have_peer = true;
      ardudeck_link_receive(buf, (size_t)n);
    }

    rover_step(0.02f);        /* the firmware, doing what it always did */
    ardudeck_link_tick();     /* the integration, telling ArduDeck about it */
  }
}
