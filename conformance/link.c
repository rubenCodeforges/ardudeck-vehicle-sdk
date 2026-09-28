#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include "conform.h"

struct link {
  int  fd;
  bool is_udp;
  struct sockaddr_in peer;
  bool have_peer;
};

uint32_t now_ms(void) {
  static struct timeval start;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (start.tv_sec == 0) start = tv;
  return (uint32_t)((tv.tv_sec - start.tv_sec) * 1000 +
                    (tv.tv_usec - start.tv_usec) / 1000);
}

link_t *link_open_udp(const char *spec, char *err, size_t err_len) {
  char host[64] = "";
  int port = 14550;

  const char *colon = strrchr(spec, ':');
  if (colon) {
    size_t n = (size_t)(colon - spec);
    if (n >= sizeof host) n = sizeof host - 1;
    memcpy(host, spec, n);
    host[n] = '\0';
    port = atoi(colon + 1);
  } else {
    port = atoi(spec);
  }
  if (port <= 0 || port > 65535) {
    snprintf(err, err_len, "bad port in '%s'", spec);
    return NULL;
  }

  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    snprintf(err, err_len, "socket: %s", strerror(errno));
    return NULL;
  }

  int on = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
  setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof on);

  struct sockaddr_in bind_addr;
  memset(&bind_addr, 0, sizeof bind_addr);
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_addr.sin_port = htons((uint16_t)port);
  if (bind(fd, (struct sockaddr *)&bind_addr, sizeof bind_addr) < 0) {
    snprintf(err, err_len, "bind %d: %s", port, strerror(errno));
    close(fd);
    return NULL;
  }

  link_t *l = calloc(1, sizeof *l);
  if (!l) {
    snprintf(err, err_len, "out of memory");
    close(fd);
    return NULL;
  }
  l->fd = fd;
  l->is_udp = true;

  /*
   * A host was named, so answer there straight away. Without one the peer is learned
   * from the first packet, which is what makes a broadcasting vehicle work with nothing
   * configured at either end.
   */
  if (host[0]) {
    l->peer.sin_family = AF_INET;
    l->peer.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &l->peer.sin_addr) == 1) l->have_peer = true;
  }
  return l;
}

static speed_t baud_constant(int baud) {
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:     return 0;
  }
}

link_t *link_open_serial(const char *spec, char *err, size_t err_len) {
  char dev[256];
  int baud = 57600;

  const char *colon = strrchr(spec, ':');
  if (colon) {
    size_t n = (size_t)(colon - spec);
    if (n >= sizeof dev) n = sizeof dev - 1;
    memcpy(dev, spec, n);
    dev[n] = '\0';
    baud = atoi(colon + 1);
  } else {
    snprintf(dev, sizeof dev, "%s", spec);
  }

  speed_t speed = baud_constant(baud);
  if (!speed) {
    snprintf(err, err_len, "unsupported baud %d", baud);
    return NULL;
  }

  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    snprintf(err, err_len, "open %s: %s", dev, strerror(errno));
    return NULL;
  }

  struct termios tio;
  if (tcgetattr(fd, &tio) != 0) {
    snprintf(err, err_len, "tcgetattr %s: %s", dev, strerror(errno));
    close(fd);
    return NULL;
  }
  cfmakeraw(&tio);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= (tcflag_t)~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    snprintf(err, err_len, "tcsetattr %s: %s", dev, strerror(errno));
    close(fd);
    return NULL;
  }

  link_t *l = calloc(1, sizeof *l);
  if (!l) {
    snprintf(err, err_len, "out of memory");
    close(fd);
    return NULL;
  }
  l->fd = fd;
  l->is_udp = false;
  return l;
}

bool link_send(link_t *l, const uint8_t *buf, size_t len) {
  if (!l) return false;
  if (l->is_udp) {
    if (!l->have_peer) return false;
    return sendto(l->fd, buf, len, 0, (struct sockaddr *)&l->peer, sizeof l->peer) ==
           (ssize_t)len;
  }
  return write(l->fd, buf, len) == (ssize_t)len;
}

int link_recv(link_t *l, uint8_t *buf, size_t cap, int timeout_ms) {
  if (!l) return -1;

  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(l->fd, &rd);
  struct timeval tv = {timeout_ms / 1000, (timeout_ms % 1000) * 1000};

  int ready = select(l->fd + 1, &rd, NULL, NULL, &tv);
  if (ready <= 0) return ready == 0 ? 0 : -1;

  if (l->is_udp) {
    struct sockaddr_in from;
    socklen_t from_len = sizeof from;
    ssize_t n = recvfrom(l->fd, buf, cap, 0, (struct sockaddr *)&from, &from_len);
    if (n <= 0) return (int)n;
    /* Answer whoever is talking, so a broadcasting vehicle needs no configuration. */
    if (!l->have_peer) {
      l->peer = from;
      l->have_peer = true;
    }
    return (int)n;
  }

  ssize_t n = read(l->fd, buf, cap);
  if (n < 0) return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
  return (int)n;
}

void link_close(link_t *l) {
  if (!l) return;
  close(l->fd);
  free(l);
}
