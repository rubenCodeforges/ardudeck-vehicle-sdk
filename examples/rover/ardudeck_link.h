#ifndef ARDUDECK_LINK_H
#define ARDUDECK_LINK_H

#include <stddef.h>
#include <stdint.h>

/** The three calls the firmware's main loop needs. Everything else is inside. */
void ardudeck_link_begin(void);
void ardudeck_link_tick(void);
void ardudeck_link_receive(const uint8_t *buf, size_t len);

/** Provided by the transport, so the integration does not care what the link is. */
void transport_send(const uint8_t *buf, size_t len);

#endif
