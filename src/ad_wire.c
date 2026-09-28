#include <string.h>

#include "ad_internal.h"

uint16_t ad_crc16(const uint8_t *data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; i++) {
    uint8_t tmp = (uint8_t)(data[i] ^ (uint8_t)(crc & 0xFF));
    tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
    crc = (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^
                     ((uint16_t)tmp >> 4));
  }
  return crc;
}

void ad_tx_begin(void) { ad_g.tx.at = 0; }

static void put_bytes(const uint8_t *src, uint8_t n) {
  if ((uint16_t)ad_g.tx.at + n > AD_MAX_PAYLOAD) return;
  memcpy(&ad_g.tx.payload[ad_g.tx.at], src, n);
  ad_g.tx.at = (uint8_t)(ad_g.tx.at + n);
}

void ad_put_u8(uint8_t v) { put_bytes(&v, 1); }

void ad_put_u16(uint16_t v) {
  uint8_t b[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
  put_bytes(b, 2);
}

void ad_put_u32(uint32_t v) {
  uint8_t b[4] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8), (uint8_t)(v >> 16),
                  (uint8_t)(v >> 24)};
  put_bytes(b, 4);
}

void ad_put_i16(int16_t v) { ad_put_u16((uint16_t)v); }
void ad_put_i32(int32_t v) { ad_put_u32((uint32_t)v); }

void ad_put_u64(uint64_t v) {
  ad_put_u32((uint32_t)(v & 0xFFFFFFFFu));
  ad_put_u32((uint32_t)(v >> 32));
}

void ad_put_f32(float v) {
  uint32_t bits;
  memcpy(&bits, &v, 4);
  ad_put_u32(bits);
}

/**
 * Fixed-width text, NUL padded.
 *
 * MAVLink char fields are not NUL terminated when the text exactly fills them, so a
 * receiver reads at most `width` bytes. Anything longer is truncated here rather than
 * running into the next field.
 */
void ad_put_str(const char *s, uint8_t width) {
  uint8_t i = 0;
  if (s) {
    for (; i < width && s[i] != '\0'; i++) ad_put_u8((uint8_t)s[i]);
  }
  for (; i < width; i++) ad_put_u8(0);
}

void ad_put_pad(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) ad_put_u8(0);
}

void ad_tx_send(ad_msg_def_t def) {
  if (!ad_g.cfg || !ad_g.cfg->send) return;

  /* Anything the caller left unwritten is zero, which is what a receiver assumes. */
  while (ad_g.tx.at < def.length) ad_put_u8(0);

  /*
   * v2 drops trailing zero bytes and the checksum covers only what is left. The mirror
   * of this rule matters more: a decoder must zero-pad a short payload back to full
   * length before reading fields, never reject it for being short.
   */
  uint8_t len = def.length;
  while (len > 1 && ad_g.tx.payload[len - 1] == 0) len--;

  uint8_t *f = ad_g.tx.buf;
  f[0] = AD_STX_V2;
  f[1] = len;
  f[2] = 0; /* incompat flags: unsigned */
  f[3] = 0; /* compat flags */
  f[4] = ad_g.seq++;
  f[5] = ad_g.sysid;
  f[6] = ad_g.compid;
  f[7] = (uint8_t)(def.id & 0xFF);
  f[8] = (uint8_t)((def.id >> 8) & 0xFF);
  f[9] = (uint8_t)((def.id >> 16) & 0xFF);
  memcpy(&f[10], ad_g.tx.payload, len);

  uint16_t crc = ad_crc16(&f[1], (size_t)(9 + len), 0xFFFF);
  crc = ad_crc16(&def.crc_extra, 1, crc);
  f[10 + len] = (uint8_t)(crc & 0xFF);
  f[11 + len] = (uint8_t)(crc >> 8);

  ad_g.cfg->send(f, (size_t)(12 + len), ad_g.cfg->user);
  ad_g.tx.at = 0;
}

/* Only inbound messages need an extra. Anything else on the link is not ours to check. */
static const ad_msg_def_t INBOUND[] = {
  AD_MSG_PARAM_REQUEST_READ, AD_MSG_PARAM_REQUEST_LIST, AD_MSG_PARAM_SET,
  AD_MSG_COMMAND_LONG, AD_MSG_MISSION_COUNT, AD_MSG_MISSION_ITEM_INT,
  AD_MSG_MISSION_REQUEST_INT, AD_MSG_MISSION_REQUEST_LIST, AD_MSG_MISSION_ACK,
  AD_MSG_MISSION_CLEAR_ALL, AD_MSG_CAL_CONTROL, AD_MSG_REQUEST, AD_MSG_SET_MODE,
};

bool ad_known_extra(uint32_t msgid, uint8_t *extra) {
  for (size_t i = 0; i < sizeof INBOUND / sizeof INBOUND[0]; i++) {
    if (INBOUND[i].id == msgid) {
      *extra = INBOUND[i].crc_extra;
      return true;
    }
  }
  return false;
}

/*
 * Readers zero-pad, because a v2 sender truncates trailing zeros and a field that lands
 * past the end of a short payload is genuinely zero rather than missing.
 */

uint8_t ad_get_u8(const uint8_t *p, uint16_t len, uint16_t off) {
  return off < len ? p[off] : 0;
}

uint16_t ad_get_u16(const uint8_t *p, uint16_t len, uint16_t off) {
  return (uint16_t)(ad_get_u8(p, len, off) | ((uint16_t)ad_get_u8(p, len, off + 1) << 8));
}

uint32_t ad_get_u32(const uint8_t *p, uint16_t len, uint16_t off) {
  return (uint32_t)ad_get_u16(p, len, off) | ((uint32_t)ad_get_u16(p, len, off + 2) << 16);
}

int32_t ad_get_i32(const uint8_t *p, uint16_t len, uint16_t off) {
  return (int32_t)ad_get_u32(p, len, off);
}

float ad_get_f32(const uint8_t *p, uint16_t len, uint16_t off) {
  uint32_t bits = ad_get_u32(p, len, off);
  float v;
  memcpy(&v, &bits, 4);
  return v;
}

void ad_get_str(const uint8_t *p, uint16_t len, uint16_t off, uint8_t width, char *out,
                size_t out_len) {
  size_t n = width < out_len - 1 ? width : out_len - 1;
  size_t i = 0;
  for (; i < n; i++) {
    uint8_t c = ad_get_u8(p, len, (uint16_t)(off + i));
    if (c == 0) break;
    out[i] = (char)c;
  }
  out[i] = '\0';
}
