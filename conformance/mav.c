#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "ad_msg_defs.h"
#include "conform.h"

static uint16_t crc16(const uint8_t *d, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; i++) {
    uint8_t t = (uint8_t)(d[i] ^ (uint8_t)(crc & 0xFF));
    t = (uint8_t)(t ^ (uint8_t)(t << 4));
    crc = (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^
                     ((uint16_t)t >> 4));
  }
  return crc;
}

/* Both directions, since the tool has to check what it hears and send what it asks. */
static const ad_msg_def_t KNOWN[] = {
  AD_MSG_HEARTBEAT, AD_MSG_SYS_STATUS, AD_MSG_GPS_RAW_INT, AD_MSG_ATTITUDE,
  AD_MSG_GLOBAL_POSITION_INT, AD_MSG_VFR_HUD, AD_MSG_PARAM_VALUE,
  AD_MSG_PARAM_REQUEST_READ, AD_MSG_PARAM_REQUEST_LIST, AD_MSG_PARAM_SET,
  AD_MSG_COMMAND_LONG, AD_MSG_COMMAND_ACK, AD_MSG_MISSION_ITEM_INT,
  AD_MSG_MISSION_REQUEST_INT, AD_MSG_MISSION_REQUEST_LIST, AD_MSG_MISSION_COUNT,
  AD_MSG_MISSION_ACK, AD_MSG_MISSION_CLEAR_ALL, AD_MSG_MISSION_CURRENT,
  AD_MSG_RC_CHANNELS, AD_MSG_STATUSTEXT, AD_MSG_HOME_POSITION,
  AD_MSG_MANIFEST, AD_MSG_MODE, AD_MSG_MISSION_CMDS, AD_MSG_PARAM_META,
  AD_MSG_PARAM_OPTION, AD_MSG_CAL_DECLARE, AD_MSG_CAL_POSE, AD_MSG_CAL_TRACK,
  AD_MSG_CAL_CONTROL, AD_MSG_CAL_PROGRESS, AD_MSG_CAL_RESULT, AD_MSG_REQUEST,
};

bool mav_extra(uint32_t id, uint8_t *extra) {
  for (size_t i = 0; i < sizeof KNOWN / sizeof KNOWN[0]; i++) {
    if (KNOWN[i].id == id) {
      if (extra) *extra = KNOWN[i].crc_extra;
      return true;
    }
  }
  return false;
}

enum { S_STX = 0, S_LEN, S_INCOMPAT, S_COMPAT, S_SEQ, S_SYSID, S_COMPID, S_MSGID,
       S_PAYLOAD, S_CRC1, S_CRC2, S_SIGNATURE };

void mav_reset(mav_parser_t *p) { memset(p, 0, sizeof *p); }

bool mav_feed(mav_parser_t *p, uint8_t b, mav_msg_t *out) {
  switch (p->state) {
    case S_STX:
      if (b != 0xFD && b != 0xFE) return false;
      p->v1 = (b == 0xFE);
      p->crc = 0xFFFF;
      p->incompat = 0;
      p->msgid = 0;
      p->msgid_at = 0;
      p->at = 0;
      p->state = S_LEN;
      return false;

    case S_LEN:
      p->length = b;
      p->crc = crc16(&b, 1, p->crc);
      p->state = p->v1 ? S_SEQ : S_INCOMPAT;
      return false;

    case S_INCOMPAT:
      p->incompat = b;
      p->crc = crc16(&b, 1, p->crc);
      p->state = S_COMPAT;
      return false;

    case S_COMPAT:
      p->crc = crc16(&b, 1, p->crc);
      p->state = S_SEQ;
      return false;

    case S_SEQ:
      p->crc = crc16(&b, 1, p->crc);
      p->state = S_SYSID;
      return false;

    case S_SYSID:
      p->sysid = b;
      p->crc = crc16(&b, 1, p->crc);
      p->state = S_COMPID;
      return false;

    case S_COMPID:
      p->compid = b;
      p->crc = crc16(&b, 1, p->crc);
      p->state = S_MSGID;
      return false;

    case S_MSGID:
      p->crc = crc16(&b, 1, p->crc);
      p->msgid |= (uint32_t)b << (8 * p->msgid_at);
      p->msgid_at++;
      if (p->msgid_at >= (p->v1 ? 1 : 3)) p->state = p->length ? S_PAYLOAD : S_CRC1;
      return false;

    case S_PAYLOAD:
      if (p->at < sizeof p->payload) p->payload[p->at] = b;
      p->at++;
      p->crc = crc16(&b, 1, p->crc);
      if (p->at >= p->length) p->state = S_CRC1;
      return false;

    case S_CRC1:
      p->crc_low = b;
      p->state = S_CRC2;
      return false;

    case S_CRC2: {
      uint16_t got = (uint16_t)(p->crc_low | ((uint16_t)b << 8));
      p->state = (!p->v1 && (p->incompat & 0x01)) ? S_SIGNATURE : S_STX;
      if (p->state == S_SIGNATURE) p->at = 0;

      uint8_t extra;
      if (!mav_extra(p->msgid, &extra)) return false;
      if (crc16(&extra, 1, p->crc) != got) return false;

      out->id = p->msgid;
      out->sysid = p->sysid;
      out->compid = p->compid;
      out->len = p->length;
      memcpy(out->payload, p->payload, p->length);
      return true;
    }

    case S_SIGNATURE:
      if (++p->at >= 13) p->state = S_STX;
      return false;

    default:
      p->state = S_STX;
      return false;
  }
}

size_t mav_build(uint8_t *out, uint32_t id, uint8_t crc_extra, const uint8_t *payload,
                 uint8_t len, uint8_t seq, uint8_t sysid, uint8_t compid) {
  while (len > 1 && payload[len - 1] == 0) len--;
  out[0] = 0xFD;
  out[1] = len;
  out[2] = 0;
  out[3] = 0;
  out[4] = seq;
  out[5] = sysid;
  out[6] = compid;
  out[7] = (uint8_t)(id & 0xFF);
  out[8] = (uint8_t)((id >> 8) & 0xFF);
  out[9] = (uint8_t)((id >> 16) & 0xFF);
  memcpy(&out[10], payload, len);
  uint16_t c = crc16(&out[1], (size_t)(9 + len), 0xFFFF);
  c = crc16(&crc_extra, 1, c);
  out[10 + len] = (uint8_t)(c & 0xFF);
  out[11 + len] = (uint8_t)(c >> 8);
  return (size_t)(12 + len);
}

/* Zero padding is the rule, not a convenience: a v2 sender truncates trailing zeros. */

uint8_t mav_u8(const mav_msg_t *m, uint16_t off) {
  return off < m->len ? m->payload[off] : 0;
}

uint16_t mav_u16(const mav_msg_t *m, uint16_t off) {
  return (uint16_t)(mav_u8(m, off) | ((uint16_t)mav_u8(m, (uint16_t)(off + 1)) << 8));
}

uint32_t mav_u32(const mav_msg_t *m, uint16_t off) {
  return (uint32_t)mav_u16(m, off) | ((uint32_t)mav_u16(m, (uint16_t)(off + 2)) << 16);
}

int32_t mav_i32(const mav_msg_t *m, uint16_t off) { return (int32_t)mav_u32(m, off); }

float mav_f32(const mav_msg_t *m, uint16_t off) {
  uint32_t bits = mav_u32(m, off);
  float v;
  memcpy(&v, &bits, 4);
  return v;
}

void mav_str(const mav_msg_t *m, uint16_t off, uint8_t width, char *out,
             size_t out_len) {
  size_t n = width < out_len - 1 ? width : out_len - 1;
  size_t i = 0;
  for (; i < n; i++) {
    uint8_t c = mav_u8(m, (uint16_t)(off + i));
    if (c == 0) break;
    out[i] = (char)c;
  }
  out[i] = '\0';
}

void mav_put_u8(uint8_t *p, uint8_t v) { p[0] = v; }

void mav_put_u16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

void mav_put_i32(uint8_t *p, int32_t v) {
  uint32_t u = (uint32_t)v;
  p[0] = (uint8_t)(u & 0xFF);
  p[1] = (uint8_t)((u >> 8) & 0xFF);
  p[2] = (uint8_t)((u >> 16) & 0xFF);
  p[3] = (uint8_t)((u >> 24) & 0xFF);
}

void mav_put_f32(uint8_t *p, float v) { memcpy(p, &v, 4); }

/* ─── results ──────────────────────────────────────────────────────────────── */

static void add_detail(rung_t *r, const char *fmt, va_list ap) {
  if (r->detail_count >= MAX_DETAIL) return;
  vsnprintf(r->detail[r->detail_count], sizeof r->detail[0], fmt, ap);
  r->detail_count++;
}

void rung_note(rung_t *r, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  add_detail(r, fmt, ap);
  va_end(ap);
}

void rung_fail(rung_t *r, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  add_detail(r, fmt, ap);
  va_end(ap);
  r->result = R_FAIL;
}

void rung_warn(rung_t *r, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  add_detail(r, fmt, ap);
  va_end(ap);
  if (r->result == R_PASS) r->result = R_WARN;
}

void rung_summary(rung_t *r, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(r->summary, sizeof r->summary, fmt, ap);
  va_end(ap);
}
