/*
 * ardudeck-conform: does this vehicle actually do what it claims?
 *
 * Safety rules this tool keeps, and a vendor should be able to rely on:
 *   - It never arms, never disarms, never commands a mode change.
 *   - It never starts a calibration that declares AD_CAL_REQ_MOTORS_LIVE.
 *   - It reads the stored mission before touching it and puts it back afterwards.
 *     When the vehicle cannot read one back, the mission checks are skipped unless
 *     --allow-mission-write is given, because the alternative is silently wiping a
 *     plan somebody flew out to load.
 */

#ifndef CONFORM_H
#define CONFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ─── transport ────────────────────────────────────────────────────────────── */

typedef struct link link_t;

/** "14550", "192.168.4.1:14550". Binds and learns the peer from the first packet. */
link_t *link_open_udp(const char *spec, char *err, size_t err_len);

/** "/dev/ttyUSB0:57600" */
link_t *link_open_serial(const char *spec, char *err, size_t err_len);

bool link_send(link_t *l, const uint8_t *buf, size_t len);

/** Bytes read, 0 on timeout, -1 on error. */
int link_recv(link_t *l, uint8_t *buf, size_t cap, int timeout_ms);

void link_close(link_t *l);

/** Milliseconds since the tool started. Monotonic. */
uint32_t now_ms(void);

/* ─── MAVLink ──────────────────────────────────────────────────────────────── */

typedef struct {
  uint32_t id;
  uint8_t  sysid;
  uint8_t  compid;
  uint8_t  len;
  uint8_t  payload[255];
} mav_msg_t;

typedef struct {
  uint8_t  state;
  uint8_t  length;
  uint8_t  incompat;
  uint8_t  sysid;
  uint8_t  compid;
  uint32_t msgid;
  uint8_t  msgid_at;
  uint16_t at;
  uint16_t crc;
  uint8_t  crc_low;
  bool     v1;
  uint8_t  payload[255];
} mav_parser_t;

void mav_reset(mav_parser_t *p);

/** True when a byte completed a frame whose checksum is right. */
bool mav_feed(mav_parser_t *p, uint8_t b, mav_msg_t *out);

size_t mav_build(uint8_t *out, uint32_t id, uint8_t crc_extra, const uint8_t *payload,
                 uint8_t len, uint8_t seq, uint8_t sysid, uint8_t compid);

/** CRC extra for a message id, or false when this build does not know it. */
bool mav_extra(uint32_t id, uint8_t *extra);

/* Field readers. Every one zero-pads, because a v2 sender truncates trailing zeros. */
uint8_t  mav_u8(const mav_msg_t *m, uint16_t off);
uint16_t mav_u16(const mav_msg_t *m, uint16_t off);
uint32_t mav_u32(const mav_msg_t *m, uint16_t off);
int32_t  mav_i32(const mav_msg_t *m, uint16_t off);
float    mav_f32(const mav_msg_t *m, uint16_t off);
void     mav_str(const mav_msg_t *m, uint16_t off, uint8_t width, char *out,
                 size_t out_len);

void mav_put_u8(uint8_t *p, uint8_t v);
void mav_put_u16(uint8_t *p, uint16_t v);
void mav_put_i32(uint8_t *p, int32_t v);
void mav_put_f32(uint8_t *p, float v);

/* ─── results ──────────────────────────────────────────────────────────────── */

typedef enum { R_PASS = 0, R_WARN, R_FAIL, R_SKIP } result_t;

#define MAX_DETAIL 8

typedef struct {
  const char *rung;
  const char *name;
  result_t    result;
  char        summary[200];
  char        detail[MAX_DETAIL][200];
  int         detail_count;
} rung_t;

void rung_note(rung_t *r, const char *fmt, ...);
void rung_fail(rung_t *r, const char *fmt, ...);
void rung_warn(rung_t *r, const char *fmt, ...);
void rung_summary(rung_t *r, const char *fmt, ...);

/* ─── what the vehicle said about itself ───────────────────────────────────── */

/* Mirrors AD_FEAT_* in ardudeck.h. The tool deliberately does not include that header:
   it must be able to test a vehicle that never used this SDK. */
#define AD_FEAT_PARAMS_BIT       (1u << 0)
#define AD_FEAT_MISSION_BIT      (1u << 1)
#define AD_FEAT_COMMANDS_BIT     (1u << 2)
#define AD_FEAT_CAL_BIT          (1u << 5)
#define AD_FEAT_MISSION_READ_BIT (1u << 9)

/* Bit numbers are fixed, but this profile version defines no protocol for these. */
#define AD_FEAT_RESERVED_BITS    ((1u << 4) | (1u << 6) | (1u << 7))

#define MAX_MODES 32
#define MAX_PARAMS 512
#define MAX_CALS 16
#define MAX_TRACKS 3
#define MAX_POSES 12

typedef struct {
  uint16_t id;
  char     name[17];
  uint8_t  flags;
  bool     seen;
} mode_info_t;

typedef struct {
  char     name[17];
  float    value;
  uint16_t index;
  bool     have_value;

  bool     have_meta;
  float    min_value, max_value, increment;
  uint8_t  flags, option_count;
  char     unit[13];
  char     help[81];
  int      options_seen;
} param_info_t;

typedef struct {
  char    id[17];
  char    name[25];
  uint8_t kind;
  uint8_t requirements;
  uint8_t pose_count, track_count;
  int     poses_seen, tracks_seen;
  bool    seen;
} cal_info_t;

typedef struct {
  bool     have_manifest;
  uint32_t features;
  uint16_t profile_version;
  uint16_t mission_capacity;
  uint16_t param_count;
  uint8_t  frame;
  uint8_t  mode_count;
  uint8_t  mission_cmd_count;
  uint8_t  cal_count;
  char     vendor[21], model[21], firmware[17], uid[25];

  mode_info_t modes[MAX_MODES];
  int         modes_seen;

  uint16_t mission_cmds[32];
  int      mission_cmds_seen;
  bool     have_mission_cmds;

  param_info_t params[MAX_PARAMS];
  int          params_seen;

  cal_info_t cals[MAX_CALS];
  int        cals_seen;

  /* rung 0 evidence */
  int      heartbeats, positions, attitudes, gps_raws, sys_status;
  uint32_t first_heartbeat_ms, last_heartbeat_ms;
  uint8_t  autopilot, vehicle_type;
  uint8_t  fix;
  int32_t  last_lat, last_lon;
  bool     position_without_fix;
  bool     zero_island;

  uint8_t sysid, compid;
} vehicle_t;

/* ─── the session ──────────────────────────────────────────────────────────── */

typedef struct {
  link_t      *link;
  mav_parser_t parser;
  vehicle_t    v;
  uint8_t      seq;
  uint8_t      sysid, compid;
  bool         verbose;
  bool         allow_mission_write;
} session_t;

/** Pump the link for up to ms, folding everything heard into the vehicle picture. */
void pump(session_t *s, int ms);

/** Pump until a message arrives, or the timeout expires. Returns false on timeout. */
bool wait_for(session_t *s, uint32_t msgid, int timeout_ms, mav_msg_t *out);

void send_msg(session_t *s, uint32_t id, const uint8_t *payload, uint8_t len);

void check_rung0(session_t *s, rung_t *r);
void check_rung1(session_t *s, rung_t *r);
void check_rung2(session_t *s, rung_t *r);
void check_rung3(session_t *s, rung_t *r);
void check_rung4(session_t *s, rung_t *r);
void check_calibration(session_t *s, rung_t *r);

#endif /* CONFORM_H */
