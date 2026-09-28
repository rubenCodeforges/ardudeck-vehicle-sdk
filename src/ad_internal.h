/* Shared state and helpers. Not a public header; vendors include ardudeck.h only. */

#ifndef AD_INTERNAL_H
#define AD_INTERNAL_H

#include "ardudeck.h"
#include "ad_msg_defs.h"

/* A payload length is one byte, so anything smaller could silently truncate a frame. */
#if AD_RX_BUFFER < 255
#error "AD_RX_BUFFER must be at least 255"
#endif

#define AD_STX_V2 0xFD
#define AD_STX_V1 0xFE
#define AD_SIGNATURE_LEN 13

#define AD_MAV_TYPE_GENERIC 0
#define AD_MAV_AUTOPILOT_GENERIC 0
/* MAV_TYPE_GCS. What a ground station puts in byte 4 of its heartbeat. */
#define AD_MAV_TYPE_GCS 6
#define AD_MODE_FLAG_CUSTOM 1
#define AD_MODE_FLAG_ARMED 128

/** One outbound frame under construction. Sized by the generator, never guessed. */
typedef struct {
  uint8_t buf[12 + AD_MAX_PAYLOAD + AD_SIGNATURE_LEN];
  uint8_t payload[AD_MAX_PAYLOAD];
  uint8_t at; /* write cursor into payload */
} ad_tx_t;

typedef struct {
  uint8_t  state;
  uint8_t  length;
  uint8_t  incompat;
  uint8_t  seq;
  uint8_t  sysid;
  uint8_t  compid;
  uint32_t msgid;
  uint8_t  msgid_at;
  uint16_t at;
  uint16_t crc;
  uint8_t  crc_low;
  bool     v1;
  uint8_t  payload[AD_RX_BUFFER];
} ad_rx_t;

typedef enum {
  AD_MISSION_IDLE = 0,
  AD_MISSION_RECEIVING,
  AD_MISSION_SENDING,
} ad_mission_state_t;

typedef struct {
  const ad_config_t *cfg;
  uint32_t now;
  uint8_t  sysid;
  uint8_t  compid;
  uint8_t  seq;
  bool     begun;

  ad_tx_t tx;
  ad_rx_t rx;

  /* Who to answer. Learned from the first thing that talks to us. */
  uint8_t  gcs_sysid;
  uint8_t  gcs_compid;
  bool     linked;
  uint32_t last_inbound;

  /* Live state. Absent stays absent: these track whether anything was ever fed.
     Positions are held as the wire holds them, degrees times 1e7, so no double
     reaches the per-frame path on a part with no hardware for it. */
  int32_t lat, lon;
  bool   have_position;
  float  speed_ms, heading_deg;
  uint8_t fix, sats;
  float  amsl_m, relative_m, climb_ms;
  bool   have_altitude;
  float  roll, pitch, yaw;
  bool   have_attitude;
  uint16_t mode, active_item;
  bool   armed;
  float  battery_v, battery_a;
  int8_t battery_pct;
  bool   have_battery;
  int32_t home_lat, home_lon;
  float  home_alt;
  bool   have_home;
  uint16_t rc[18];
  uint8_t  rc_count, rc_rssi;
  bool     have_rc;

  uint32_t next_heartbeat;
  uint32_t next_manifest;
  uint32_t next_stream[AD_STREAM_COUNT];
  uint32_t interval[AD_STREAM_COUNT];
  uint32_t min_interval[AD_STREAM_COUNT];

  /* Parameter enumeration, one per tick so a slow link is never flooded. */
  bool     param_streaming;
  uint16_t param_next;
  bool     meta_streaming;
  uint16_t meta_next;
  uint8_t  meta_option_next;

  /* Manifest burst, also one message per tick. */
  uint8_t manifest_step;

  ad_mission_state_t mission_state;
  uint16_t mission_expect;
  uint16_t mission_at;
  uint32_t mission_deadline;
#if AD_MAX_MISSION_ITEMS > 0
  ad_wp_t mission[AD_MAX_MISSION_ITEMS];
#endif

  uint32_t last_set_mode;

  const ad_calibration_t *cal_active;
  uint32_t cal_last_progress;
} ad_state_t;

extern ad_state_t ad_g;

/* wire */
uint16_t ad_crc16(const uint8_t *data, size_t len, uint16_t crc);
void     ad_tx_begin(void);
void     ad_put_u8(uint8_t v);
void     ad_put_u16(uint16_t v);
void     ad_put_u32(uint32_t v);
void     ad_put_i16(int16_t v);
void     ad_put_i32(int32_t v);
void     ad_put_u64(uint64_t v);
void     ad_put_f32(float v);
void     ad_put_str(const char *s, uint8_t width);
void     ad_put_pad(uint8_t count);
void     ad_tx_send(ad_msg_def_t def);

uint8_t  ad_get_u8(const uint8_t *p, uint16_t len, uint16_t off);
uint16_t ad_get_u16(const uint8_t *p, uint16_t len, uint16_t off);
uint32_t ad_get_u32(const uint8_t *p, uint16_t len, uint16_t off);
int32_t  ad_get_i32(const uint8_t *p, uint16_t len, uint16_t off);
float    ad_get_f32(const uint8_t *p, uint16_t len, uint16_t off);
void     ad_get_str(const uint8_t *p, uint16_t len, uint16_t off, uint8_t width,
                    char *out, size_t out_len);

/** False for a message this build does not know, which is not an error on a shared link. */
bool ad_known_extra(uint32_t msgid, uint8_t *extra);

/* shared helpers */
bool ad_elapsed(uint32_t deadline);
void ad_statustext(ad_severity_t severity, const char *text);
void ad_command_ack(uint16_t cmd, uint8_t result);
void ad_learn_peer(uint8_t sysid, uint8_t compid);

/* subsystems */
void ad_params_reset(void);
void ad_params_handle(uint32_t msgid, const uint8_t *p, uint16_t len);
void ad_params_tick(void);
void ad_params_send_value(uint16_t index);
void ad_params_announce(const char *name);

void ad_mission_reset(void);
bool ad_mission_handle(uint32_t msgid, const uint8_t *p, uint16_t len);
void ad_mission_tick(void);

void ad_cal_reset(void);
void ad_cal_handle(const uint8_t *p, uint16_t len);
void ad_cal_declare_step(uint8_t step);
uint8_t ad_cal_declare_steps(void);

#endif /* AD_INTERNAL_H */
