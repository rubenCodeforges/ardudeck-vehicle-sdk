#include <string.h>

#include "ad_internal.h"

ad_state_t ad_g;

static const ad_config_t *cfg(void) { return ad_g.cfg; }

/** Wrap-safe. A 32 bit millisecond clock rolls over every 49 days and must not stall. */
bool ad_elapsed(uint32_t deadline) {
  return (int32_t)(ad_g.now - deadline) >= 0;
}

void ad_learn_peer(uint8_t sysid, uint8_t compid) {
  if (sysid == 0) return;
  ad_g.gcs_sysid = sysid;
  ad_g.gcs_compid = compid;
  ad_g.linked = true;
  ad_g.last_inbound = ad_g.now;
}

uint32_t ardudeck_silent_for(uint32_t now_ms) {
  if (!ad_g.linked) return AD_NEVER_HEARD;
  return (uint32_t)(now_ms - ad_g.last_inbound);
}

bool ardudeck_linked(void) {
  return ardudeck_silent_for(ad_g.now) < AD_LINK_TIMEOUT_MS;
}

void ad_statustext(ad_severity_t severity, const char *text) {
  if (!ad_g.begun || !text) return;
  ad_tx_begin();
  ad_put_u8((uint8_t)severity);
  ad_put_str(text, 50);
  ad_tx_send(AD_MSG_STATUSTEXT);
}

void ardudeck_notify(ad_severity_t severity, const char *text) {
  ad_statustext(severity, text);
}

void ad_command_ack(uint16_t cmd, uint8_t result) {
  ad_tx_begin();
  ad_put_u16(cmd);
  ad_put_u8(result);
  ad_tx_send(AD_MSG_COMMAND_ACK);
}

/* ─── outbound state ───────────────────────────────────────────────────────── */

/** MAV_TYPE for a declared frame. The icon, and the HUD layout, come from this. */
static uint8_t mav_type_for_frame(ad_frame_t frame) {
  switch (frame) {
    case AD_FRAME_MULTIROTOR:      return 2;  /* QUADROTOR */
    case AD_FRAME_FIXED_WING:      return 1;
    case AD_FRAME_VTOL:            return 21; /* VTOL_TILTROTOR */
    case AD_FRAME_HELICOPTER:      return 4;
    case AD_FRAME_ROVER:           return 10; /* GROUND_ROVER */
    case AD_FRAME_SURFACE_BOAT:    return 11;
    case AD_FRAME_SUBMARINE:       return 12;
    case AD_FRAME_ANTENNA_TRACKER: return 5;
    default:                       return 0;  /* GENERIC */
  }
}

static void send_heartbeat(void) {
  uint8_t base_mode = AD_MODE_FLAG_CUSTOM;
  if (ad_g.armed) base_mode = (uint8_t)(base_mode | AD_MODE_FLAG_ARMED);

  ad_tx_begin();
  ad_put_u32(ad_g.mode);
  ad_put_u8(mav_type_for_frame(cfg()->caps ? cfg()->caps->frame : AD_FRAME_UNKNOWN));
  /*
   * The AUTOPILOT field stays generic on purpose: claiming ArduPilot or PX4 makes a
   * ground station apply that firmware's conventions to a vehicle that does not share
   * them. The TYPE field above is the opposite case, because every ground station draws
   * its icon from it and a generic one loses the vehicle's identity everywhere.
   */
  ad_put_u8(AD_MAV_AUTOPILOT_GENERIC);
  ad_put_u8(base_mode);
  ad_put_u8(ad_g.armed ? 4 : 3); /* MAV_STATE_ACTIVE : MAV_STATE_STANDBY */
  ad_put_u8(3);                  /* MAVLink version */
  ad_tx_send(AD_MSG_HEARTBEAT);
}

static void send_sys_status(void) {
  ad_tx_begin();
  ad_put_u32(0);
  ad_put_u32(0);
  ad_put_u32(0);
  ad_put_u16(0);
  ad_put_u16(ad_g.have_battery ? (uint16_t)(ad_g.battery_v * 1000.0f) : 0xFFFF);
  ad_put_i16(ad_g.have_battery && ad_g.battery_a >= 0.0f
                 ? (int16_t)(ad_g.battery_a * 100.0f)
                 : -1);
  ad_put_u16(0);
  ad_put_u16(0);
  ad_put_u16(0);
  ad_put_u16(0);
  ad_put_u16(0);
  ad_put_u16(0);
  ad_put_u8((uint8_t)(int8_t)(ad_g.have_battery ? ad_g.battery_pct : -1));
  ad_tx_send(AD_MSG_SYS_STATUS);
}

static void send_position(void) {
  int32_t lat = ad_g.have_position ? ad_g.lat : 0;
  int32_t lon = ad_g.have_position ? ad_g.lon : 0;

  ad_tx_begin();
  ad_put_u64((uint64_t)ad_g.now * 1000u);
  ad_put_i32(lat);
  ad_put_i32(lon);
  ad_put_i32((int32_t)(ad_g.amsl_m * 1000.0f));
  ad_put_u16(0xFFFF); /* eph unknown */
  ad_put_u16(0xFFFF); /* epv unknown */
  ad_put_u16((uint16_t)(ad_g.speed_ms * 100.0f));
  ad_put_u16((uint16_t)(ad_g.heading_deg * 100.0f));
  ad_put_u8(ad_g.fix);
  ad_put_u8(ad_g.sats);
  ad_tx_send(AD_MSG_GPS_RAW_INT);

  /* No fix means no marker. A vehicle that reports 0,0 puts a dot in the Gulf of
     Guinea and somebody flies toward it. */
  if (!ad_g.have_position || ad_g.fix < 2) return;

  ad_tx_begin();
  ad_put_u32(ad_g.now);
  ad_put_i32(lat);
  ad_put_i32(lon);
  ad_put_i32((int32_t)(ad_g.amsl_m * 1000.0f));
  ad_put_i32((int32_t)(ad_g.relative_m * 1000.0f));
  ad_put_i16(0);
  ad_put_i16(0);
  ad_put_i16((int16_t)(-ad_g.climb_ms * 100.0f));
  ad_put_u16((uint16_t)(ad_g.heading_deg * 100.0f));
  ad_tx_send(AD_MSG_GLOBAL_POSITION_INT);
}

static void send_vfr_hud(void) {
  ad_tx_begin();
  ad_put_f32(ad_g.speed_ms);
  ad_put_f32(ad_g.speed_ms);
  ad_put_f32(ad_g.have_altitude ? ad_g.amsl_m : 0.0f);
  ad_put_f32(ad_g.climb_ms);
  ad_put_i16((int16_t)ad_g.heading_deg);
  ad_put_u16(0);
  ad_tx_send(AD_MSG_VFR_HUD);
}

static void send_attitude(void) {
  if (!ad_g.have_attitude) return;
  ad_tx_begin();
  ad_put_u32(ad_g.now);
  ad_put_f32(ad_g.roll);
  ad_put_f32(ad_g.pitch);
  ad_put_f32(ad_g.yaw);
  ad_put_f32(0.0f);
  ad_put_f32(0.0f);
  ad_put_f32(0.0f);
  ad_tx_send(AD_MSG_ATTITUDE);
}

static void send_rc(void) {
  if (!ad_g.have_rc) return;
  ad_tx_begin();
  ad_put_u32(ad_g.now);
  for (uint8_t i = 0; i < 18; i++) {
    ad_put_u16(i < ad_g.rc_count ? ad_g.rc[i] : 0xFFFF);
  }
  ad_put_u8(ad_g.rc_count);
  ad_put_u8(ad_g.rc_rssi);
  ad_tx_send(AD_MSG_RC_CHANNELS);
}

static void send_home(void) {
  if (!ad_g.have_home) return;
  ad_tx_begin();
  ad_put_i32(ad_g.home_lat);
  ad_put_i32(ad_g.home_lon);
  ad_put_i32((int32_t)(ad_g.home_alt * 1000.0f));
  ad_put_pad(40); /* local x y z, q[4], approach x y z: all unknown here */
  ad_tx_send(AD_MSG_HOME_POSITION);
}

static void send_mission_current(void) {
  ad_tx_begin();
  ad_put_u16(ad_g.active_item);
  ad_tx_send(AD_MSG_MISSION_CURRENT);
}

/* ─── the manifest ─────────────────────────────────────────────────────────── */

static void send_manifest(void) {
  const ad_capability_t *c = cfg()->caps;
  ad_tx_begin();
  ad_put_u32(c ? c->features : 0);
  ad_put_u16(AD_PROFILE_VERSION);
  ad_put_u16(c ? c->mission_capacity : 0);
  ad_put_u16(cfg()->param_count);
  ad_put_u8(c ? (uint8_t)c->frame : AD_FRAME_UNKNOWN);
  ad_put_u8(c ? c->mode_count : 0);
  ad_put_u8(c ? (uint8_t)c->mission_cmd_count : 0);
  ad_put_u8(cfg()->calibration_count);
  ad_put_str(c ? c->vendor : "", 20);
  ad_put_str(c ? c->model : "", 20);
  ad_put_str(c ? c->firmware : "", 16);
  ad_put_str(c && c->uid ? c->uid : "", 24);
  ad_tx_send(AD_MSG_MANIFEST);
}

static void send_mission_cmds(void) {
  const ad_capability_t *c = cfg()->caps;
  uint16_t n = c ? c->mission_cmd_count : 0;
  if (n > 32) n = 32;
  ad_tx_begin();
  for (uint16_t i = 0; i < 32; i++) {
    ad_put_u16(i < n && c->mission_cmds ? c->mission_cmds[i] : 0);
  }
  ad_put_u8((uint8_t)n);
  ad_tx_send(AD_MSG_MISSION_CMDS);
}

static void send_mode(uint8_t index) {
  const ad_capability_t *c = cfg()->caps;
  if (!c || !c->modes || index >= c->mode_count) return;
  const ad_mode_t *m = &c->modes[index];
  ad_tx_begin();
  ad_put_u16(m->id);
  ad_put_u8(index);
  ad_put_u8(c->mode_count);
  ad_put_u8(m->flags);
  ad_put_str(m->name ? m->name : "", 16);
  ad_tx_send(AD_MSG_MODE);
}

static uint8_t manifest_steps(void) {
  const ad_capability_t *c = cfg()->caps;
  return (uint8_t)(2 + (c ? c->mode_count : 0) + ad_cal_declare_steps());
}

/** One message per tick, so describing a vehicle never floods a slow link. */
static void manifest_tick(void) {
  uint8_t total = manifest_steps();

  if (ad_g.manifest_step >= total) {
    if (!ad_elapsed(ad_g.next_manifest)) return;
    ad_g.manifest_step = 0;
  }

  uint8_t step = ad_g.manifest_step++;
  if (ad_g.manifest_step >= total) {
    ad_g.next_manifest =
        ad_g.now + (ad_g.linked ? AD_MANIFEST_IDLE_MS : AD_MANIFEST_SEARCH_MS);
  }
  if (step == 0) { send_manifest(); return; }
  if (step == 1) { send_mission_cmds(); return; }
  step = (uint8_t)(step - 2);

  const ad_capability_t *c = cfg()->caps;
  uint8_t modes = c ? c->mode_count : 0;
  if (step < modes) { send_mode(step); return; }
  ad_cal_declare_step((uint8_t)(step - modes));
}

/* ─── inbound ──────────────────────────────────────────────────────────────── */

#define MAV_RESULT_ACCEPTED    0
#define MAV_RESULT_DENIED      2
#define MAV_RESULT_UNSUPPORTED 3

#if AD_WITH_COMMANDS
/**
 * Is this one of the modes the firmware actually published?
 *
 * A ground station that does not read the declared table has to guess, and the usual
 * guess is an ArduPilot mode number for the frame type. Acting on it would move the
 * vehicle into whatever this firmware happens to number the same. An undeclared id is
 * refused instead, which turns a silent wrong mode into a visible refusal.
 */
static bool mode_declared(uint16_t id) {
  const ad_capability_t *c = cfg()->caps;
  if (!c || !c->modes) return false;
  for (uint8_t i = 0; i < c->mode_count; i++) {
    if (c->modes[i].id == id) return true;
  }
  return false;
}

/**
 * Hand the vendor the argument the command is about, wherever MAVLink put it.
 *
 * DO_SET_MODE carries the custom mode in param2, behind a base-mode bitmask in param1
 * whose bit 0 says whether the custom mode is meaningful at all. NAV_TAKEOFF carries
 * altitude in param7. A vendor reading a[0] for either gets a bitmask or a zero, which
 * is the kind of wart this library exists to absorb.
 */
static bool normalise_args(uint16_t cmd, float *args) {
  if (cmd == AD_CMD_SET_MODE) {
    uint32_t base = (uint32_t)args[0];
    if (!(base & 0x01)) return false; /* custom mode not enabled: nothing to act on */
    float mode = args[1];
    for (uint8_t i = 0; i < 7; i++) args[i] = 0.0f;
    args[0] = mode;
  } else if (cmd == AD_CMD_TAKEOFF) {
    float alt = args[6];
    args[0] = alt;
  }
  return true;
}
#endif /* AD_WITH_COMMANDS: both helpers are only reachable from the command paths */

static void handle_command(const uint8_t *p, uint16_t len) {
#if AD_WITH_COMMANDS
  float args[7];
  for (uint8_t i = 0; i < 7; i++) args[i] = ad_get_f32(p, len, (uint16_t)(i * 4));
  uint16_t cmd = ad_get_u16(p, len, 28);

  const ad_capability_t *c = cfg()->caps;
  if (!c || !(c->features & AD_FEAT_COMMANDS) || !cfg()->on_command) {
    ad_command_ack(cmd, MAV_RESULT_UNSUPPORTED);
    return;
  }

  if (!normalise_args(cmd, args)) {
    ad_command_ack(cmd, MAV_RESULT_ACCEPTED);
    return;
  }

  if (cmd == AD_CMD_SET_MODE && !mode_declared((uint16_t)args[0])) {
    /* Stamped so the legacy SET_MODE that follows does not repeat the complaint. */
    ad_g.last_set_mode = ad_g.now ? ad_g.now : 1;
    ad_command_ack(cmd, MAV_RESULT_DENIED);
    ad_statustext(AD_WARNING, "Unknown mode, ignored");
    return;
  }

  char why[64];
  why[0] = '\0';
  bool ok = cfg()->on_command(cmd, args, why, sizeof why, cfg()->user);
  if (cmd == AD_CMD_SET_MODE) ad_g.last_set_mode = ad_g.now ? ad_g.now : 1;

  /* Acknowledged the moment the callback returns, which is why a command that reboots
     or blocks has to return first and act afterwards. */
  ad_command_ack(cmd, ok ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED);
  if (!ok && why[0] != '\0') ad_statustext(AD_WARNING, why);
#else
  (void)p;
  ad_command_ack(ad_get_u16(p, len, 28), MAV_RESULT_UNSUPPORTED);
#endif
}

/**
 * The legacy SET_MODE message, for ground stations that send only that.
 *
 * ArduDeck sends it immediately after DO_SET_MODE as a fallback for older ArduPilot
 * builds, so acting on both would switch mode twice and call the vendor twice for one
 * press. The recent-command window suppresses the duplicate.
 */
static void handle_set_mode(const uint8_t *p, uint16_t len) {
#if AD_WITH_COMMANDS
  if (ad_g.last_set_mode != 0 && (uint32_t)(ad_g.now - ad_g.last_set_mode) < 2000) return;

  uint8_t base = ad_get_u8(p, len, 5);
  if (!(base & 0x01)) return;

  const ad_capability_t *c = cfg()->caps;
  if (!c || !(c->features & AD_FEAT_COMMANDS) || !cfg()->on_command) return;

  float args[7] = {0};
  args[0] = (float)ad_get_u32(p, len, 0);
  if (!mode_declared((uint16_t)args[0])) {
    ad_statustext(AD_WARNING, "Unknown mode, ignored");
    return;
  }

  char why[64];
  why[0] = '\0';
  /* SET_MODE has no acknowledgement in the protocol, so a refusal can only be words. */
  if (!cfg()->on_command(AD_CMD_SET_MODE, args, why, sizeof why, cfg()->user) && why[0]) {
    ad_statustext(AD_WARNING, why);
  }
#else
  (void)p; (void)len;
#endif
}

static void handle_request(const uint8_t *p, uint16_t len) {
  switch (ad_get_u8(p, len, 4)) {
    case 0: /* manifest */
      ad_g.manifest_step = 0;
      ad_g.next_manifest = ad_g.now;
      break;
    case 1: /* parameter metadata */
      ad_g.meta_streaming = true;
      ad_g.meta_next = 0;
      ad_g.meta_option_next = 0;
      break;
    case 2: /* calibration */
      ad_g.manifest_step = (uint8_t)(2 + (cfg()->caps ? cfg()->caps->mode_count : 0));
      break;
    default:
      break;
  }
}

static void dispatch(uint32_t msgid, const uint8_t *p, uint16_t len) {
  if (msgid == AD_MSG_PARAM_REQUEST_LIST.id || msgid == AD_MSG_PARAM_REQUEST_READ.id ||
      msgid == AD_MSG_PARAM_SET.id) {
    const ad_capability_t *c = cfg()->caps;
    if (c && (c->features & AD_FEAT_PARAMS)) ad_params_handle(msgid, p, len);
    return;
  }

  if (msgid == AD_MSG_COMMAND_LONG.id) { handle_command(p, len); return; }
  if (msgid == AD_MSG_SET_MODE.id)     { handle_set_mode(p, len); return; }
  if (msgid == AD_MSG_CAL_CONTROL.id)  { ad_cal_handle(p, len); return; }
  if (msgid == AD_MSG_REQUEST.id)      { handle_request(p, len); return; }

  ad_mission_handle(msgid, p, len);
}

/*
 * Frame parser. Accepts v1 and v2 because ground stations still send both, and skips a
 * v2 signature rather than choking on it. Emits v2 only.
 */
enum {
  S_STX = 0, S_LEN, S_INCOMPAT, S_COMPAT, S_SEQ, S_SYSID, S_COMPID,
  S_MSGID, S_PAYLOAD, S_CRC1, S_CRC2, S_SIGNATURE
};

static void feed(uint8_t b) {
  ad_rx_t *r = &ad_g.rx;

  switch (r->state) {
    case S_STX:
      if (b != AD_STX_V2 && b != AD_STX_V1) return;
      r->v1 = (b == AD_STX_V1);
      r->crc = 0xFFFF;
      r->incompat = 0;
      r->msgid = 0;
      r->msgid_at = 0;
      r->at = 0;
      r->state = S_LEN;
      return;

    case S_LEN:
      r->length = b;
      r->crc = ad_crc16(&b, 1, r->crc);
      r->state = r->v1 ? S_SEQ : S_INCOMPAT;
      return;

    case S_INCOMPAT:
      r->incompat = b;
      r->crc = ad_crc16(&b, 1, r->crc);
      r->state = S_COMPAT;
      return;

    case S_COMPAT:
      r->crc = ad_crc16(&b, 1, r->crc);
      r->state = S_SEQ;
      return;

    case S_SEQ:
      r->seq = b;
      r->crc = ad_crc16(&b, 1, r->crc);
      r->state = S_SYSID;
      return;

    case S_SYSID:
      r->sysid = b;
      r->crc = ad_crc16(&b, 1, r->crc);
      r->state = S_COMPID;
      return;

    case S_COMPID:
      r->compid = b;
      r->crc = ad_crc16(&b, 1, r->crc);
      r->state = S_MSGID;
      return;

    case S_MSGID:
      r->crc = ad_crc16(&b, 1, r->crc);
      r->msgid |= (uint32_t)b << (8 * r->msgid_at);
      r->msgid_at++;
      /* v1 carries a single byte id, v2 three. */
      if (r->msgid_at >= (r->v1 ? 1 : 3)) r->state = r->length ? S_PAYLOAD : S_CRC1;
      return;

    case S_PAYLOAD:
      if (r->at < AD_RX_BUFFER) r->payload[r->at] = b;
      r->at++;
      r->crc = ad_crc16(&b, 1, r->crc);
      if (r->at >= r->length) r->state = S_CRC1;
      return;

    case S_CRC1:
      r->crc_low = b;
      r->state = S_CRC2;
      return;

    case S_CRC2: {
      uint16_t got = (uint16_t)(r->crc_low | ((uint16_t)b << 8));

      /* A signed v2 frame carries 13 more bytes. Discard them: signing is the link's
         business, and choking here would lose every following frame too. */
      r->state = (!r->v1 && (r->incompat & 0x01)) ? S_SIGNATURE : S_STX;
      if (r->state == S_SIGNATURE) r->at = 0;

      uint8_t extra;
      if (!ad_known_extra(r->msgid, &extra)) return;
      if (ad_crc16(&extra, 1, r->crc) != got) return;

      /*
       * A heartbeat only counts as contact when it came from a ground station. On a
       * shared or broadcast link every vehicle is heartbeating too, and learning one of
       * those as the peer would aim telemetry at another vehicle and hold the failsafe
       * open with nobody watching. MAV_TYPE_GCS is byte 4 of the payload.
       */
      if (r->msgid == AD_MSG_HEARTBEAT.id) {
        if (ad_get_u8(r->payload, r->length, 4) != AD_MAV_TYPE_GCS) return;
        ad_learn_peer(r->sysid, r->compid);
        return; /* nothing in a heartbeat is ours to act on */
      }

      ad_learn_peer(r->sysid, r->compid);
      dispatch(r->msgid, r->payload, r->length);
      return;
    }

    case S_SIGNATURE:
      if (++r->at >= AD_SIGNATURE_LEN) r->state = S_STX;
      return;

    default:
      r->state = S_STX;
      return;
  }
}

void ardudeck_receive(const uint8_t *buf, size_t len) {
  if (!ad_g.begun || !buf) return;
  for (size_t i = 0; i < len; i++) feed(buf[i]);
}

/* ─── state setters ────────────────────────────────────────────────────────── */

void ardudeck_position(double lat, double lon, float speed_ms, float heading_deg,
                       uint8_t fix, uint8_t sats) {
  ad_g.lat = (int32_t)(lat * 1e7);
  ad_g.lon = (int32_t)(lon * 1e7);
  ad_g.speed_ms = speed_ms;
  ad_g.heading_deg = heading_deg;
  ad_g.fix = fix;
  ad_g.sats = sats;
  ad_g.have_position = fix >= 2;
}

void ardudeck_altitude(float amsl_m, float relative_m, float climb_ms) {
  ad_g.amsl_m = amsl_m;
  ad_g.relative_m = relative_m;
  ad_g.climb_ms = climb_ms;
  ad_g.have_altitude = true;
}

void ardudeck_attitude(float roll, float pitch, float yaw) {
  ad_g.roll = roll;
  ad_g.pitch = pitch;
  ad_g.yaw = yaw;
  ad_g.have_attitude = true;
}

void ardudeck_status(uint16_t mode, uint16_t active_item, float battery_v, bool armed) {
  ad_g.mode = mode;
  ad_g.active_item = active_item;
  ad_g.armed = armed;
  if (battery_v > 0.0f) {
    ad_g.battery_v = battery_v;
    ad_g.have_battery = true;
  }
}

void ardudeck_battery(float volts, float amps, int8_t percent) {
  ad_g.battery_v = volts;
  ad_g.battery_a = amps;
  ad_g.battery_pct = percent;
  ad_g.have_battery = volts > 0.0f;
}

void ardudeck_home(double lat, double lon, float alt) {
  ad_g.home_lat = (int32_t)(lat * 1e7);
  ad_g.home_lon = (int32_t)(lon * 1e7);
  ad_g.home_alt = alt;
  ad_g.have_home = true;
}

void ardudeck_rc(const uint16_t *channels, uint8_t count, uint8_t rssi) {
  if (!channels) return;
  if (count > 18) count = 18;
  for (uint8_t i = 0; i < count; i++) ad_g.rc[i] = channels[i];
  ad_g.rc_count = count;
  ad_g.rc_rssi = rssi;
  ad_g.have_rc = true;
}

void ardudeck_param_changed(const char *name) {
  if (ad_g.begun && name) ad_params_announce(name);
}

void ardudeck_rate_limit(ad_stream_t stream, float max_hz) {
  if (stream >= AD_STREAM_COUNT) return;
  uint32_t floor_ms = max_hz > 0.0f ? (uint32_t)(1000.0f / max_hz) : 0xFFFFFFFFu;
  ad_g.min_interval[stream] = floor_ms;
  if (ad_g.interval[stream] < floor_ms) ad_g.interval[stream] = floor_ms;
}

/* ─── lifecycle ────────────────────────────────────────────────────────────── */

static const uint32_t DEFAULT_INTERVAL[AD_STREAM_COUNT] = {
  250,  /* position, 4 Hz */
  100,  /* attitude, 10 Hz */
  1000, /* status, 1 Hz */
  0,    /* rc, off until asked for */
};

void ardudeck_begin(const ad_config_t *config) {
  memset(&ad_g, 0, sizeof ad_g);
  if (!config || !config->send || !config->now_ms) return;

  ad_g.cfg = config;
  ad_g.sysid = config->system_id ? config->system_id : AD_DEFAULT_SYSTEM_ID;
  ad_g.compid = config->component_id ? config->component_id : AD_DEFAULT_COMPONENT_ID;
  ad_g.gcs_sysid = 255; /* the conventional ground station id, until one identifies */
  ad_g.gcs_compid = 190;
  ad_g.battery_pct = -1;
  ad_g.now = config->now_ms();

  for (int i = 0; i < AD_STREAM_COUNT; i++) {
    ad_g.interval[i] = DEFAULT_INTERVAL[i];
    ad_g.min_interval[i] = 0;
    ad_g.next_stream[i] = ad_g.now;
  }

  ad_g.next_heartbeat = ad_g.now;
  ad_g.next_manifest = ad_g.now;
  ad_g.manifest_step = 0;

  ad_params_reset();
  ad_mission_reset();
  ad_cal_reset();
  ad_g.begun = true;
}

static bool stream_due(ad_stream_t s) {
  if (ad_g.interval[s] == 0 || ad_g.interval[s] == 0xFFFFFFFFu) return false;
  if (!ad_elapsed(ad_g.next_stream[s])) return false;
  ad_g.next_stream[s] = ad_g.now + ad_g.interval[s];
  return true;
}

void ardudeck_tick(uint32_t now_ms) {
  if (!ad_g.begun) return;
  ad_g.now = now_ms;

  if (ad_elapsed(ad_g.next_heartbeat)) {
    ad_g.next_heartbeat = ad_g.now + 1000;
    send_heartbeat();
  }

  if (stream_due(AD_STREAM_POSITION)) {
    send_position();
    send_vfr_hud();
  }
  if (stream_due(AD_STREAM_ATTITUDE)) send_attitude();
  if (stream_due(AD_STREAM_STATUS)) {
    send_sys_status();
    send_mission_current();
    send_home();
  }
  if (stream_due(AD_STREAM_RC)) send_rc();

  manifest_tick();
  ad_params_tick();
  ad_mission_tick();
}
