/*
 * Host tests. Plain cc, no board, no framework, about a second to run.
 *
 * A passing test proves nothing until you have watched it fail. Break something on
 * purpose and confirm the relevant case goes red before trusting any of this.
 */

#include <stdio.h>
#include <string.h>

#include "ardudeck.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...)                                                               \
  do {                                                                                 \
    checks++;                                                                          \
    if (!(cond)) {                                                                     \
      failures++;                                                                      \
      printf("  FAIL %s:%d ", __FILE__, __LINE__);                                     \
      printf(__VA_ARGS__);                                                             \
      printf("\n");                                                                    \
    }                                                                                  \
  } while (0)

/* ─── a fake link ──────────────────────────────────────────────────────────── */

#define CAP 8192
static uint8_t sent[CAP];
static size_t sent_len;
static uint32_t clock_ms;

static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  if (sent_len + len <= CAP) {
    memcpy(&sent[sent_len], buf, len);
    sent_len += len;
  }
}

static uint32_t now(void) { return clock_ms; }

static void drain(void) { sent_len = 0; }

/** Find the first frame with this message id in what the SDK sent. */
static const uint8_t *find(uint32_t msgid, uint8_t *out_len) {
  size_t i = 0;
  while (i + 12 <= sent_len) {
    if (sent[i] != 0xFD) { i++; continue; }
    uint8_t len = sent[i + 1];
    if (i + 12 + len > sent_len) break;
    uint32_t id = (uint32_t)sent[i + 7] | ((uint32_t)sent[i + 8] << 8) |
                  ((uint32_t)sent[i + 9] << 16);
    if (id == msgid) {
      if (out_len) *out_len = len;
      return &sent[i + 10];
    }
    i += (size_t)(12 + len);
  }
  return NULL;
}

static int count_of(uint32_t msgid) {
  size_t i = 0;
  int n = 0;
  while (i + 12 <= sent_len) {
    if (sent[i] != 0xFD) { i++; continue; }
    uint8_t len = sent[i + 1];
    if (i + 12 + len > sent_len) break;
    uint32_t id = (uint32_t)sent[i + 7] | ((uint32_t)sent[i + 8] << 8) |
                  ((uint32_t)sent[i + 9] << 16);
    if (id == msgid) n++;
    i += (size_t)(12 + len);
  }
  return n;
}

/**
 * Copy a payload out, zero padded to the full length.
 *
 * A v2 sender truncates trailing zeros, so reading a field straight out of the buffer
 * runs into the checksum. Every reader has to pad, including this one.
 */
static bool payload_of(uint32_t msgid, uint8_t *out, size_t size) {
  uint8_t len = 0;
  const uint8_t *p = find(msgid, &len);
  memset(out, 0, size);
  if (!p) return false;
  memcpy(out, p, len < size ? len : size);
  return true;
}

static uint16_t u16_at(const uint8_t *p, int off) {
  return (uint16_t)(p[off] | ((uint16_t)p[off + 1] << 8));
}

static float f32_at(const uint8_t *p, int off) {
  float v;
  memcpy(&v, p + off, 4);
  return v;
}

/* ─── a vehicle to test against ────────────────────────────────────────────── */

static float cruise = 1.5f;
static int32_t timeout_s = 20;
static uint8_t fs_action = 1;

static const char *const FS_OPTIONS[] = {"Continue", "Hold", "Return home", "Stop"};

static bool fs_get(const ad_param_t *p, float *out) {
  (void)p;
  *out = (float)fs_action;
  return true;
}
static bool fs_set(const ad_param_t *p, float v) {
  (void)p;
  fs_action = (uint8_t)v;
  return true;
}

static const ad_param_t PARAMS[] = {
  AD_F32("CRUISE_SPD", &cruise, 0.2f, 4.0f, "m/s", "Speed held between waypoints"),
  AD_I32("LINK_TIMEOUT", &timeout_s, 1, 120, "s", "Silence before the failsafe fires"),
  AD_ENUM("FS_ACTION", fs_get, fs_set, FS_OPTIONS, 4, "What to do when the link fails"),
};

static const uint16_t MISSION_CMDS[] = {AD_NAV_WAYPOINT, AD_NAV_RETURN_TO_LAUNCH};

static const ad_mode_t MODES[] = {
  {0, "Idle", 0},
  {1, "Running", 0},
  {4, "Mission", AD_MODE_MISSION},
  {6, "Manual", AD_MODE_LOCAL_ONLY},
};

static const ad_cal_track_t COMPASS_TRACKS[] = {
  {"Sectors turned through", "", 12.0f},
  {"Total turning", "deg", 720.0f},
};

static const ad_cal_pose_t ACCEL_POSES[] = {
  {"Level", 0.0f, 0.0f},
  {"Left side", -90.0f, 0.0f},
};

static const ad_calibration_t CALS[] = {
  {
    .id = "compass", .name = "Compass", .kind = AD_CAL_COVERAGE,
    .requirements = AD_CAL_REQ_DISARMED,
    .warning = "Turn the boat through two full circles by hand.",
    .tracks = COMPASS_TRACKS, .track_count = 2,
  },
  {
    .id = "accel", .name = "Accelerometer", .kind = AD_CAL_POSITIONAL,
    .requirements = AD_CAL_REQ_DISARMED | AD_CAL_REQ_STATIONARY,
    .poses = ACCEL_POSES, .pose_count = 2,
  },
};

static const ad_capability_t CAPS = {
  .vendor = "acme", .model = "boat kit", .firmware = "0.4.1",
  .frame = AD_FRAME_SURFACE_BOAT,
  .features = AD_FEAT_PARAMS | AD_FEAT_MISSION | AD_FEAT_COMMANDS |
              AD_FEAT_CALIBRATION | AD_FEAT_MISSION_READ,
  .mission_cmds = MISSION_CMDS, .mission_cmd_count = 2, .mission_capacity = 32,
  .modes = MODES, .mode_count = 3,
};

static uint16_t last_mission_count;
static bool refuse_mission;
static uint16_t last_command;
static int command_calls;
static bool refuse_command;
static char last_cal[20];
static ad_cal_action_t last_action;
static int param_saves;

static bool on_param_set(const ad_param_t *p, float value, void *user) {
  (void)p; (void)value; (void)user;
  param_saves++;
  return true;
}

static bool on_mission(const ad_wp_t *items, uint16_t count, char *why, size_t why_len,
                       void *user) {
  (void)items; (void)user;
  last_mission_count = count;
  if (refuse_mission) {
    snprintf(why, why_len, "No GPS fix. 2 satellites, needs 5.");
    return false;
  }
  return true;
}

static uint16_t stored_count = 2;
static uint16_t on_mission_read(ad_wp_t *items, uint16_t capacity, void *user) {
  (void)user;
  if (capacity < stored_count) return 0;
  for (uint16_t i = 0; i < stored_count; i++) {
    items[i].seq = i;
    items[i].command = AD_NAV_WAYPOINT;
    items[i].alt_frame = AD_ALT_RELATIVE;
    items[i].lat = 52.5 + i * 0.001;
    items[i].lon = 13.4;
    items[i].alt = 30.0f;
    items[i].p1 = items[i].p2 = items[i].p3 = items[i].p4 = 0.0f;
  }
  return stored_count;
}

static float last_args[7];

static bool on_command(uint16_t cmd, const float args[7], char *why, size_t why_len,
                       void *user) {
  (void)user;
  last_command = cmd;
  command_calls++;
  for (int i = 0; i < 7; i++) last_args[i] = args[i];
  if (refuse_command) {
    snprintf(why, why_len, "No GPS fix. 2 satellites, needs 5.");
    return false;
  }
  return true;
}

static bool on_calibrate(const char *cal_id, ad_cal_action_t action, char *why,
                         size_t why_len, void *user) {
  (void)why; (void)why_len; (void)user;
  snprintf(last_cal, sizeof last_cal, "%s", cal_id);
  last_action = action;
  return true;
}

static ad_config_t make_config(void) {
  ad_config_t cfg;
  memset(&cfg, 0, sizeof cfg);
  cfg.caps = &CAPS;
  cfg.params = PARAMS;
  cfg.param_count = AD_COUNT(PARAMS);
  cfg.calibrations = CALS;
  cfg.calibration_count = AD_COUNT(CALS);
  cfg.send = sink;
  cfg.now_ms = now;
  cfg.on_param_set = on_param_set;
  cfg.on_mission = on_mission;
  cfg.on_mission_read = on_mission_read;
  cfg.on_command = on_command;
  cfg.on_calibrate = on_calibrate;
  return cfg;
}

static void boot_at(uint32_t start) {
  clock_ms = start;
  drain();
  param_saves = 0;
  refuse_mission = false;
  refuse_command = false;
  last_mission_count = 0xFFFF;
  last_command = 0;
  command_calls = 0;
  memset(last_args, 0, sizeof last_args);
  last_cal[0] = '\0';
  static ad_config_t cfg;
  cfg = make_config();
  ardudeck_begin(&cfg);
}

static void boot(void) { boot_at(1000); }

/** Run the loop for a while, so anything sent one message per tick gets its turn. */
static void run_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i += 20) {
    clock_ms += 20;
    ardudeck_tick(clock_ms);
  }
}

/* ─── building inbound frames ──────────────────────────────────────────────── */

/* Every message the SDK is allowed to emit, and its published CRC extra. */
static bool extra_for(uint32_t id, uint8_t *out) {
  static const struct { uint32_t id; uint8_t extra; } TABLE[] = {
    {0, 50}, {1, 124}, {22, 220}, {24, 24}, {30, 39}, {33, 104}, {42, 28}, {44, 221},
    {47, 153}, {51, 196}, {65, 118}, {73, 38}, {74, 20}, {77, 143}, {242, 104},
    {253, 83}, {43000, 144}, {43001, 219}, {43002, 219}, {43003, 203}, {43004, 82},
    {43005, 223}, {43006, 214}, {43007, 24}, {43009, 42}, {43010, 48},
  };
  for (size_t i = 0; i < sizeof TABLE / sizeof TABLE[0]; i++) {
    if (TABLE[i].id == id) { *out = TABLE[i].extra; return true; }
  }
  return false;
}

static uint16_t crc16(const uint8_t *d, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; i++) {
    uint8_t t = (uint8_t)(d[i] ^ (uint8_t)(crc & 0xFF));
    t = (uint8_t)(t ^ (uint8_t)(t << 4));
    crc = (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^
                     ((uint16_t)t >> 4));
  }
  return crc;
}

static void inject(uint32_t msgid, uint8_t extra, const uint8_t *payload, uint8_t len) {
  uint8_t f[300];
  while (len > 1 && payload[len - 1] == 0) len--;
  f[0] = 0xFD;
  f[1] = len;
  f[2] = 0;
  f[3] = 0;
  f[4] = 7;
  f[5] = 255;
  f[6] = 190;
  f[7] = (uint8_t)(msgid & 0xFF);
  f[8] = (uint8_t)((msgid >> 8) & 0xFF);
  f[9] = (uint8_t)((msgid >> 16) & 0xFF);
  memcpy(&f[10], payload, len);
  uint16_t c = crc16(&f[1], (size_t)(9 + len), 0xFFFF);
  c = crc16(&extra, 1, c);
  f[10 + len] = (uint8_t)(c & 0xFF);
  f[11 + len] = (uint8_t)(c >> 8);
  ardudeck_receive(f, (size_t)(12 + len));
}

static void put_u16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static void put_f32(uint8_t *p, float v) { memcpy(p, &v, 4); }

static void put_i32(uint8_t *p, int32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* ─── tests ────────────────────────────────────────────────────────────────── */

/**
 * The frame on the wire, byte for byte.
 *
 * Generated by tools/gen_defs.py's verified CRC, not typed. If this one test passes,
 * framing, truncation and the checksum are all right, and every other message rides on
 * the same code.
 */
static void test_golden_heartbeat(void) {
  /* Byte 14 is MAV_TYPE. It must be 11, SURFACE_BOAT, because that is the frame this
     vehicle declared. A generic 0 there loses the boat icon in every ground station. */
  static const uint8_t WANT[] = {
    0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x01, 0x03, 0x03, 0x4B, 0x7F,
  };
  boot();
  ardudeck_tick(clock_ms);
  CHECK(sent_len >= sizeof WANT, "nothing sent");
  CHECK(memcmp(sent, WANT, sizeof WANT) == 0, "heartbeat bytes differ");
}

/**
 * Trailing zeros come off, and the checksum covers what is left.
 *
 * STATUSTEXT declares 51 bytes and carries five characters here, so a frame longer than
 * six proves the truncation is not happening and every strict receiver would still
 * accept it, quietly wasting most of the link.
 */
static void test_statustext_truncates_trailing_zeros(void) {
  boot();
  ardudeck_tick(clock_ms);
  drain();
  ardudeck_notify(AD_INFO, "hello");

  CHECK(sent_len == 18, "frame is %zu bytes, want 18", sent_len);
  CHECK(sent[1] == 6, "payload length %u, want 6", sent[1]);
  CHECK(sent[10] == AD_INFO, "severity %u", sent[10]);
  CHECK(memcmp(&sent[11], "hello", 5) == 0, "text wrong");
}

/**
 * Every frame checks out against a second implementation.
 *
 * The checksum here is written independently of the one in the library, so agreement
 * across a busy run means the framing is right rather than merely self-consistent.
 */
static void test_every_frame_passes_an_independent_checksum(void) {
  boot();
  run_ms(3000);
  ardudeck_notify(AD_WARNING, "check me");

  size_t i = 0;
  int frames = 0;
  while (i + 12 <= sent_len) {
    CHECK(sent[i] == 0xFD, "byte %zu is 0x%02X, not a start of frame", i, sent[i]);
    if (sent[i] != 0xFD) return;

    uint8_t len = sent[i + 1];
    uint32_t id = (uint32_t)sent[i + 7] | ((uint32_t)sent[i + 8] << 8) |
                  ((uint32_t)sent[i + 9] << 16);
    uint8_t extra = 0;
    CHECK(extra_for(id, &extra), "no known extra for message %u", id);

    uint16_t want = crc16(&sent[i + 1], (size_t)(9 + len), 0xFFFF);
    want = crc16(&extra, 1, want);
    uint16_t got = (uint16_t)(sent[i + 10 + len] | ((uint16_t)sent[i + 11 + len] << 8));
    CHECK(want == got, "message %u checksum 0x%04X, want 0x%04X", id, got, want);

    CHECK(len == 1 || sent[i + 10 + len - 1] != 0,
          "message %u ends in a zero byte that should have been trimmed", id);

    i += (size_t)(12 + len);
    frames++;
  }
  CHECK(i == sent_len, "trailing bytes after the last frame");
  CHECK(frames > 15, "only %d frames in three seconds", frames);
}

static void test_manifest_describes_the_vehicle(void) {
  boot();
  run_ms(1000);
  uint8_t len = 0;
  const uint8_t *p = find(43000, &len);
  CHECK(p != NULL, "no manifest");
  if (!p) return;

  CHECK(u16_at(p, 4) == 1, "profile version %u", u16_at(p, 4));
  CHECK(u16_at(p, 6) == 32, "mission capacity %u", u16_at(p, 6));
  CHECK(u16_at(p, 8) == 3, "param count %u", u16_at(p, 8));
  CHECK(p[10] == AD_FRAME_SURFACE_BOAT, "frame %u", p[10]);
  CHECK(p[11] == 3, "mode count %u", p[11]);
  CHECK(p[12] == 2, "mission cmd count %u", p[12]);
  CHECK(p[13] == 2, "cal count %u", p[13]);
  CHECK(memcmp(p + 14, "acme", 4) == 0, "vendor wrong");
}

static void test_every_mode_is_named(void) {
  boot();
  run_ms(1000);
  CHECK(count_of(43001) == 3, "modes sent %d, want 3", count_of(43001));
}

static void test_calibrations_declared_with_poses_and_tracks(void) {
  boot();
  run_ms(2000);
  CHECK(count_of(43005) == 2, "declares %d, want 2", count_of(43005));
  CHECK(count_of(43007) == 2, "tracks %d, want 2", count_of(43007));
  CHECK(count_of(43006) == 2, "poses %d, want 2", count_of(43006));
}

static void test_param_list_streams_values_and_metadata(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t payload[2] = {1, 1};
  inject(21, 159, payload, 2); /* PARAM_REQUEST_LIST */
  run_ms(600);

  CHECK(count_of(22) == 3, "PARAM_VALUE %d, want 3", count_of(22));
  CHECK(count_of(43003) == 3, "PARAM_META %d, want 3", count_of(43003));
  CHECK(count_of(43004) == 4, "PARAM_OPTION %d, want 4", count_of(43004));
}

static void test_param_metadata_carries_unit_and_range(void) {
  boot();
  run_ms(100);
  drain();
  uint8_t payload[2] = {1, 1};
  inject(21, 159, payload, 2);
  run_ms(600);

  uint8_t len = 0;
  const uint8_t *p = find(43003, &len);
  CHECK(p != NULL, "no metadata");
  if (!p) return;
  CHECK(f32_at(p, 0) == 0.2f, "min %f", (double)f32_at(p, 0));
  CHECK(f32_at(p, 4) == 4.0f, "max %f", (double)f32_at(p, 4));
  CHECK(memcmp(p + 16, "CRUISE_SPD", 10) == 0, "name wrong");
  CHECK(memcmp(p + 32, "m/s", 3) == 0, "unit wrong");
}

static void test_param_set_clamps_and_reports_back(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t payload[23];
  memset(payload, 0, sizeof payload);
  put_f32(payload, 99.0f); /* far above the 4.0 ceiling */
  payload[4] = 1;
  payload[5] = 1;
  memcpy(&payload[6], "CRUISE_SPD", 10);
  payload[22] = 9; /* REAL32 */
  inject(23, 168, payload, sizeof payload);

  CHECK(cruise == 4.0f, "clamped to %f, want 4.0", (double)cruise);
  CHECK(param_saves == 1, "saves %d, want 1", param_saves);

  uint8_t len = 0;
  const uint8_t *p = find(22, &len);
  CHECK(p != NULL, "no PARAM_VALUE answer");
  if (p) CHECK(f32_at(p, 0) == 4.0f, "reported %f", (double)f32_at(p, 0));
}

static void test_unknown_param_is_ignored_not_answered(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t payload[23];
  memset(payload, 0, sizeof payload);
  put_f32(payload, 1.0f);
  memcpy(&payload[6], "NOPE", 4);
  inject(23, 168, payload, sizeof payload);
  CHECK(count_of(22) == 0, "answered a parameter that does not exist");
}

static void test_mission_upload_round_trip(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t count[4];
  put_u16(count, 2);
  count[2] = 1;
  count[3] = 1;
  inject(44, 221, count, 4); /* MISSION_COUNT */
  CHECK(count_of(51) == 1, "no MISSION_REQUEST_INT for item 0");

  for (uint16_t seq = 0; seq < 2; seq++) {
    uint8_t item[37];
    memset(item, 0, sizeof item);
    put_i32(&item[16], (int32_t)(52.5e7));
    put_i32(&item[20], (int32_t)(13.4e7));
    put_f32(&item[24], 30.0f);
    put_u16(&item[28], seq);
    put_u16(&item[30], AD_NAV_WAYPOINT);
    item[34] = 3; /* GLOBAL_RELATIVE_ALT */
    inject(73, 38, item, sizeof item);
  }

  CHECK(last_mission_count == 2, "delivered %u items", last_mission_count);
  uint8_t ack[4];
  CHECK(payload_of(47, ack, sizeof ack), "no MISSION_ACK");
  CHECK(ack[2] == 0, "ack result %u, want 0", ack[2]);
}

static void test_mission_rejects_a_command_not_declared(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t count[4];
  put_u16(count, 1);
  inject(44, 221, count, 4);

  uint8_t item[37];
  memset(item, 0, sizeof item);
  put_u16(&item[28], 0);
  put_u16(&item[30], AD_NAV_TAKEOFF); /* not in MISSION_CMDS */
  item[34] = 3;
  inject(73, 38, item, sizeof item);

  uint8_t ack[4];
  CHECK(payload_of(47, ack, sizeof ack), "no MISSION_ACK");
  CHECK(ack[2] == 3, "ack result %u, want 3 UNSUPPORTED", ack[2]);
  CHECK(last_mission_count == 0xFFFF, "delivered a plan it should have refused");
}

static void test_mission_refusal_carries_the_firmware_words(void) {
  boot();
  run_ms(100);
  drain();
  refuse_mission = true;

  uint8_t count[4];
  put_u16(count, 0);
  inject(44, 221, count, 4);

  uint8_t len = 0;
  const uint8_t *text = find(253, &len);
  CHECK(text != NULL, "no STATUSTEXT with the reason");
  if (text) CHECK(memcmp(text + 1, "No GPS fix", 10) == 0, "wrong text");
}

static void test_mission_too_big_is_refused_before_transfer(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t count[4];
  put_u16(count, 999);
  inject(44, 221, count, 4);

  uint8_t ack[4];
  CHECK(payload_of(47, ack, sizeof ack), "no MISSION_ACK");
  CHECK(ack[2] == 4, "ack result %u, want 4 NO_SPACE", ack[2]);
  CHECK(count_of(51) == 0, "started a transfer it had no room for");
}

static void test_mission_download(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t req[2] = {1, 1};
  inject(43, 132, req, 2); /* MISSION_REQUEST_LIST */

  uint8_t len = 0;
  const uint8_t *c = find(44, &len);
  CHECK(c != NULL, "no MISSION_COUNT");
  if (c) CHECK(u16_at(c, 0) == 2, "count %u", u16_at(c, 0));

  drain();
  uint8_t item_req[4];
  put_u16(item_req, 1);
  inject(51, 196, item_req, 4);
  const uint8_t *item = find(73, &len);
  CHECK(item != NULL, "no MISSION_ITEM_INT");
  if (item) CHECK(u16_at(item, 28) == 1, "seq %u", u16_at(item, 28));
}

static void test_command_acknowledged_and_refusals_explained(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t cmd[33];
  memset(cmd, 0, sizeof cmd);
  put_f32(cmd, 1.0f);
  put_u16(&cmd[28], AD_CMD_ARM);
  inject(76, 152, cmd, sizeof cmd);

  CHECK(last_command == AD_CMD_ARM, "command %u", last_command);
  uint8_t ack[4];
  CHECK(payload_of(77, ack, sizeof ack), "no COMMAND_ACK");
  CHECK(ack[2] == 0, "result %u, want 0", ack[2]);

  drain();
  refuse_command = true;
  inject(76, 152, cmd, sizeof cmd);
  CHECK(payload_of(77, ack, sizeof ack), "no COMMAND_ACK on refusal");
  CHECK(ack[2] == 2, "result %u, want 2 DENIED", ack[2]);
  uint8_t len = 0;
  CHECK(find(253, &len) != NULL, "refusal carried no words");
}

static void test_calibration_start_reaches_the_firmware(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t ctl[19];
  memset(ctl, 0, sizeof ctl);
  ctl[2] = AD_CAL_START;
  memcpy(&ctl[3], "compass", 7);
  inject(43008, 117, ctl, sizeof ctl);

  CHECK(strcmp(last_cal, "compass") == 0, "cal id %s", last_cal);
  CHECK(last_action == AD_CAL_START, "action %d", (int)last_action);
}

static void test_calibration_refused_while_armed(void) {
  boot();
  run_ms(100);
  ardudeck_status(1, 0, 12.4f, true);
  drain();

  uint8_t ctl[19];
  memset(ctl, 0, sizeof ctl);
  ctl[2] = AD_CAL_START;
  memcpy(&ctl[3], "compass", 7);
  inject(43008, 117, ctl, sizeof ctl);

  CHECK(last_cal[0] == '\0', "the vehicle let a disarmed-only calibration start armed");
  uint8_t len = 0;
  CHECK(find(253, &len) != NULL, "refused silently");
}

static void test_calibration_progress_is_throttled(void) {
  boot();
  run_ms(100);
  drain();

  ad_cal_progress_t p;
  memset(&p, 0, sizeof p);
  p.percent = 50;
  for (int i = 0; i < 20; i++) ardudeck_cal_progress("compass", &p);
  CHECK(count_of(43009) == 1, "sent %d progress frames in one instant",
        count_of(43009));

  /* A captured pose is never throttled: it is the one the operator is waiting on. */
  drain();
  p.step_done = true;
  for (int i = 0; i < 3; i++) ardudeck_cal_progress("compass", &p);
  CHECK(count_of(43009) == 3, "throttled a pose capture");
}

static void test_no_fix_means_no_marker(void) {
  boot();
  ardudeck_position(0.0, 0.0, 0.0f, 0.0f, 0, 0);
  run_ms(400);
  CHECK(count_of(33) == 0, "put a position on the map with no fix");
  CHECK(count_of(24) > 0, "stopped reporting GPS state entirely");

  drain();
  ardudeck_position(52.5, 13.4, 1.2f, 90.0f, 3, 9);
  run_ms(400);
  CHECK(count_of(33) > 0, "withheld a position it actually had");
}

static void test_unknown_battery_is_not_flat(void) {
  boot();
  run_ms(1200);
  uint8_t len = 0;
  const uint8_t *p = find(1, &len);
  CHECK(p != NULL, "no SYS_STATUS");
  if (!p) return;
  CHECK(u16_at(p, 14) == 0xFFFF, "voltage %u, want 0xFFFF unknown", u16_at(p, 14));
  CHECK((int8_t)p[30] == -1, "battery percent %d, want -1", (int8_t)p[30]);
}

static void test_a_split_frame_still_arrives(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t payload[2] = {1, 1};
  /* Same frame as the streaming test, fed one byte at a time. */
  uint8_t f[14];
  f[0] = 0xFD;
  f[1] = 2;
  f[2] = 0;
  f[3] = 0;
  f[4] = 1;
  f[5] = 255;
  f[6] = 190;
  f[7] = 21;
  f[8] = 0;
  f[9] = 0;
  memcpy(&f[10], payload, 2);
  uint16_t c = crc16(&f[1], 11, 0xFFFF);
  uint8_t extra = 159;
  c = crc16(&extra, 1, c);
  f[12] = (uint8_t)(c & 0xFF);
  f[13] = (uint8_t)(c >> 8);

  for (size_t i = 0; i < sizeof f; i++) ardudeck_receive(&f[i], 1);
  run_ms(200);
  CHECK(count_of(22) > 0, "a frame split into single bytes was lost");
}

static void test_a_corrupt_frame_is_dropped(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t count[4];
  put_u16(count, 2);
  uint8_t f[16];
  f[0] = 0xFD;
  f[1] = 4;
  f[2] = 0;
  f[3] = 0;
  f[4] = 1;
  f[5] = 255;
  f[6] = 190;
  f[7] = 44;
  f[8] = 0;
  f[9] = 0;
  memcpy(&f[10], count, 4);
  f[14] = 0x00; /* wrong checksum */
  f[15] = 0x00;
  ardudeck_receive(f, sizeof f);
  CHECK(count_of(51) == 0, "acted on a frame with a bad checksum");
}

static void test_clock_rollover_keeps_sending(void) {
  clock_ms = 0xFFFFF000u;
  boot_at(0xFFFFF000u);
  drain();

  for (int i = 0; i < 400; i++) {
    clock_ms += 20;
    ardudeck_tick(clock_ms);
  }
  CHECK(count_of(0) >= 6, "heartbeats across the wrap: %d", count_of(0));
}

static void test_param_changed_reannounces(void) {
  boot();
  run_ms(100);
  drain();
  cruise = 3.0f;
  ardudeck_param_changed("CRUISE_SPD");
  uint8_t len = 0;
  const uint8_t *p = find(22, &len);
  CHECK(p != NULL, "no PARAM_VALUE");
  if (p) CHECK(f32_at(p, 0) == 3.0f, "announced %f", (double)f32_at(p, 0));
}

static void test_rate_limit_is_honoured(void) {
  boot();
  ardudeck_rate_limit(AD_STREAM_ATTITUDE, 1.0f);
  ardudeck_attitude(0.1f, 0.2f, 0.3f);
  run_ms(2000);
  int n = count_of(30);
  CHECK(n >= 1 && n <= 3, "attitude frames in 2 s at 1 Hz: %d", n);
}

/*
 * The exact bytes ArduDeck puts on the wire, taken from its own send path rather than
 * from the MAVLink spec. The two disagree, and the ground station is what a vendor
 * actually has to work with.
 */
static void send_command_long(uint16_t cmd, const float p[7]) {
  uint8_t buf[33];
  memset(buf, 0, sizeof buf);
  for (int i = 0; i < 7; i++) put_f32(&buf[i * 4], p[i]);
  put_u16(&buf[28], cmd);
  inject(76, 152, buf, sizeof buf);
}

static void test_set_mode_reads_the_slot_ardudeck_uses(void) {
  boot();
  run_ms(100);
  drain();

  /* ipc-handlers.ts: param1 = 1 | armedBit, param2 = customMode. */
  float p[7] = {1.0f, 4.0f, 0, 0, 0, 0, 0};
  send_command_long(AD_CMD_SET_MODE, p);

  CHECK(last_command == AD_CMD_SET_MODE, "command %u", last_command);
  CHECK(last_args[0] == 4.0f, "mode arrived as %f, want 4 in a[0]", (double)last_args[0]);
}

static void test_set_mode_is_ignored_without_the_custom_bit(void) {
  boot();
  run_ms(100);
  drain();

  /* No MAV_MODE_FLAG_CUSTOM_MODE_ENABLED means param2 is not a mode at all. */
  float p[7] = {0.0f, 4.0f, 0, 0, 0, 0, 0};
  send_command_long(AD_CMD_SET_MODE, p);
  CHECK(last_command == 0, "acted on a mode change that was not one");

  uint8_t ack[4];
  CHECK(payload_of(77, ack, sizeof ack), "did not answer at all");
}

static void test_takeoff_altitude_comes_from_param7(void) {
  boot();
  run_ms(100);
  drain();

  float p[7] = {0, 0, 0, 0, 0, 0, 25.0f};
  send_command_long(AD_CMD_TAKEOFF, p);

  CHECK(last_command == AD_CMD_TAKEOFF, "command %u", last_command);
  CHECK(last_args[0] == 25.0f, "altitude arrived as %f, want 25 in a[0]",
        (double)last_args[0]);
}

static void test_arm_arguments_are_left_alone(void) {
  boot();
  run_ms(100);
  drain();

  float p[7] = {1.0f, 21196.0f, 0, 0, 0, 0, 0};
  send_command_long(AD_CMD_ARM, p);
  CHECK(last_args[0] == 1.0f, "arm flag %f", (double)last_args[0]);
  CHECK(last_args[1] == 21196.0f, "force flag %f", (double)last_args[1]);
}

/**
 * ArduDeck sends DO_SET_MODE and then the legacy SET_MODE message as a fallback for
 * older ArduPilot builds. Acting on both would switch mode twice per press.
 */
static void test_legacy_set_mode_does_not_double_fire(void) {
  boot();
  run_ms(100);
  drain();

  float p[7] = {1.0f, 4.0f, 0, 0, 0, 0, 0};
  send_command_long(AD_CMD_SET_MODE, p);
  int after_command = command_calls;

  uint8_t sm[6];
  memset(sm, 0, sizeof sm);
  sm[0] = 4;     /* custom_mode */
  sm[4] = 1;     /* target_system */
  sm[5] = 1;     /* base_mode, custom enabled */
  inject(11, 89, sm, sizeof sm);

  CHECK(command_calls == after_command,
        "the legacy follow-up fired the callback again (%d then %d)",
        after_command, command_calls);
}

static void test_legacy_set_mode_works_on_its_own(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t sm[6];
  memset(sm, 0, sizeof sm);
  sm[0] = 1; /* a mode the fixture declares; an undeclared one is refused, see below */
  sm[4] = 1;
  sm[5] = 1;
  inject(11, 89, sm, sizeof sm);

  CHECK(last_command == AD_CMD_SET_MODE, "a ground station that sends only SET_MODE "
        "was ignored (command %u)", last_command);
  CHECK(last_args[0] == 1.0f, "mode %f", (double)last_args[0]);
}

/**
 * A ground station that does not read the declared mode table guesses, and the usual
 * guess is an ArduPilot mode number for the frame type. ArduDeck did exactly this before
 * the mission mode flag existed: a boat got mode 10, Rover's AUTO. Acting on it would
 * move the vehicle into whatever this firmware numbers 10, so it is refused instead.
 */
static void test_an_undeclared_mode_is_refused(void) {
  boot();
  run_ms(100);
  drain();

  last_command = 0;
  /* param1 = 1 says the custom mode is meaningful; param2 = 10 is Rover's AUTO. */
  float p[7] = {1.0f, 10.0f, 0, 0, 0, 0, 0};
  send_command_long(AD_CMD_SET_MODE, p);

  CHECK(last_command == 0, "an undeclared mode reached the firmware (command %u)",
        last_command);
  uint8_t ack[4];
  CHECK(payload_of(77, ack, sizeof ack), "no COMMAND_ACK for an undeclared mode");
  CHECK(ack[2] == 2, "result %u, want 2 DENIED", ack[2]);
}

/**
 * A ground station watching a mission sends its heartbeat and very little else.
 *
 * The regression: HEARTBEAT was missing from the SDK's inbound table, so those frames
 * were discarded before anything recorded that somebody was there. `ardudeck_silent_for`
 * kept answering "never heard" on a perfectly healthy link, and a firmware built on the
 * failsafe advice in the docs would turn for home a few seconds into a mission flown
 * from a laptop.
 */
static void test_a_ground_station_heartbeat_counts_as_contact(void) {
  boot();
  run_ms(100);
  drain();

  CHECK(ardudeck_silent_for(now()) == AD_NEVER_HEARD,
        "heard something before anything was sent");

  uint8_t hb[9];
  memset(hb, 0, sizeof hb);
  hb[4] = 6; /* MAV_TYPE_GCS, what ArduDeck puts in its own heartbeat */
  hb[5] = 8; /* MAV_AUTOPILOT_INVALID */
  inject(0, 50, hb, sizeof hb);

  CHECK(ardudeck_silent_for(now()) != AD_NEVER_HEARD,
        "a ground station heartbeat did not count as contact");
  CHECK(ardudeck_silent_for(now()) < 100, "silent for %u ms after a heartbeat",
        ardudeck_silent_for(now()));

  /* And it keeps the link alive on its own, with no other traffic at all. */
  run_ms(1000);
  uint8_t hb2[9];
  memset(hb2, 0, sizeof hb2);
  hb2[4] = 6;
  hb2[5] = 8;
  inject(0, 50, hb2, sizeof hb2);
  CHECK(ardudeck_linked(), "a link fed only by heartbeats was reported as lost");
}

/**
 * On a broadcast link every vehicle is heartbeating too. Treating one of those as the
 * ground station would aim telemetry at another vehicle and hold the failsafe open with
 * nobody watching, which is worse than the bug above.
 */
static void test_another_vehicles_heartbeat_is_not_contact(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t hb[9];
  memset(hb, 0, sizeof hb);
  hb[4] = 2; /* MAV_TYPE_QUADROTOR: another vehicle, not a ground station */
  hb[5] = 3; /* MAV_AUTOPILOT_ARDUPILOTMEGA */
  inject(0, 50, hb, sizeof hb);

  CHECK(ardudeck_silent_for(now()) == AD_NEVER_HEARD,
        "another vehicle's heartbeat was mistaken for a ground station");
}

/** The same guard on the legacy path, which has no acknowledgement to carry a refusal. */
static void test_an_undeclared_legacy_mode_is_refused(void) {
  boot();
  run_ms(100);
  drain();

  last_command = 0;
  uint8_t sm[6];
  memset(sm, 0, sizeof sm);
  sm[0] = 10;
  sm[4] = 1;
  sm[5] = 1;
  inject(11, 89, sm, sizeof sm);

  CHECK(last_command == 0, "an undeclared legacy mode reached the firmware (command %u)",
        last_command);
}

/**
 * A geofence and a rally point set ride the same mission protocol, told apart only by
 * mission_type. Accepting one as a flight plan would make the vehicle fly the boundary
 * it was told to stay inside.
 */
static void test_a_fence_upload_is_not_taken_as_a_mission(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t count[5];
  memset(count, 0, sizeof count);
  put_u16(count, 4);
  count[2] = 1;
  count[3] = 1;
  count[4] = 1; /* MAV_MISSION_TYPE_FENCE */
  inject(44, 221, count, sizeof count);

  CHECK(count_of(51) == 0, "started a transfer for a fence upload");
  CHECK(last_mission_count == 0xFFFF, "delivered a fence as a flight plan");

  uint8_t ack[5];
  CHECK(payload_of(47, ack, sizeof ack), "no answer to the fence upload");
  CHECK(ack[2] == 3, "ack result %u, want 3 UNSUPPORTED", ack[2]);
  CHECK(ack[3] == 1, "ack mission_type %u, want 1 so the sender routes it to its fence "
        "transfer instead of believing a mission was refused", ack[3]);
}

static void test_a_rally_request_is_refused_with_its_own_type(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t req[3] = {1, 1, 2}; /* MAV_MISSION_TYPE_RALLY */
  inject(43, 132, req, sizeof req);

  uint8_t ack[5];
  CHECK(payload_of(47, ack, sizeof ack), "no answer to the rally request");
  CHECK(ack[3] == 2, "ack mission_type %u, want 2", ack[3]);
  CHECK(count_of(44) == 0, "answered a rally request with a mission count");
}

/**
 * Carrying mission_type must not change the ordinary case.
 *
 * An accepted mission ack has zeros in both trailing fields, so v2 trims it to two bytes
 * exactly as before, and a receiver pads them back to ACCEPTED and MISSION.
 */
static void test_an_ordinary_mission_ack_is_unchanged(void) {
  boot();
  run_ms(100);
  drain();

  uint8_t count[5];
  memset(count, 0, sizeof count);
  put_u16(count, 0);
  inject(44, 221, count, sizeof count);

  uint8_t len = 0;
  CHECK(find(47, &len) != NULL, "no MISSION_ACK");
  CHECK(len <= 3, "ack grew to %u bytes for an ordinary mission", len);

  uint8_t ack[5];
  payload_of(47, ack, sizeof ack);
  CHECK(ack[2] == 0, "result %u, want 0 ACCEPTED after padding", ack[2]);
  CHECK(ack[3] == 0, "mission_type %u, want 0 MISSION after padding", ack[3]);
}

/**
 * A link failsafe needs to know when the ground station went quiet.
 *
 * The vehicle cannot use "a ground station exists" as proof the link is alive: a laptop
 * can be closed without saying anything. Reporting the silence is the only honest answer.
 */
static void test_silence_is_reported_so_a_failsafe_can_work(void) {
  boot();
  CHECK(ardudeck_silent_for(clock_ms) == AD_NEVER_HEARD,
        "claimed to have heard something before anything arrived");
  CHECK(!ardudeck_linked(), "linked before any ground station spoke");

  run_ms(100);
  uint8_t req[2] = {1, 1};
  inject(21, 159, req, 2);

  CHECK(ardudeck_silent_for(clock_ms) < 100, "silence %u ms just after hearing one",
        ardudeck_silent_for(clock_ms));
  CHECK(ardudeck_linked(), "not linked immediately after hearing a ground station");

  run_ms(6000);
  CHECK(ardudeck_silent_for(clock_ms) >= 5000, "silence %u ms after six quiet seconds",
        ardudeck_silent_for(clock_ms));
  CHECK(!ardudeck_linked(), "still reports linked six seconds after the last word");
}

static void test_the_heartbeat_carries_the_declared_frame(void) {
  boot();
  run_ms(1200);
  uint8_t hb[9];
  CHECK(payload_of(0, hb, sizeof hb), "no heartbeat");
  CHECK(hb[4] == 11, "MAV_TYPE %u, want 11 SURFACE_BOAT", hb[4]);
  CHECK(hb[5] == 0, "autopilot %u, want 0 GENERIC", hb[5]);
}

/* ─── driver ───────────────────────────────────────────────────────────────── */

#define RUN(t)                                                                         \
  do {                                                                                 \
    int before = failures;                                                             \
    t();                                                                               \
    printf("%-4s %s\n", failures == before ? "ok" : "FAIL", #t);                       \
  } while (0)

int main(void) {
  RUN(test_golden_heartbeat);
  RUN(test_the_heartbeat_carries_the_declared_frame);
  RUN(test_silence_is_reported_so_a_failsafe_can_work);
  RUN(test_statustext_truncates_trailing_zeros);
  RUN(test_every_frame_passes_an_independent_checksum);
  RUN(test_manifest_describes_the_vehicle);
  RUN(test_every_mode_is_named);
  RUN(test_calibrations_declared_with_poses_and_tracks);
  RUN(test_param_list_streams_values_and_metadata);
  RUN(test_param_metadata_carries_unit_and_range);
  RUN(test_param_set_clamps_and_reports_back);
  RUN(test_unknown_param_is_ignored_not_answered);
  RUN(test_param_changed_reannounces);
  RUN(test_mission_upload_round_trip);
  RUN(test_mission_rejects_a_command_not_declared);
  RUN(test_mission_refusal_carries_the_firmware_words);
  RUN(test_mission_too_big_is_refused_before_transfer);
  RUN(test_a_fence_upload_is_not_taken_as_a_mission);
  RUN(test_a_rally_request_is_refused_with_its_own_type);
  RUN(test_an_ordinary_mission_ack_is_unchanged);
  RUN(test_mission_download);
  RUN(test_command_acknowledged_and_refusals_explained);
  RUN(test_set_mode_reads_the_slot_ardudeck_uses);
  RUN(test_set_mode_is_ignored_without_the_custom_bit);
  RUN(test_takeoff_altitude_comes_from_param7);
  RUN(test_arm_arguments_are_left_alone);
  RUN(test_legacy_set_mode_does_not_double_fire);
  RUN(test_legacy_set_mode_works_on_its_own);
  RUN(test_an_undeclared_mode_is_refused);
  RUN(test_an_undeclared_legacy_mode_is_refused);
  RUN(test_a_ground_station_heartbeat_counts_as_contact);
  RUN(test_another_vehicles_heartbeat_is_not_contact);
  RUN(test_calibration_start_reaches_the_firmware);
  RUN(test_calibration_refused_while_armed);
  RUN(test_calibration_progress_is_throttled);
  RUN(test_no_fix_means_no_marker);
  RUN(test_unknown_battery_is_not_flat);
  RUN(test_a_split_frame_still_arrives);
  RUN(test_a_corrupt_frame_is_dropped);
  RUN(test_clock_rollover_keeps_sending);
  RUN(test_rate_limit_is_honoured);

  printf("\n%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
