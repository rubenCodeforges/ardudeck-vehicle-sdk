/*
 * A vehicle built on the SDK, on a real UDP socket.
 *
 * This is what the conformance tool is pointed at in CI. It is also how a flaw gets
 * proven: start it with --break <thing>, and the corresponding check must go red.
 * A conformance test nobody has watched fail is not a test.
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "ardudeck.h"

static int sock = -1;
static struct sockaddr_in peer;
static bool have_peer = false;
static int bcast_port = 14550;

static bool break_units = false;
static bool break_modes = false;
static bool break_no_mission_cmds = false;
static bool break_zero_island = false;
static bool break_silent_command = false;

static uint32_t now_ms(void) {
  static struct timeval start;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (start.tv_sec == 0) start = tv;
  return (uint32_t)((tv.tv_sec - start.tv_sec) * 1000 +
                    (tv.tv_usec - start.tv_usec) / 1000);
}

static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  struct sockaddr_in to;
  if (have_peer) {
    to = peer;
  } else {
    memset(&to, 0, sizeof to);
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    to.sin_port = htons((uint16_t)bcast_port);
  }
  sendto(sock, buf, len, 0, (struct sockaddr *)&to, sizeof to);
}

/* ─── the vehicle ──────────────────────────────────────────────────────────── */

static float cruise = 1.5f;
static float arrive = 3.0f;
static float hdg_kp = 1.2f;
static int32_t link_timeout = 20;
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

static const ad_param_t PARAMS_OK[] = {
  AD_F32("CRUISE_SPD", &cruise, 0.2f, 4.0f, "m/s", "Speed held between waypoints"),
  AD_F32("ARRIVE_RAD", &arrive, 0.5f, 20.0f, "m", "How close counts as arriving"),
  AD_F32("HDG_KP", &hdg_kp, 0.0f, 5.0f, "", "Steering gain"),
  AD_I32("LINK_TIMEOUT", &link_timeout, 1, 120, "s", "Silence before the failsafe"),
  AD_ENUM("FS_ACTION", fs_get, fs_set, FS_OPTIONS, 4, "What to do when the link fails"),
};

/* Same table with the descriptions stripped, which rung 2 must catch. */
static const ad_param_t PARAMS_BROKEN[] = {
  AD_F32("CRUISE_SPD", &cruise, 0.2f, 4.0f, "m/s", ""),
  AD_F32("ARRIVE_RAD", &arrive, 0.5f, 20.0f, "m", ""),
  AD_F32("HDG_KP", &hdg_kp, 0.0f, 5.0f, "", ""),
  AD_I32("LINK_TIMEOUT", &link_timeout, 1, 120, "s", "Silence before the failsafe"),
  AD_ENUM("FS_ACTION", fs_get, fs_set, FS_OPTIONS, 4, "What to do when the link fails"),
};

static const uint16_t MISSION_CMDS[] = {AD_NAV_WAYPOINT, AD_NAV_RETURN_TO_LAUNCH};

static const ad_mode_t MODES_OK[] = {
  {0, "Idle", 0}, {1, "Running", 0}, {2, "Holding", 0}, {3, "Returning", 0},
};

static const ad_mode_t MODES_BROKEN[] = {
  {0, "Idle", 0}, {1, "", 0}, {2, "Holding", 0}, {3, "Returning", 0},
};

static const ad_cal_track_t COMPASS_TRACKS[] = {
  {"Sectors turned through", "", 12.0f},
  {"Total turning", "deg", 720.0f},
};

static const ad_cal_pose_t ACCEL_POSES[] = {
  {"Level", 0.0f, 0.0f}, {"Left side", -90.0f, 0.0f}, {"Nose down", 0.0f, -90.0f},
};

static const ad_calibration_t CALS[] = {
  {
    .id = "compass", .name = "Compass", .kind = AD_CAL_COVERAGE,
    .requirements = AD_CAL_REQ_DISARMED,
    .warning = "Turn the vehicle through two full circles by hand.",
    .tracks = COMPASS_TRACKS, .track_count = 2,
  },
  {
    .id = "accel", .name = "Accelerometer", .kind = AD_CAL_POSITIONAL,
    .requirements = AD_CAL_REQ_DISARMED | AD_CAL_REQ_STATIONARY,
    .poses = ACCEL_POSES, .pose_count = 3,
  },
  {
    .id = "compassmot", .name = "Compass interference", .kind = AD_CAL_SWEEP,
    .requirements = AD_CAL_REQ_MOTORS_LIVE | AD_CAL_REQ_PROPS_OFF,
    .warning = "Motors will spin. Remove the propellers and tie the airframe down.",
    .prompt = "Raise the throttle slowly to full over ten seconds.",
  },
};

static ad_capability_t CAPS = {
  .vendor = "acme", .model = "test rover", .firmware = "1.0.0",
  .frame = AD_FRAME_ROVER,
  .features = AD_FEAT_PARAMS | AD_FEAT_MISSION | AD_FEAT_MISSION_READ |
              AD_FEAT_COMMANDS | AD_FEAT_CALIBRATION,
  .mission_cmds = MISSION_CMDS, .mission_cmd_count = 2, .mission_capacity = 16,
  .modes = MODES_OK, .mode_count = 4,
};

#define STORE 32
static ad_wp_t stored[STORE];
static uint16_t stored_count;

static bool on_param_set(const ad_param_t *p, float value, void *user) {
  (void)p; (void)value; (void)user;
  return true;
}

static bool on_mission(const ad_wp_t *items, uint16_t count, char *why, size_t why_len,
                       void *user) {
  (void)why; (void)why_len; (void)user;
  if (count > STORE) return false;
  memcpy(stored, items, sizeof(ad_wp_t) * count);
  stored_count = count;
  return true;
}

static uint16_t on_mission_read(ad_wp_t *items, uint16_t capacity, void *user) {
  (void)user;
  if (capacity < stored_count) return 0;
  memcpy(items, stored, sizeof(ad_wp_t) * stored_count);
  return stored_count;
}

static bool on_command(uint16_t cmd, const float args[7], char *why, size_t why_len,
                       void *user) {
  (void)args; (void)user;
  if (break_silent_command) return true; /* accepts anything, including nonsense */
  switch (cmd) {
    case AD_CMD_ARM:
    case AD_CMD_SET_MODE:
    case AD_CMD_START_MISSION:
    case AD_CMD_TAKEOFF:
    case AD_CMD_RETURN_HOME:
      return true;
    default:
      snprintf(why, why_len, "This vehicle does not do that");
      return false;
  }
}

static bool on_calibrate(const char *cal_id, ad_cal_action_t action, char *why,
                         size_t why_len, void *user) {
  (void)cal_id; (void)action; (void)why; (void)why_len; (void)user;
  return true;
}

int main(int argc, char **argv) {
  int port = 14550;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--break") && i + 1 < argc) {
      const char *what = argv[++i];
      if (!strcmp(what, "units")) break_units = true;
      else if (!strcmp(what, "modes")) break_modes = true;
      else if (!strcmp(what, "no-mission-cmds")) break_no_mission_cmds = true;
      else if (!strcmp(what, "zero-island")) break_zero_island = true;
      else if (!strcmp(what, "silent-command")) break_silent_command = true;
      else {
        fprintf(stderr, "unknown --break '%s'\n", what);
        return 2;
      }
    } else {
      fprintf(stderr,
              "usage: fake_vehicle [--port N] [--break units|modes|no-mission-cmds|"
              "zero-island|silent-command]\n");
      return 2;
    }
  }

  bcast_port = port;
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { perror("socket"); return 1; }
  int on = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);

  struct sockaddr_in me;
  memset(&me, 0, sizeof me);
  me.sin_family = AF_INET;
  me.sin_addr.s_addr = htonl(INADDR_ANY);
  me.sin_port = htons((uint16_t)(port + 1));
  if (bind(sock, (struct sockaddr *)&me, sizeof me) < 0) { perror("bind"); return 1; }

  if (break_modes) { CAPS.modes = MODES_BROKEN; }
  /* Declares missions, lists nothing it will fly. Nothing can be planned. */
  if (break_no_mission_cmds) { CAPS.mission_cmd_count = 0; }

  static ad_config_t cfg;
  memset(&cfg, 0, sizeof cfg);
  cfg.caps = &CAPS;
  cfg.params = break_units ? PARAMS_BROKEN : PARAMS_OK;
  cfg.param_count = 5;
  cfg.calibrations = CALS;
  cfg.calibration_count = 3;
  cfg.send = sink;
  cfg.now_ms = now_ms;
  cfg.on_param_set = on_param_set;
  cfg.on_mission = on_mission;
  cfg.on_mission_read = on_mission_read;
  cfg.on_command = on_command;
  cfg.on_calibrate = on_calibrate;
  ardudeck_begin(&cfg);

  uint8_t buf[1024];
  for (;;) {
    struct sockaddr_in from;
    socklen_t from_len = sizeof from;

    fd_set rd;
    FD_ZERO(&rd);
    FD_SET(sock, &rd);
    struct timeval tv = {0, 20000};
    if (select(sock + 1, &rd, NULL, NULL, &tv) > 0) {
      ssize_t n = recvfrom(sock, buf, sizeof buf, 0, (struct sockaddr *)&from,
                           &from_len);
      if (n > 0) {
        peer = from;
        have_peer = true;
        ardudeck_receive(buf, (size_t)n);
      }
    }

    if (break_zero_island) {
      /* Claims a fix while sitting on the null island, which rung 0 must catch. */
      ardudeck_position(0.0, 0.0, 0.0f, 0.0f, 3, 11);
    } else {
      ardudeck_position(52.5163, 13.3777, 1.4f, 92.0f, 3, 11);
    }
    ardudeck_altitude(38.0f, 0.5f, 0.0f);
    ardudeck_attitude(0.02f, -0.01f, 1.6f);
    ardudeck_status(1, 0, 12.4f, false);
    ardudeck_battery(12.4f, 3.1f, 78);
    ardudeck_home(52.5163, 13.3777, 38.0f);
    ardudeck_tick(now_ms());
  }
}
