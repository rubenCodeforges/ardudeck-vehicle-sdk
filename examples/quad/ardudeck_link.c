/*
 * The integration. Everything ArduDeck needs, in one file that reaches the firmware only
 * through vehicle.h.
 *
 * Read it next to vehicle.c: that file did not change by a line to get a map, an
 * instrument panel with a live horizon, a parameter editor, mission upload, a command bar
 * and a six point accelerometer routine.
 */
#include "ardudeck_link.h"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "ardudeck.h"
#include "vehicle.h"

/* ─── 1. Who this vehicle is ───────────────────────────────────────────────── */

static const uint16_t MISSION_CMDS[] = {
  AD_NAV_WAYPOINT, AD_NAV_TAKEOFF, AD_NAV_LAND, AD_NAV_RETURN_TO_LAUNCH,
};

static const ad_mode_t MODES[] = {
  {QUAD_STABILIZE, "Stabilize", 0},
  {QUAD_ALTHOLD,   "AltHold",   0},
  {QUAD_LOITER,    "Loiter",    0},
  {QUAD_AUTO,      "Auto",      AD_MODE_MISSION},   /* the mode that flies a plan */
  {QUAD_RTL,       "RTL",       0},
  /* Landing is worth entering deliberately, but only once it is already up. */
  {QUAD_LAND,      "Land",      AD_MODE_ARMED_ONLY},
  /* Entered by the failsafe, never chosen from a picker. */
  {QUAD_FAILSAFE,  "Failsafe",  AD_MODE_TERMINAL},
};

static const ad_capability_t CAPS = {
  .vendor = "acme",
  .model = "X500 quad",
  .firmware = "2.1.0",
  .frame = AD_FRAME_MULTIROTOR,
  .features = AD_FEAT_PARAMS | AD_FEAT_MISSION | AD_FEAT_COMMANDS |
              AD_FEAT_CALIBRATION | AD_FEAT_HOME,
  .mission_cmds = MISSION_CMDS,
  .mission_cmd_count = AD_COUNT(MISSION_CMDS),
  .mission_capacity = QUAD_MAX_WAYPOINTS,
  .modes = MODES,
  .mode_count = AD_COUNT(MODES),
};

/* ─── 2. Parameters, including one that is a choice ────────────────────────── */

/*
 * A dropdown is a parameter whose value is an index into these labels. It needs a getter
 * and a setter rather than plain storage, because what you store is your own enum and
 * what crosses the wire is the position in this list.
 */
static const char *const RTL_BEHAVIOUR[] = {"Land", "Hover and wait", "Hold position"};

static bool rtl_get(const ad_param_t *p, float *out) {
  (void)p;
  *out = (float)quad_params()->rtl_action;
  return true;
}

static bool rtl_set(const ad_param_t *p, float value) {
  (void)p;
  if (value < 0.0f || value > 2.0f) return false;
  quad_params()->rtl_action = (uint8_t)value;
  return true;
}

static const ad_param_t *params_table(void) {
  static ad_param_t table[6];
  static bool built = false;
  if (!built) {
    quad_params_t *p = quad_params();
    table[0] = (ad_param_t)AD_F32("RATE_P", &p->rate_p, 0.0f, 0.5f, "",
                                  "Rate loop gain. Raise it and watch it lean harder");
    table[1] = (ad_param_t)AD_F32("HOVER_THR", &p->hover_throttle, 0.1f, 0.9f, "",
                                  "Throttle it takes to hold height");
    table[2] = (ad_param_t)AD_F32("WP_SPEED", &p->wp_speed_ms, 1.0f, 15.0f, "m/s",
                                  "Speed held between waypoints");
    table[3] = (ad_param_t)AD_F32("CLIMB_SPD", &p->climb_speed_ms, 0.5f, 6.0f, "m/s",
                                  "Vertical speed in a climb or descent");
    table[4] = (ad_param_t)AD_F32("RTL_ALT", &p->rtl_alt_m, 5.0f, 120.0f, "m",
                                  "Height it climbs to before returning");
    table[5] = (ad_param_t)AD_ENUM("RTL_ACTION", rtl_get, rtl_set, RTL_BEHAVIOUR, 3,
                                   "What it does once it arrives home");
    built = true;
  }
  return table;
}

/* ─── 3. A positional calibration ──────────────────────────────────────────── */

static const ad_cal_pose_t ACCEL_POSES[] = {
  {"Level",       0.0f,   0.0f},
  {"Left side",  -90.0f,  0.0f},
  {"Right side",  90.0f,  0.0f},
  {"Nose down",   0.0f, -90.0f},
  {"Nose up",     0.0f,  90.0f},
  {"On its back", 180.0f, 0.0f},
};

static const ad_calibration_t CALS[] = {
  {
    .id = "accel",
    .name = "Accelerometer",
    .kind = AD_CAL_POSITIONAL,
    .requirements = AD_CAL_REQ_DISARMED | AD_CAL_REQ_STATIONARY,
    .warning = "Hold the frame in each position until it is accepted. A hand that "
               "drifts is the usual reason this fails.",
    .poses = ACCEL_POSES,
    .pose_count = AD_COUNT(ACCEL_POSES),
  },
};

/* ─── 4. Callbacks ─────────────────────────────────────────────────────────── */

static bool on_param_set(const ad_param_t *p, float value, void *user) {
  (void)p; (void)value; (void)user;
  return quad_save_params();
}

static bool on_mission(const ad_wp_t *items, uint16_t count, char *why, size_t why_len,
                       void *user) {
  (void)user;
  quad_wp_t wps[QUAD_MAX_WAYPOINTS];
  if (count > QUAD_MAX_WAYPOINTS) count = QUAD_MAX_WAYPOINTS;
  for (uint16_t i = 0; i < count; i++) {
    wps[i].lat = items[i].lat;
    wps[i].lon = items[i].lon;
    /* The plan may be drawn in any frame; this firmware flies relative to the launch. */
    wps[i].alt_m = items[i].alt;
  }

  const char *reason = "Rejected";
  if (quad_load_mission(wps, count, &reason)) return true;
  snprintf(why, why_len, "%s", reason);
  return false;
}

static bool on_command(uint16_t cmd, const float args[7], char *why, size_t why_len,
                       void *user) {
  (void)user;
  const char *reason = "Refused";

  switch (cmd) {
    case AD_CMD_ARM:
      /* a[1] is 21196 when the operator has chosen to force it. */
      if (quad_arm(args[0] != 0.0f, args[1] == 21196.0f, &reason)) return true;
      break;
    case AD_CMD_TAKEOFF:
      /* a[0] is the altitude, which MAVLink puts in param7 and the SDK moves here. */
      if (quad_takeoff(args[0], &reason)) return true;
      break;
    case AD_CMD_SET_MODE:
      /* a[0] is one of the ids in MODES. The SDK refuses anything else for you. */
      if (quad_set_mode((quad_mode_t)args[0], &reason)) return true;
      break;
    case AD_CMD_START_MISSION:
      if (quad_set_mode(QUAD_AUTO, &reason)) return true;
      break;
    case AD_CMD_RETURN_HOME:
      if (quad_set_mode(QUAD_RTL, &reason)) return true;
      break;
    default:
      snprintf(why, why_len, "This aircraft does not do that");
      return false;
  }

  /* Whatever the firmware said, in its own words, is what the operator reads. */
  snprintf(why, why_len, "%s", reason);
  return false;
}

static bool on_calibrate(const char *cal_id, ad_cal_action_t action, char *why,
                         size_t why_len, void *user) {
  (void)cal_id; (void)user;
  const char *reason = "Refused";

  switch (action) {
    case AD_CAL_START:
      if (!quad_accel_cal_start(&reason)) { snprintf(why, why_len, "%s", reason); return false; }
      ardudeck_cal_progress("accel", &(ad_cal_progress_t){.step = 0, .percent = 0});
      return true;
    case AD_CAL_ACCEPT: {
      uint8_t next = 0;
      float quality = 0.0f;
      if (quad_accel_cal_accept(&next, &quality)) {
        ardudeck_cal_done("accel", true, quality, "Offsets stored");
      } else {
        ardudeck_cal_progress("accel", &(ad_cal_progress_t){
          .step = next, .step_done = true,
          .percent = (uint8_t)(next * 100 / AD_COUNT(ACCEL_POSES)),
        });
      }
      return true;
    }
    case AD_CAL_CANCEL:
      quad_accel_cal_cancel();
      ardudeck_cal_done("accel", false, 0.0f, "Cancelled");
      return true;
    default:
      snprintf(why, why_len, "Finish the poses instead");
      return false;
  }
}

/* ─── 5. The link ──────────────────────────────────────────────────────────── */

static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  transport_send(buf, len);
}

static uint32_t now_ms(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

static ad_config_t CONFIG;

void ardudeck_link_begin(void) {
  CONFIG.caps = &CAPS;
  CONFIG.params = params_table();
  CONFIG.param_count = 6;
  CONFIG.calibrations = CALS;
  CONFIG.calibration_count = AD_COUNT(CALS);
  CONFIG.send = sink;
  CONFIG.now_ms = now_ms;
  CONFIG.on_param_set = on_param_set;
  CONFIG.on_mission = on_mission;
  CONFIG.on_command = on_command;
  CONFIG.on_calibrate = on_calibrate;
  ardudeck_begin(&CONFIG);
}

/* ─── 6. Hand it the state the firmware already has ────────────────────────── */

void ardudeck_link_tick(void) {
  ardudeck_position(quad_lat(), quad_lon(), quad_groundspeed_ms(), quad_heading_deg(),
                    3 /* a real one reports its GPS fix */, 14);
  /* The two a boat has no use for: the horizon moves, and height is a real number. */
  ardudeck_attitude(quad_roll_rad(), quad_pitch_rad(), quad_yaw_rad());
  ardudeck_altitude(quad_alt_amsl_m(), quad_alt_rel_m(), quad_climb_ms());
  ardudeck_status((uint16_t)quad_mode(), quad_active_waypoint(), quad_battery_v(),
                  quad_armed());
  ardudeck_battery(quad_battery_v(), 6.5f, -1);

  /*
   * A link failsafe is fed by the ground station being there, not by the vehicle sending.
   * The SDK transmits either way, so asking it is the only way to know.
   */
  if (quad_flying() && ardudeck_silent_for(now_ms()) > 3000) {
    const char *why = NULL;
    quad_set_mode(QUAD_RTL, &why);
  }

  ardudeck_tick(now_ms());
}

void ardudeck_link_receive(const uint8_t *buf, size_t len) {
  ardudeck_receive(buf, len);
}
