/*
 * A multirotor, end to end.
 *
 * Where the boat example stops at the water line, this one covers what only a flying
 * vehicle has: an attitude to draw a horizon with, an altitude, a takeoff that needs a
 * height, a six point accelerometer routine, and a parameter that is a choice rather
 * than a number.
 *
 * Rungs 0 through 4 plus two calibrations. Everything here is declaration; the flight
 * code is whatever you already have.
 */

#include <stdio.h>   /* the vendor's snprintf, not the SDK's: the core has none */

#include "ardudeck.h"

/* Stand-ins for the firmware this would be compiled into. */
extern struct { float rate_p, rate_d, hover_thr, rtl_alt_m, land_speed_ms;
                uint16_t wp_speed_cms; } cfg;
extern int      config_save(void);
extern double   gps_lat(void), gps_lon(void);
extern float    gps_speed_ms(void), heading_deg(void), battery_volts(void);
extern float    roll_rad(void), pitch_rad(void), yaw_rad(void);
extern float    alt_amsl_m(void), alt_rel_m(void), climb_ms(void);
extern int      gps_fix(void), gps_sats(void), is_armed(void), is_flying(void);
extern int      fc_mode(void), fc_active_item(void);
extern int      fc_load_mission(const ad_wp_t *items, unsigned count);
extern int      fc_set_mode(int mode);
extern int      fc_arm(int arm, int forced), fc_takeoff(float alt_m);
extern int      accel_cal(int action), compass_cal(int action);
extern int      prearm_failed(char *out, unsigned len);
extern unsigned now_ms_impl(void);
extern void     link_send(const void *buf, unsigned len);

extern void     vTaskDelay(unsigned ticks);
#define pdMS_TO_TICKS(ms) ((ms) / 10)

enum { MODE_STABILIZE, MODE_ALTHOLD, MODE_LOITER, MODE_AUTO, MODE_RTL, MODE_LAND,
       MODE_FAILSAFE };
#define MAX_WAYPOINTS 64

/* ─── 1. Who this vehicle is ───────────────────────────────────────────────── */

static const uint16_t MISSION_CMDS[] = {
  AD_NAV_WAYPOINT, AD_NAV_TAKEOFF, AD_NAV_LAND, AD_NAV_LOITER_TIME,
  AD_NAV_RETURN_TO_LAUNCH,
};

static const ad_mode_t MODES[] = {
  {MODE_STABILIZE, "Stabilize", 0},
  {MODE_ALTHOLD,   "AltHold",   0},
  {MODE_LOITER,    "Loiter",    0},
  {MODE_AUTO,      "Auto",      AD_MODE_MISSION},   /* the mode that flies a plan */
  {MODE_RTL,       "RTL",       0},
  /* Landing is worth entering deliberately, but only once it is already up. */
  {MODE_LAND,      "Land",      AD_MODE_ARMED_ONLY},
  /* Entered by the failsafe, never chosen from a picker. */
  {MODE_FAILSAFE,  "Failsafe",  AD_MODE_TERMINAL},
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
  .mission_capacity = MAX_WAYPOINTS,
  .modes = MODES,
  .mode_count = AD_COUNT(MODES),
};

/* ─── 2. Parameters, including one that is a choice ────────────────────────── */

/*
 * A dropdown is a parameter whose value is an index into these labels. It needs a getter
 * and a setter rather than plain storage, because what you store is your own enum and
 * what crosses the wire is the position in this list.
 */
static const char *const RTL_BEHAVIOUR[] = {"Land", "Hover and wait", "Return to pilot"};

static uint8_t rtl_behaviour = 0;

static bool rtl_get(const ad_param_t *p, float *out) {
  (void)p;
  *out = (float)rtl_behaviour;
  return true;
}

static bool rtl_set(const ad_param_t *p, float value) {
  (void)p;
  if (value < 0.0f || value > 2.0f) return false;
  rtl_behaviour = (uint8_t)value;
  return true;
}

static const ad_param_t PARAMS[] = {
  AD_F32("RATE_P", &cfg.rate_p, 0.0f, 0.5f, "", "Rate loop gain on roll and pitch"),
  AD_F32("RATE_D", &cfg.rate_d, 0.0f, 0.05f, "",
         "Rate loop damping. Raise if it oscillates in a fast stop"),
  AD_F32("HOVER_THR", &cfg.hover_thr, 0.1f, 0.9f, "",
         "Throttle it takes to hold height. Learned in flight"),
  AD_F32("RTL_ALT", &cfg.rtl_alt_m, 5.0f, 120.0f, "m",
         "Height it climbs to before returning"),
  AD_F32("LAND_SPD", &cfg.land_speed_ms, 0.2f, 2.0f, "m/s", "Descent rate while landing"),
  AD_U16("WP_SPEED", &cfg.wp_speed_cms, 100, 1500, "cm/s", "Speed held between waypoints"),
  AD_ENUM("RTL_ACTION", rtl_get, rtl_set, RTL_BEHAVIOUR, 3,
          "What it does once it arrives home"),
};

/* ─── 3. Calibration: one positional, one coverage ─────────────────────────── */

/*
 * A positional routine names each orientation and waits. The operator holds the frame
 * there and presses accept, which arrives as AD_CAL_ACCEPT, and the last one ends it.
 */
static const ad_cal_pose_t ACCEL_POSES[] = {
  {"Level",      0.0f,   0.0f},
  {"Left side", -90.0f,  0.0f},
  {"Right side", 90.0f,  0.0f},
  {"Nose down",  0.0f, -90.0f},
  {"Nose up",    0.0f,  90.0f},
  {"On its back", 180.0f, 0.0f},
};

static const ad_cal_track_t COMPASS_TRACKS[] = {
  {"Directions covered", "",    12.0f},
  {"Total turning",      "deg", 720.0f},
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
  {
    .id = "compass",
    .name = "Compass",
    .kind = AD_CAL_COVERAGE,
    .requirements = AD_CAL_REQ_DISARMED,
    .warning = "Turn the aircraft through every orientation, away from cars and "
               "reinforced concrete.",
    .tracks = COMPASS_TRACKS,
    .track_count = AD_COUNT(COMPASS_TRACKS),
  },
};

/* ─── 4. Callbacks ─────────────────────────────────────────────────────────── */

static bool on_param_set(const ad_param_t *p, float value, void *user) {
  (void)p; (void)value; (void)user;
  return config_save() == 0;
}

static bool on_mission(const ad_wp_t *items, uint16_t count, char *why, size_t why_len,
                       void *user) {
  (void)user;
  /* seq is the index in this array. There is no home item at position 0. */
  if (count > MAX_WAYPOINTS) {
    snprintf(why, why_len, "%u waypoints, this aircraft holds %d", count, MAX_WAYPOINTS);
    return false;
  }
  if (fc_load_mission(items, count) == 0) return true;
  snprintf(why, why_len, "Could not store %u waypoints", count);
  return false;
}

static bool on_command(uint16_t cmd, const float args[7], char *why, size_t why_len,
                       void *user) {
  (void)user;
  switch (cmd) {
    case AD_CMD_ARM: {
      /* a[1] is 21196 when the operator has chosen to force it. */
      const bool arm = args[0] != 0.0f;
      const bool forced = args[1] == 21196.0f;
      if (arm && !forced && prearm_failed(why, (unsigned)why_len)) return false;
      return fc_arm(arm, forced) == 0;
    }
    case AD_CMD_TAKEOFF:
      /* a[0] is the altitude, which MAVLink puts in param7 and the SDK moves here. */
      if (!is_armed()) {
        snprintf(why, why_len, "Arm it first");
        return false;
      }
      return fc_takeoff(args[0]) == 0;
    case AD_CMD_START_MISSION:
      if (gps_fix() < 3) {
        snprintf(why, why_len, "No 3D fix. %d satellites.", gps_sats());
        return false;
      }
      return true;
    case AD_CMD_SET_MODE:
      /* a[0] is a mode id from MODES above. The SDK has already refused any id that
         is not in that table, so this only ever sees one of ours. */
      return fc_set_mode((int)args[0]) == 0;
    case AD_CMD_RETURN_HOME:
      return fc_set_mode(MODE_RTL) == 0;
    default:
      snprintf(why, why_len, "This aircraft does not do that");
      return false;
  }
}

static bool on_calibrate(const char *cal_id, ad_cal_action_t action, char *why,
                         size_t why_len, void *user) {
  (void)user;
  if (is_flying()) {
    snprintf(why, why_len, "Not while it is flying");
    return false;
  }
  if (cal_id[0] == 'a') return accel_cal((int)action) == 0;
  if (compass_cal((int)action) == 0) return true;
  snprintf(why, why_len, "No compass on the bus");
  return false;
}

static void sink(const uint8_t *buf, size_t len, void *user) {
  (void)user;
  link_send(buf, (unsigned)len);
}

static uint32_t now_ms(void) { return now_ms_impl(); }

/* ─── 5. Feed it state ─────────────────────────────────────────────────────── */

static const ad_config_t CONFIG = {
  .caps = &CAPS,
  .params = PARAMS,
  .param_count = AD_COUNT(PARAMS),
  .calibrations = CALS,
  .calibration_count = AD_COUNT(CALS),
  .send = sink,
  .now_ms = now_ms,
  .on_param_set = on_param_set,
  .on_mission = on_mission,
  .on_command = on_command,
  .on_calibrate = on_calibrate,
};

void ardudeck_task(void) {
  ardudeck_begin(&CONFIG);

  for (;;) {
    ardudeck_position(gps_lat(), gps_lon(), gps_speed_ms(), heading_deg(),
                      (uint8_t)gps_fix(), (uint8_t)gps_sats());
    /* The two a boat has no use for: the horizon moves, and height is a real number. */
    ardudeck_attitude(roll_rad(), pitch_rad(), yaw_rad());
    ardudeck_altitude(alt_amsl_m(), alt_rel_m(), climb_ms());
    ardudeck_status((uint16_t)fc_mode(), (uint16_t)fc_active_item(), battery_volts(),
                    is_armed() != 0);

    /*
     * A link failsafe is fed by the ground station being there, not by the vehicle
     * sending. The SDK transmits either way, so asking it is the only way to know.
     */
    if (ardudeck_silent_for(now_ms()) > 3000 && is_flying()) fc_set_mode(MODE_RTL);

    ardudeck_tick(now_ms());
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* Inbound bytes, from wherever they arrive:
 *
 *   ardudeck_receive(buf, len);
 *
 * While the accelerometer routine waits for a pose, say which one:
 *
 *   ardudeck_cal_progress("accel", &(ad_cal_progress_t){
 *     .step = pose_index,          // which entry of ACCEL_POSES
 *     .step_done = captured,       // true the moment that one is in
 *     .percent = pct,
 *   });
 *
 *   ardudeck_cal_done("accel", true, 0.96f, "Offsets stored");
 *   ardudeck_param_changed("INS_ACCOFFS_X");   // every value the routine wrote
 */
