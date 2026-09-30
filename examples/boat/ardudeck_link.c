/*
 * A surface vessel, end to end.
 *
 * A working boat with a helm: there is a physical controller on the hull that owns the
 * drives, and this is the second way in, for the laptop on the bank. That arrangement is
 * the common one on anything crewed or tended, and it changes what a ground station is
 * allowed to do.
 *
 * What a boat has that an aircraft does not: no altitude worth reporting, no attitude
 * anyone wants a horizon from, a heading that comes from a compass rather than a course
 * over ground, and a drift that means arriving at a waypoint is a radius, not a point.
 */

#include <stdio.h>   /* the vendor's snprintf, not the SDK's: the core has none */

#include "ardudeck.h"

/* Stand-ins for the firmware this would be compiled into. */
extern struct { float transit_speed_ms, arrive_radius_m, heading_kp, heading_kd,
                      turn_rate_max_dps, shallow_alarm_m, battery_low_v; } cfg;
extern int      config_save(void);
extern double   gps_lat(void), gps_lon(void);
extern float    gps_speed_ms(void), compass_heading_deg(void), battery_volts(void);
extern float    depth_m(void);
extern int      gps_fix(void), gps_sats(void), drives_enabled(void);
extern int      helm_has_control(void);          /* the physical controller is holding it */
extern int      nav_mode(void), nav_active_item(void);
extern int      nav_load(const ad_wp_t *items, unsigned count);
extern int      nav_read(ad_wp_t *out, unsigned capacity);
extern int      nav_set_mode(int mode);
extern int      compass_cal(int action);
extern unsigned now_ms_impl(void);
extern void     link_send(const void *buf, unsigned len);

/* FreeRTOS, from your platform. Declared here only so this file compiles on its own. */
extern void     vTaskDelay(unsigned ticks);
#define pdMS_TO_TICKS(ms) ((ms) / 10)

enum { NAV_STANDBY, NAV_TRANSIT, NAV_STATION, NAV_RETURN, NAV_MOORED, NAV_HELM };
#define NAV_MAX_ITEMS 48

/* ─── 1. Who this vehicle is ───────────────────────────────────────────────── */

static const uint16_t MISSION_CMDS[] = {
  AD_NAV_WAYPOINT, AD_NAV_LOITER_TIME, AD_NAV_RETURN_TO_LAUNCH, AD_DO_SET_SPEED,
};

static const ad_mode_t MODES[] = {
  {NAV_STANDBY, "Standby",  0},
  {NAV_TRANSIT, "Transit",  AD_MODE_MISSION},   /* the mode that runs a plan */
  /* Holding a position against wind and current, which a boat can do and a wing cannot. */
  {NAV_STATION, "Station",  0},
  {NAV_RETURN,  "Return",   0},
  {NAV_MOORED,  "Moored",   AD_MODE_TERMINAL},
  /*
   * The helm is a physical controller wired to the drives, with its own deadman. A ground
   * station cannot hold that switch, so it must not be able to select the mode that
   * depends on it. Declared so the operator can see why the boat is not responding.
   */
  {NAV_HELM,    "Helm",     AD_MODE_LOCAL_ONLY},
};

static const ad_capability_t CAPS = {
  .vendor = "acme",
  .model = "survey cat 1200",
  .firmware = "2.3.0",
  .frame = AD_FRAME_SURFACE_BOAT,
  .features = AD_FEAT_PARAMS | AD_FEAT_MISSION | AD_FEAT_MISSION_READ |
              AD_FEAT_COMMANDS | AD_FEAT_CALIBRATION | AD_FEAT_HOME,
  .mission_cmds = MISSION_CMDS,
  .mission_cmd_count = AD_COUNT(MISSION_CMDS),
  .mission_capacity = NAV_MAX_ITEMS,
  .modes = MODES,
  .mode_count = AD_COUNT(MODES),
};

/* ─── 2. Parameters, with the metadata that makes them editable ────────────── */

static const ad_param_t PARAMS[] = {
  AD_F32("TRANSIT_SPD", &cfg.transit_speed_ms, 0.2f, 6.0f, "m/s",
         "Speed held between waypoints"),
  AD_F32("ARRIVE_RAD", &cfg.arrive_radius_m, 1.0f, 30.0f, "m",
         "How close counts as reaching a waypoint. Raise it in a current"),
  AD_F32("TURN_RATE", &cfg.turn_rate_max_dps, 5.0f, 90.0f, "deg/s",
         "Fastest turn commanded. Lower it for a towed sensor"),
  AD_F32("HDG_KP", &cfg.heading_kp, 0.0f, 5.0f, "", "Steering gain"),
  AD_F32("HDG_KD", &cfg.heading_kd, 0.0f, 2.0f, "",
         "Steering damping. Raise this if the hull weaves"),
  AD_F32("SHALLOW_M", &cfg.shallow_alarm_m, 0.0f, 50.0f, "m",
         "Warns below this sounder depth. 0 disables"),
  AD_F32("BATT_LOW_V", &cfg.battery_low_v, 0.0f, 60.0f, "V",
         "Pack voltage that triggers the low action. 0 disables"),
};

/* ─── 3. Calibration: a compass, settled by turning the hull ───────────────── */

/*
 * Coverage, not poses. Nobody is going to pick a boat up and hold it nose down, so the
 * routine ends when the hull has been through enough headings, however that happens.
 */
static const ad_cal_track_t COMPASS_TRACKS[] = {
  {"Headings covered", "",    12.0f},
  {"Total rotation",   "deg", 720.0f},
};

static const ad_calibration_t CALS[] = {
  {
    .id = "compass",
    .name = "Compass",
    .kind = AD_CAL_COVERAGE,
    .requirements = AD_CAL_REQ_DISARMED,
    .warning = "Turn the boat through two full circles, on the water and away from "
               "steel hulls, pontoons and the launch ramp.",
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
  /* The same loader the boat's own controller uses, so there is one set of rules about
     what a valid plan is rather than two that drift. */
  if (nav_load(items, count) == 0) return true;
  /* The operator reads this sentence, so it has to say something they can act on. */
  snprintf(why, why_len, "Could not store %u waypoints", count);
  return false;
}

static uint16_t on_mission_read(ad_wp_t *items, uint16_t capacity, void *user) {
  (void)user;
  const int n = nav_read(items, capacity);
  return n < 0 ? 0 : (uint16_t)n;
}

static bool on_command(uint16_t cmd, const float args[7], char *why, size_t why_len,
                       void *user) {
  (void)user;

  /* Nothing from the bank while somebody is holding the deadman on the hull. */
  if (helm_has_control() && cmd != AD_CMD_RETURN_HOME) {
    snprintf(why, why_len, "The helm has control");
    return false;
  }

  switch (cmd) {
    case AD_CMD_START_MISSION:
      if (gps_fix() < 3) {
        snprintf(why, why_len, "No GPS fix. %d satellites, needs 5.", gps_sats());
        return false;
      }
      return nav_set_mode(NAV_TRANSIT) == 0;
    case AD_CMD_SET_MODE:
      /* a[0] is a mode id from MODES above, already unpacked by the SDK. */
      return nav_set_mode((int)args[0]) == 0;
    case AD_CMD_RETURN_HOME:
      return nav_set_mode(NAV_RETURN) == 0;
    case AD_CMD_ARM:
      if (args[0] != 0.0f && battery_volts() < cfg.battery_low_v) {
        snprintf(why, why_len, "Pack at %.1f V, below BATT_LOW_V", (double)battery_volts());
        return false;
      }
      return true;
    default:
      snprintf(why, why_len, "This boat does not do that");
      return false;
  }
}

static bool on_calibrate(const char *cal_id, ad_cal_action_t action, char *why,
                         size_t why_len, void *user) {
  (void)cal_id; (void)user;
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
  .on_mission_read = on_mission_read,
  .on_command = on_command,
  .on_calibrate = on_calibrate,
};

void ardudeck_task(void) {
  ardudeck_begin(&CONFIG);

  for (;;) {
    /*
     * Heading comes from the compass, not from course over ground. A boat being set
     * sideways by a current is pointing somewhere other than where it is going, and the
     * icon should show where it is pointing.
     */
    ardudeck_position(gps_lat(), gps_lon(), gps_speed_ms(), compass_heading_deg(),
                      (uint8_t)gps_fix(), (uint8_t)gps_sats());
    ardudeck_status((uint16_t)nav_mode(), (uint16_t)nav_active_item(), battery_volts(),
                    drives_enabled() != 0);

    /* No attitude and no altitude on purpose. See the note in docs/build-a-boat.md. */

    if (cfg.shallow_alarm_m > 0.0f && depth_m() > 0.0f &&
        depth_m() < cfg.shallow_alarm_m) {
      ardudeck_notify(AD_WARNING, "Shallow water");
    }

    ardudeck_tick(now_ms());
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* Inbound bytes, from wherever they arrive:
 *
 *   ardudeck_receive(buf, len);
 *
 * And when the compass routine has something to report:
 *
 *   ardudeck_cal_progress("compass", &(ad_cal_progress_t){
 *     .track = {headings_seen, total_rotation_deg},
 *     .percent = pct,
 *     .hint = turning_too_fast ? "Turn more slowly" : NULL,
 *   });
 *
 *   ardudeck_cal_done("compass", true, 0.91f, "Offsets stored");
 *   ardudeck_param_changed("MAG_OFS_X");   // for every value the routine wrote
 */
