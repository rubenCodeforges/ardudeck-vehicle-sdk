/*
 * A real vehicle, end to end.
 *
 * The rctestflight ESP32 boat: parameter names and flight modes as that firmware
 * actually has them, so this reads as an integration rather than a demonstration.
 * Rungs 0 through 4 plus calibration, in about sixty lines of declaration.
 *
 * The boat keeps its own WebSocket and JSON protocol for the phone that flies it. This
 * is its second surface, for a laptop. A browser cannot open a UDP socket, so neither
 * one replaces the other.
 */

#include <stdio.h>   /* the vendor's snprintf, not the SDK's: the core has none */

#include "ardudeck.h"

/* Stand-ins for the firmware this would be compiled into. */
extern struct { float cruise_speed_ms, arrival_radius_m, heading_kp, heading_kd,
                      cruise_throttle_pct, battery_low_v; } cfg;
extern int      config_save(void);
extern double   gps_lat(void), gps_lon(void);
extern float    gps_speed_ms(void), heading_deg(void), battery_volts(void);
extern int      gps_fix(void), gps_sats(void), safety_armed(void);
extern int      nav_mode(void), nav_active_item(void);
extern int      nav_load(const ad_wp_t *items, unsigned count);
extern int      nav_set_mode(int mode);
extern int      compass_cal(int action);
extern unsigned now_ms_impl(void);
extern void     link_send(const void *buf, unsigned len);

/* FreeRTOS, from your platform. Declared here only so this file compiles on its own. */
extern void     vTaskDelay(unsigned ticks);
#define pdMS_TO_TICKS(ms) ((ms) / 10)

enum { NAV_IDLE, NAV_RUNNING, NAV_HOLDING, NAV_PAUSED, NAV_RETURNING, NAV_FINISHED,
       NAV_MANUAL };
#define NAV_MAX_ITEMS 32

/* ─── 1. Who this vehicle is ───────────────────────────────────────────────── */

static const uint16_t MISSION_CMDS[] = {AD_NAV_WAYPOINT, AD_NAV_RETURN_TO_LAUNCH};

static const ad_mode_t MODES[] = {
  {NAV_IDLE,      "Idle",      0},
  {NAV_RUNNING,   "Running",   0},
  {NAV_HOLDING,   "Holding",   0},
  {NAV_PAUSED,    "Paused",    0},
  {NAV_RETURNING, "Returning", 0},
  {NAV_FINISHED,  "Finished",  AD_MODE_TERMINAL},
  /* The manual screen owns the motors through a deadman on the boat's own socket, so
     it exists but is not something a ground station may switch into. */
  {NAV_MANUAL,    "Manual",    AD_MODE_LOCAL_ONLY},
};

static const ad_capability_t CAPS = {
  .vendor = "rctestflight",
  .model = "boat kit",
  .firmware = "0.4.1",
  .frame = AD_FRAME_SURFACE_BOAT,
  .features = AD_FEAT_PARAMS | AD_FEAT_MISSION | AD_FEAT_MISSION_READ |
              AD_FEAT_COMMANDS | AD_FEAT_CALIBRATION | AD_FEAT_HOME,
  .mission_cmds = MISSION_CMDS,
  .mission_cmd_count = 2,
  .mission_capacity = NAV_MAX_ITEMS,
  .modes = MODES,
  .mode_count = 7,
};

/* ─── 2. Parameters, with the metadata that makes them editable ────────────── */

static const ad_param_t PARAMS[] = {
  AD_F32("CRUISE_SPD", &cfg.cruise_speed_ms, 0.2f, 4.0f, "m/s",
         "Speed held between waypoints"),
  AD_F32("CRUISE_THR", &cfg.cruise_throttle_pct, 0.0f, 100.0f, "%",
         "Throttle this hull needs to hold CRUISE_SPD"),
  AD_F32("ARRIVE_RAD", &cfg.arrival_radius_m, 0.5f, 20.0f, "m",
         "How close counts as reaching a waypoint"),
  AD_F32("HDG_KP", &cfg.heading_kp, 0.0f, 5.0f, "", "Steering gain"),
  AD_F32("HDG_KD", &cfg.heading_kd, 0.0f, 2.0f, "",
         "Steering damping. Raise this if the boat weaves"),
  AD_F32("BATT_LOW_V", &cfg.battery_low_v, 0.0f, 30.0f, "V",
         "Pack voltage that triggers the low action. 0 disables"),
};

/* ─── 3. Calibration: two things settled by one circle ─────────────────────── */

static const ad_cal_track_t COMPASS_TRACKS[] = {
  {"Sectors turned through", "",    12.0f},
  {"Total turning",          "deg", 720.0f},
  {"Gyro agreement",         "",    0.7f},
};

static const ad_calibration_t CALS[] = {
  {
    .id = "compass",
    .name = "Compass",
    .kind = AD_CAL_COVERAGE,
    .requirements = AD_CAL_REQ_DISARMED,
    .warning = "Turn the boat through two full circles by hand, away from "
               "steel and away from the trailer.",
    .tracks = COMPASS_TRACKS,
    .track_count = 3,
  },
};

/* ─── 4. Four callbacks ────────────────────────────────────────────────────── */

static bool on_param_set(const ad_param_t *p, float value, void *user) {
  (void)p; (void)value; (void)user;
  return config_save() == 0;
}

static bool on_mission(const ad_wp_t *items, uint16_t count, char *why, size_t why_len,
                       void *user) {
  (void)user;
  if (nav_load(items, count) == 0) return true;
  /* The operator reads this sentence, so it has to say something they can act on. */
  snprintf(why, why_len, "Could not store %u waypoints", count);
  return false;
}

static uint16_t on_mission_read(ad_wp_t *items, uint16_t capacity, void *user) {
  (void)items; (void)capacity; (void)user;
  return 0; /* the boat keeps its plan in NVS; wire this up to read it back */
}

static bool on_command(uint16_t cmd, const float args[7], char *why, size_t why_len,
                       void *user) {
  (void)user;
  switch (cmd) {
    case AD_CMD_START_MISSION:
      if (gps_fix() < 3) {
        snprintf(why, why_len, "No GPS fix. %d satellites, needs 5.", gps_sats());
        return false;
      }
      return true;
    case AD_CMD_SET_MODE:
      /* a[0] is a mode id from MODES above, already unpacked by the SDK. */
      return nav_set_mode((int)args[0]) == 0;
    case AD_CMD_RETURN_HOME:
    case AD_CMD_ARM:
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
  .param_count = 6,
  .calibrations = CALS,
  .calibration_count = 1,
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
    ardudeck_position(gps_lat(), gps_lon(), gps_speed_ms(), heading_deg(),
                      (uint8_t)gps_fix(), (uint8_t)gps_sats());
    ardudeck_status((uint16_t)nav_mode(), (uint16_t)nav_active_item(), battery_volts(),
                    safety_armed() != 0);
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
 *     .track = {sectors_seen, gyro_turn_deg, agreement},
 *     .percent = pct,
 *     .hint = turning_too_fast ? "Turn more slowly" : NULL,
 *   });
 *
 *   ardudeck_cal_done("compass", true, 0.91f, "Offsets stored, gyro was reversed");
 *   ardudeck_param_changed("MAG_OFS_X");   // for every value the routine wrote
 */
