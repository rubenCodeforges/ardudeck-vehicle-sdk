/*
 * A fixed wing, end to end.
 *
 * What only a wing has: an airspeed that is not the groundspeed, a takeoff that is a
 * pitch angle rather than a height, a loiter that is a circle because it cannot stop, and
 * the two calibration shapes nothing else here uses.
 */

#include <stdio.h>   /* the vendor's snprintf, not the SDK's: the core has none */

#include "ardudeck.h"

/* Stand-ins for the firmware this would be compiled into. */
extern struct { float roll_limit_deg, pitch_limit_deg, trim_airspeed_ms, min_airspeed_ms,
                      loiter_radius_m, rtl_alt_m; uint16_t trim_throttle_pct; } cfg;
extern int      config_save(void);
extern double   gps_lat(void), gps_lon(void);
extern float    gps_speed_ms(void), heading_deg(void), battery_volts(void);
extern float    roll_rad(void), pitch_rad(void), yaw_rad(void);
extern float    airspeed_ms(void), alt_amsl_m(void), alt_rel_m(void), climb_ms(void);
extern int      gps_fix(void), gps_sats(void), is_armed(void), is_flying(void);
extern int      fc_mode(void), fc_active_item(void);
extern int      fc_load_mission(const ad_wp_t *items, unsigned count);
extern int      fc_set_mode(int mode), fc_arm(int arm, int forced);
extern int      fc_takeoff(float pitch_deg);
extern int      airspeed_zero(int action), level_cal(int action), servo_sweep(int action);
extern int      prearm_failed(char *out, unsigned len);
extern unsigned now_ms_impl(void);
extern void     link_send(const void *buf, unsigned len);

extern void     vTaskDelay(unsigned ticks);
#define pdMS_TO_TICKS(ms) ((ms) / 10)

enum { MODE_MANUAL, MODE_FBWA, MODE_CRUISE, MODE_AUTO, MODE_LOITER, MODE_RTL,
       MODE_LAND, MODE_FAILSAFE };
#define MAX_WAYPOINTS 96

/* ─── 1. Who this vehicle is ───────────────────────────────────────────────── */

static const uint16_t MISSION_CMDS[] = {
  AD_NAV_WAYPOINT, AD_NAV_TAKEOFF, AD_NAV_LAND, AD_NAV_LOITER_TIME,
  AD_NAV_RETURN_TO_LAUNCH, AD_DO_SET_SPEED,
};

static const ad_mode_t MODES[] = {
  {MODE_MANUAL,   "Manual",   0},
  {MODE_FBWA,     "FBWA",     0},
  {MODE_CRUISE,   "Cruise",   0},
  {MODE_AUTO,     "Auto",     AD_MODE_MISSION},   /* the mode that flies a plan */
  /* A wing cannot hold position, so its loiter is a circle at LOITER_RAD. */
  {MODE_LOITER,   "Loiter",   0},
  {MODE_RTL,      "RTL",      0},
  /* Landing commits to an approach. Offering it on the ground is offering a crash. */
  {MODE_LAND,     "Land",     AD_MODE_ARMED_ONLY},
  {MODE_FAILSAFE, "Failsafe", AD_MODE_TERMINAL},
};

static const ad_capability_t CAPS = {
  .vendor = "acme",
  .model = "Skywalker 1900",
  .firmware = "3.2.0",
  .frame = AD_FRAME_FIXED_WING,
  .features = AD_FEAT_PARAMS | AD_FEAT_MISSION | AD_FEAT_COMMANDS |
              AD_FEAT_CALIBRATION | AD_FEAT_HOME,
  .mission_cmds = MISSION_CMDS,
  .mission_cmd_count = AD_COUNT(MISSION_CMDS),
  .mission_capacity = MAX_WAYPOINTS,
  .modes = MODES,
  .mode_count = AD_COUNT(MODES),
};

/* ─── 2. Parameters ────────────────────────────────────────────────────────── */

static const ad_param_t PARAMS[] = {
  AD_F32("ROLL_LIMIT", &cfg.roll_limit_deg, 10.0f, 75.0f, "deg",
         "Steepest bank the autopilot will command"),
  AD_F32("PITCH_LIMIT", &cfg.pitch_limit_deg, 5.0f, 45.0f, "deg",
         "Steepest climb or dive the autopilot will command"),
  AD_F32("TRIM_ARSPD", &cfg.trim_airspeed_ms, 8.0f, 40.0f, "m/s",
         "Airspeed it cruises at. Below MIN_ARSPD it will not hold height"),
  AD_F32("MIN_ARSPD", &cfg.min_airspeed_ms, 5.0f, 30.0f, "m/s",
         "Never commanded below this. Set it above your measured stall"),
  AD_F32("LOITER_RAD", &cfg.loiter_radius_m, 20.0f, 500.0f, "m",
         "Radius of the circle it flies when holding. A wing cannot stop"),
  AD_F32("RTL_ALT", &cfg.rtl_alt_m, 20.0f, 300.0f, "m",
         "Height it climbs to before returning"),
  AD_U16("TRIM_THR", &cfg.trim_throttle_pct, 0, 100, "%",
         "Throttle that holds TRIM_ARSPD in level flight"),
};

/* ─── 3. Three calibrations, three different shapes ────────────────────────── */

/*
 * An airspeed sensor is zeroed by covering the pitot and waiting: nothing to move,
 * nothing to hold, no progress worth drawing. That is AD_CAL_INSTANT.
 *
 * Control throws are checked by moving one stick end to end while the vehicle watches
 * what the surfaces did. That is AD_CAL_SWEEP, and `prompt` is what to move.
 */
static const ad_cal_pose_t LEVEL_POSES[] = {
  {"Level, on its wheels or a flat bench", 0.0f, 0.0f},
};

static const ad_calibration_t CALS[] = {
  {
    .id = "airspeed",
    .name = "Airspeed zero",
    .kind = AD_CAL_INSTANT,
    .requirements = AD_CAL_REQ_DISARMED | AD_CAL_REQ_STATIONARY,
    .warning = "Cover the pitot tube with your hand and keep it out of the wind. "
               "A breeze during this becomes a permanent offset.",
  },
  {
    .id = "level",
    .name = "Level",
    .kind = AD_CAL_POSITIONAL,
    .requirements = AD_CAL_REQ_DISARMED | AD_CAL_REQ_STATIONARY |
                    AD_CAL_REQ_LEVEL_SURFACE,
    .warning = "Set the airframe at its flying attitude, not nose down on the bench.",
    .poses = LEVEL_POSES,
    .pose_count = AD_COUNT(LEVEL_POSES),
  },
  {
    .id = "throws",
    .name = "Control throws",
    .kind = AD_CAL_SWEEP,
    .requirements = AD_CAL_REQ_DISARMED | AD_CAL_REQ_PROPS_OFF,
    .warning = "The surfaces will move. Keep your fingers clear of the linkages.",
    .prompt = "Move the right stick fully left, right, forward and back",
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
      const bool arm = args[0] != 0.0f;
      const bool forced = args[1] == 21196.0f;
      if (arm && !forced && prearm_failed(why, (unsigned)why_len)) return false;
      return fc_arm(arm, forced) == 0;
    }
    case AD_CMD_TAKEOFF:
      /*
       * a[0] is the altitude for a vehicle that climbs vertically. A wing does not: it
       * needs a climb angle and a runway or a pair of hands. Take the number as the
       * target height and use your own launch pitch to get there.
       */
      if (!is_armed()) {
        snprintf(why, why_len, "Arm it first");
        return false;
      }
      if (airspeed_ms() > 2.0f) {
        snprintf(why, why_len, "Already moving. Launch it by hand from FBWA instead");
        return false;
      }
      return fc_takeoff(cfg.pitch_limit_deg) == 0;
    case AD_CMD_START_MISSION:
      if (gps_fix() < 3) {
        snprintf(why, why_len, "No 3D fix. %d satellites.", gps_sats());
        return false;
      }
      return true;
    case AD_CMD_SET_MODE:
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
  if (cal_id[0] == 'a') return airspeed_zero((int)action) == 0;
  if (cal_id[0] == 'l') return level_cal((int)action) == 0;
  if (cal_id[0] == 't') return servo_sweep((int)action) == 0;
  snprintf(why, why_len, "No such calibration");
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
    /* speed_ms here is groundspeed: it is what moves the icon across the map. */
    ardudeck_position(gps_lat(), gps_lon(), gps_speed_ms(), heading_deg(),
                      (uint8_t)gps_fix(), (uint8_t)gps_sats());
    /* And this is the one the pilot flies by. Into wind they are nothing alike. */
    ardudeck_airspeed(airspeed_ms());
    ardudeck_attitude(roll_rad(), pitch_rad(), yaw_rad());
    ardudeck_altitude(alt_amsl_m(), alt_rel_m(), climb_ms());
    ardudeck_status((uint16_t)fc_mode(), (uint16_t)fc_active_item(), battery_volts(),
                    is_armed() != 0);

    if (ardudeck_silent_for(now_ms()) > 3000 && is_flying()) fc_set_mode(MODE_RTL);

    ardudeck_tick(now_ms());
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
