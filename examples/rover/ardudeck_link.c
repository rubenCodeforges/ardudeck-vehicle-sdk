/*
 * The integration. Everything ArduDeck needs, in one file that talks to the firmware
 * only through vehicle.h.
 *
 * Read it next to vehicle.c: that file did not change by a single line to get a map, an
 * instrument panel, a parameter editor, mission upload and a command bar.
 */
#include "ardudeck_link.h"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "ardudeck.h"
#include "vehicle.h"

/* ─── 1. Who this vehicle is ───────────────────────────────────────────────── */

static const uint16_t MISSION_CMDS[] = {AD_NAV_WAYPOINT};

static const ad_mode_t MODES[] = {
  {ROVER_IDLE,   "Idle",   0},
  {ROVER_MANUAL, "Manual", 0},
  {ROVER_AUTO,   "Auto",   AD_MODE_MISSION},   /* the mode that drives a plan */
  {ROVER_HOLD,   "Hold",   0},
};

static const ad_capability_t CAPS = {
  .vendor = "acme",
  .model = "rover kit",
  .firmware = "1.0.0",
  .frame = AD_FRAME_ROVER,
  .features = AD_FEAT_PARAMS | AD_FEAT_MISSION | AD_FEAT_COMMANDS,
  .mission_cmds = MISSION_CMDS,
  .mission_cmd_count = AD_COUNT(MISSION_CMDS),
  .mission_capacity = ROVER_MAX_WAYPOINTS,
  .modes = MODES,
  .mode_count = AD_COUNT(MODES),
};

/*
 * The parameters point straight at the firmware's own struct, so a value written from
 * ArduDeck is the value rover_step() reads on its next pass. Nothing is copied.
 */
static const ad_param_t *params_table(void) {
  static ad_param_t table[4];
  static bool built = false;
  if (!built) {
    rover_params_t *p = rover_params();
    table[0] = (ad_param_t)AD_F32("CRUISE_SPD", &p->cruise_speed_ms, 0.2f, 4.0f, "m/s",
                                  "Speed held between waypoints");
    table[1] = (ad_param_t)AD_F32("ARRIVE_RAD", &p->arrive_radius_m, 0.5f, 20.0f, "m",
                                  "How close counts as reaching a waypoint");
    table[2] = (ad_param_t)AD_F32("HDG_KP", &p->heading_kp, 0.0f, 5.0f, "",
                                  "Steering gain. Raise it and watch the turns tighten");
    table[3] = (ad_param_t)AD_F32("BATT_LOW_V", &p->battery_low_v, 0.0f, 30.0f, "V",
                                  "Refuses to arm below this. 0 disables");
    built = true;
  }
  return table;
}

/* ─── 2. Callbacks, each one a line or two of translation ──────────────────── */

static bool on_param_set(const ad_param_t *p, float value, void *user) {
  (void)p; (void)value; (void)user;
  return rover_save_params();
}

static bool on_mission(const ad_wp_t *items, uint16_t count, char *why, size_t why_len,
                       void *user) {
  (void)user;
  /* The SDK's waypoint carries more than this rover understands, so take the two fields
     it does and let the firmware's own rules decide about the rest. */
  rover_wp_t wps[ROVER_MAX_WAYPOINTS];
  if (count > ROVER_MAX_WAYPOINTS) count = ROVER_MAX_WAYPOINTS;
  for (uint16_t i = 0; i < count; i++) {
    wps[i].lat = items[i].lat;
    wps[i].lon = items[i].lon;
  }

  const char *reason = "Rejected";
  if (rover_load_mission(wps, count, &reason)) return true;
  snprintf(why, why_len, "%s", reason);
  return false;
}

static bool on_command(uint16_t cmd, const float args[7], char *why, size_t why_len,
                       void *user) {
  (void)user;
  const char *reason = "Refused";

  switch (cmd) {
    case AD_CMD_ARM:
      if (rover_arm(args[0] != 0.0f, &reason)) return true;
      break;
    case AD_CMD_SET_MODE:
      /* a[0] is one of the ids in MODES above. The SDK refuses anything else. */
      if (rover_set_mode((rover_mode_t)args[0], &reason)) return true;
      break;
    case AD_CMD_START_MISSION:
      if (rover_set_mode(ROVER_AUTO, &reason)) return true;
      break;
    case AD_CMD_RETURN_HOME:
      if (rover_set_mode(ROVER_HOLD, &reason)) return true;
      break;
    default:
      snprintf(why, why_len, "This rover does not do that");
      return false;
  }

  /* Whatever the firmware said, said in its own words, is what the operator reads. */
  snprintf(why, why_len, "%s", reason);
  return false;
}

/* ─── 3. The link ──────────────────────────────────────────────────────────── */

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
  CONFIG.param_count = 4;
  CONFIG.send = sink;
  CONFIG.now_ms = now_ms;
  CONFIG.on_param_set = on_param_set;
  CONFIG.on_mission = on_mission;
  CONFIG.on_command = on_command;
  ardudeck_begin(&CONFIG);
}

/* ─── 4. Hand it the state the firmware already has ────────────────────────── */

void ardudeck_link_tick(void) {
  ardudeck_position(rover_lat(), rover_lon(), rover_speed_ms(), rover_heading_deg(),
                    3 /* a real one reports its GPS fix */, 12);
  ardudeck_status((uint16_t)rover_mode(), rover_active_waypoint(), rover_battery_v(),
                  rover_armed());
  ardudeck_battery(rover_battery_v(), 1.2f, -1);
  ardudeck_tick(now_ms());
}

void ardudeck_link_receive(const uint8_t *buf, size_t len) {
  ardudeck_receive(buf, len);
}
