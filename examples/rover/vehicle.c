/*
 * The firmware. A rover that drives to waypoints, with a battery that runs down.
 *
 * Nothing here mentions ArduDeck, MAVLink or the SDK. When you add the SDK to your own
 * firmware, this is the file you do not touch.
 */
#include "vehicle.h"

#include <math.h>
#include <string.h>

#define EARTH_R 6371000.0
#define DEG (M_PI / 180.0)

static rover_params_t  params;
static rover_mode_t    mode;
static bool            armed;
static double          lat, lon;
static float           heading_deg, speed_ms, battery_v;
static rover_wp_t      mission[ROVER_MAX_WAYPOINTS];
static uint16_t        mission_count, active;

void rover_init(void) {
  params.cruise_speed_ms = 1.5f;
  params.arrive_radius_m = 2.0f;
  params.heading_kp = 1.2f;
  params.battery_low_v = 10.5f;
  mode = ROVER_IDLE;
  armed = false;
  lat = 52.5163;
  lon = 13.3777;
  heading_deg = 90.0f;
  battery_v = 12.6f;
}

rover_params_t *rover_params(void) { return &params; }
bool rover_save_params(void) { return true; } /* a real one writes flash here */

double       rover_lat(void) { return lat; }
double       rover_lon(void) { return lon; }
float        rover_heading_deg(void) { return heading_deg; }
float        rover_speed_ms(void) { return speed_ms; }
float        rover_battery_v(void) { return battery_v; }
bool         rover_armed(void) { return armed; }
rover_mode_t rover_mode(void) { return mode; }
uint16_t     rover_active_waypoint(void) { return active; }
uint16_t     rover_mission_count(void) { return mission_count; }
const rover_wp_t *rover_mission(void) { return mission; }

bool rover_arm(bool arm, const char **why) {
  if (arm && battery_v < params.battery_low_v) {
    *why = "Battery too low to arm";
    return false;
  }
  armed = arm;
  if (!arm) { mode = ROVER_IDLE; speed_ms = 0.0f; }
  return true;
}

bool rover_set_mode(rover_mode_t want, const char **why) {
  if (want == ROVER_AUTO && mission_count == 0) {
    *why = "No mission loaded";
    return false;
  }
  if (want == ROVER_AUTO && !armed) {
    *why = "Arm it first";
    return false;
  }
  mode = want;
  if (want == ROVER_AUTO) active = 0;
  return true;
}

bool rover_load_mission(const rover_wp_t *items, uint16_t count, const char **why) {
  if (count > ROVER_MAX_WAYPOINTS) {
    *why = "Too many waypoints";
    return false;
  }
  memcpy(mission, items, count * sizeof items[0]);
  mission_count = count;
  active = 0;
  return true;
}

/* Metres between two positions, near enough at rover distances. */
static double distance_m(double a_lat, double a_lon, double b_lat, double b_lon) {
  const double dx = (b_lon - a_lon) * DEG * cos(a_lat * DEG) * EARTH_R;
  const double dy = (b_lat - a_lat) * DEG * EARTH_R;
  return sqrt(dx * dx + dy * dy);
}

static double bearing_deg(double a_lat, double a_lon, double b_lat, double b_lon) {
  const double dx = (b_lon - a_lon) * DEG * cos(a_lat * DEG);
  const double dy = (b_lat - a_lat) * DEG;
  double deg = atan2(dx, dy) / DEG;
  return deg < 0.0 ? deg + 360.0 : deg;
}

void rover_step(float dt_s) {
  battery_v -= 0.0008f * dt_s * (armed ? 3.0f : 1.0f);
  if (battery_v < 9.0f) battery_v = 9.0f;

  if (mode != ROVER_AUTO || !armed || mission_count == 0) {
    speed_ms = 0.0f;
    return;
  }

  const rover_wp_t *target = &mission[active];
  if (distance_m(lat, lon, target->lat, target->lon) < (double)params.arrive_radius_m) {
    if (active + 1 < mission_count) {
      active++;
    } else {
      mode = ROVER_HOLD; /* finished the plan */
      speed_ms = 0.0f;
      return;
    }
    target = &mission[active];
  }

  /* Turn towards the target, then drive. The gain is a parameter so the operator can
     see a change they make actually do something. */
  double want = bearing_deg(lat, lon, target->lat, target->lon);
  double error = want - (double)heading_deg;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  heading_deg += (float)(error * (double)params.heading_kp * (double)dt_s);
  if (heading_deg < 0.0f) heading_deg += 360.0f;
  if (heading_deg >= 360.0f) heading_deg -= 360.0f;

  speed_ms = params.cruise_speed_ms;
  const double travelled = (double)speed_ms * (double)dt_s;
  lat += travelled * cos((double)heading_deg * DEG) / EARTH_R / DEG;
  lon += travelled * sin((double)heading_deg * DEG) / (EARTH_R * cos(lat * DEG)) / DEG;
}
