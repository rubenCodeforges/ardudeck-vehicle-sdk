/*
 * The firmware. A multirotor that takes off, flies a plan and comes home.
 *
 * Nothing here mentions ArduDeck, MAVLink or the SDK. When you add the SDK to your own
 * firmware, this is the file you do not touch.
 */
#include "vehicle.h"

#include <math.h>
#include <string.h>

#define EARTH_R 6371000.0
#define DEG (M_PI / 180.0)
#define ACCEL_POSE_COUNT 6

static quad_params_t params;
static quad_mode_t   mode;
static bool          armed, flying;
static double        lat, lon, home_lat, home_lon;
static float         alt_rel, climb, ground_speed, heading, battery;
static float         roll, pitch;
static float         target_alt;
static quad_wp_t     mission[QUAD_MAX_WAYPOINTS];
static uint16_t      mission_count, active;
static bool          cal_running;
static uint8_t       cal_pose;

void quad_init(void) {
  params.rate_p = 0.15f;
  params.hover_throttle = 0.45f;
  params.wp_speed_ms = 5.0f;
  params.climb_speed_ms = 2.0f;
  params.rtl_alt_m = 30.0f;
  params.rtl_action = 0;
  mode = QUAD_STABILIZE;
  armed = flying = false;
  lat = home_lat = 52.5163;
  lon = home_lon = 13.3777;
  heading = 90.0f;
  battery = 16.8f;
}

quad_params_t *quad_params(void) { return &params; }
bool  quad_save_params(void) { return true; }   /* a real one writes flash here */

double      quad_lat(void) { return lat; }
double      quad_lon(void) { return lon; }
float       quad_roll_rad(void) { return roll; }
float       quad_pitch_rad(void) { return pitch; }
float       quad_yaw_rad(void) { return (float)(heading * DEG); }
float       quad_alt_rel_m(void) { return alt_rel; }
float       quad_alt_amsl_m(void) { return alt_rel + 34.0f; }  /* field elevation */
float       quad_climb_ms(void) { return climb; }
float       quad_groundspeed_ms(void) { return ground_speed; }
float       quad_heading_deg(void) { return heading; }
float       quad_battery_v(void) { return battery; }
bool        quad_armed(void) { return armed; }
bool        quad_flying(void) { return flying; }
quad_mode_t quad_mode(void) { return mode; }
uint16_t    quad_active_waypoint(void) { return active; }
uint16_t    quad_mission_count(void) { return mission_count; }

const char *quad_prearm_reason(void) {
  if (battery < 14.0f) return "Battery below 14.0 V";
  if (cal_running) return "Accelerometer calibration in progress";
  return NULL;
}

bool quad_arm(bool arm, bool forced, const char **why) {
  if (arm) {
    const char *reason = quad_prearm_reason();
    /* A forced arm overrides the checks that are advisory. A calibration in progress
       is not one of them: the aircraft is being held sideways by a person. */
    if (reason && (!forced || cal_running)) { *why = reason; return false; }
    home_lat = lat;
    home_lon = lon;
  } else if (flying) {
    *why = "Still flying. Land it first";
    return false;
  }
  armed = arm;
  return true;
}

bool quad_takeoff(float alt_m, const char **why) {
  if (!armed) { *why = "Arm it first"; return false; }
  if (flying) { *why = "Already flying"; return false; }
  if (alt_m < 1.0f || alt_m > 120.0f) { *why = "Takeoff height must be 1 to 120 m"; return false; }
  target_alt = alt_m;
  flying = true;
  mode = QUAD_ALTHOLD;
  return true;
}

bool quad_set_mode(quad_mode_t want, const char **why) {
  if (want == QUAD_AUTO && mission_count == 0) { *why = "No mission loaded"; return false; }
  if (want == QUAD_AUTO && !flying) { *why = "Take off first"; return false; }
  if (want == QUAD_LAND && !flying) { *why = "Already on the ground"; return false; }
  mode = want;
  if (want == QUAD_AUTO) { active = 0; target_alt = mission[0].alt_m; }
  if (want == QUAD_RTL) target_alt = params.rtl_alt_m;
  return true;
}

bool quad_load_mission(const quad_wp_t *items, uint16_t count, const char **why) {
  if (count > QUAD_MAX_WAYPOINTS) { *why = "Too many waypoints"; return false; }
  memcpy(mission, items, count * sizeof items[0]);
  mission_count = count;
  active = 0;
  return true;
}

/* ─── the accelerometer routine ────────────────────────────────────────────── */

bool quad_accel_cal_start(const char **why) {
  if (armed) { *why = "Disarm it first"; return false; }
  cal_running = true;
  cal_pose = 0;
  return true;
}

void quad_accel_cal_cancel(void) { cal_running = false; cal_pose = 0; }
uint8_t quad_accel_cal_pose(void) { return cal_pose; }
bool quad_accel_cal_running(void) { return cal_running; }

bool quad_accel_cal_accept(uint8_t *next_pose, float *quality) {
  if (!cal_running) return false;
  cal_pose++;
  *next_pose = cal_pose;
  if (cal_pose < ACCEL_POSE_COUNT) return false;
  cal_running = false;
  cal_pose = 0;
  *quality = 0.96f;
  return true;   /* done */
}

/* ─── flight ───────────────────────────────────────────────────────────────── */

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

/** Fly toward a point, and report true once it is close enough. */
static bool fly_to(double t_lat, double t_lon, float dt_s) {
  const double range = distance_m(lat, lon, t_lat, t_lon);
  if (range < 2.0) { ground_speed = 0.0f; roll = pitch = 0.0f; return true; }

  const double want = bearing_deg(lat, lon, t_lat, t_lon);
  double error = want - (double)heading;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  heading += (float)(error * 2.0 * (double)dt_s);
  if (heading < 0.0f) heading += 360.0f;
  if (heading >= 360.0f) heading -= 360.0f;

  ground_speed = params.wp_speed_ms;
  /* A multirotor leans to move, so the attitude the operator sees follows the demand. */
  pitch = (float)(-params.wp_speed_ms * (double)params.rate_p * 0.1);
  roll = (float)(error * (double)params.rate_p * 0.02);

  const double travelled = (double)ground_speed * (double)dt_s;
  lat += travelled * cos((double)heading * DEG) / EARTH_R / DEG;
  lon += travelled * sin((double)heading * DEG) / (EARTH_R * cos(lat * DEG)) / DEG;
  return false;
}

/** Move toward the height the current mode wants. */
static void hold_altitude(float dt_s) {
  const float error = target_alt - alt_rel;
  const float step = params.climb_speed_ms * dt_s;
  if (fabsf(error) < step) {
    alt_rel = target_alt;
    climb = 0.0f;
  } else {
    climb = error > 0.0f ? params.climb_speed_ms : -params.climb_speed_ms;
    alt_rel += climb * dt_s;
  }
}

void quad_step(float dt_s) {
  battery -= 0.0015f * dt_s * (flying ? 4.0f : 1.0f);
  if (battery < 12.0f) battery = 12.0f;

  if (!armed || !flying) {
    ground_speed = climb = 0.0f;
    roll = pitch = 0.0f;
    return;
  }

  switch (mode) {
    case QUAD_AUTO: {
      if (mission_count == 0) break;
      const quad_wp_t *target = &mission[active];
      target_alt = target->alt_m;
      if (fly_to(target->lat, target->lon, dt_s) && fabsf(alt_rel - target_alt) < 1.0f) {
        if (active + 1 < mission_count) {
          active++;
        } else {
          quad_set_mode(QUAD_RTL, &(const char *){NULL});  /* plan finished */
        }
      }
      break;
    }
    case QUAD_RTL:
      /* Climb to the return height first, then go home, then do what RTL_ACTION says. */
      if (alt_rel < params.rtl_alt_m - 1.0f) {
        target_alt = params.rtl_alt_m;
      } else if (fly_to(home_lat, home_lon, dt_s)) {
        if (params.rtl_action == 0) mode = QUAD_LAND;
        else if (params.rtl_action == 1) target_alt = params.rtl_alt_m;
      }
      break;
    case QUAD_LAND:
      target_alt = 0.0f;
      if (alt_rel < 0.2f) { flying = false; armed = false; mode = QUAD_STABILIZE; }
      break;
    default:
      ground_speed = 0.0f;
      roll = pitch = 0.0f;
      break;
  }

  hold_altitude(dt_s);
}
