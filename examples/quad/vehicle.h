/*
 * A small multirotor firmware. Knows nothing about ArduDeck or MAVLink.
 *
 * It is a simulation, but the shape is the real one: a state estimate, a mode, a mission,
 * and a control loop that moves the aircraft toward where it should be.
 */
#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdbool.h>
#include <stdint.h>

#define QUAD_MAX_WAYPOINTS 32

typedef enum {
  QUAD_STABILIZE = 0, QUAD_ALTHOLD = 1, QUAD_LOITER = 2,
  QUAD_AUTO = 3, QUAD_RTL = 4, QUAD_LAND = 5, QUAD_FAILSAFE = 6,
} quad_mode_t;

/** What the operator can change. A real one keeps this in flash. */
typedef struct {
  float rate_p;          /* how hard it corrects an attitude error */
  float hover_throttle;  /* throttle that holds height */
  float wp_speed_ms;     /* horizontal speed between waypoints */
  float climb_speed_ms;  /* vertical speed in a climb */
  float rtl_alt_m;       /* height it climbs to before returning */
  uint8_t rtl_action;    /* 0 land, 1 hover and wait, 2 hold position */
} quad_params_t;

typedef struct {
  double lat, lon;
  float  alt_m;          /* relative to where it armed */
} quad_wp_t;

void          quad_init(void);
/** Advance the simulation by `dt_s`. Call this from your main loop. */
void          quad_step(float dt_s);

quad_params_t *quad_params(void);
bool          quad_save_params(void);

double        quad_lat(void);
double        quad_lon(void);
float         quad_roll_rad(void);
float         quad_pitch_rad(void);
float         quad_yaw_rad(void);
float         quad_alt_rel_m(void);
float         quad_alt_amsl_m(void);
float         quad_climb_ms(void);
float         quad_groundspeed_ms(void);
float         quad_heading_deg(void);
float         quad_battery_v(void);
bool          quad_armed(void);
bool          quad_flying(void);
quad_mode_t   quad_mode(void);
uint16_t      quad_active_waypoint(void);

/** Why it will not arm, or NULL when it will. */
const char   *quad_prearm_reason(void);

bool          quad_arm(bool arm, bool forced, const char **why);
bool          quad_takeoff(float alt_m, const char **why);
bool          quad_set_mode(quad_mode_t mode, const char **why);
bool          quad_load_mission(const quad_wp_t *items, uint16_t count, const char **why);
uint16_t      quad_mission_count(void);

/** The accelerometer routine, which needs the aircraft held in six orientations. */
bool          quad_accel_cal_start(const char **why);
void          quad_accel_cal_cancel(void);
/** The operator says the current pose is held. Returns true when the routine is done. */
bool          quad_accel_cal_accept(uint8_t *next_pose, float *quality);
uint8_t       quad_accel_cal_pose(void);
bool          quad_accel_cal_running(void);

#endif
