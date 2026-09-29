/*
 * A small rover firmware. This file knows nothing about ArduDeck or MAVLink, and that is
 * the point: it is what you already have before any of this starts.
 */
#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdbool.h>
#include <stdint.h>

#define ROVER_MAX_WAYPOINTS 16

typedef enum { ROVER_IDLE = 0, ROVER_MANUAL = 1, ROVER_AUTO = 2, ROVER_HOLD = 3 } rover_mode_t;

/** Everything the operator can change. A real firmware keeps this in flash. */
typedef struct {
  float cruise_speed_ms;   /* how fast it drives between waypoints */
  float arrive_radius_m;   /* how close counts as arrived */
  float heading_kp;        /* steering gain */
  float battery_low_v;     /* 0 disables the warning */
} rover_params_t;

typedef struct {
  double lat, lon;
} rover_wp_t;

void          rover_init(void);
/** Advance the simulation by `dt_s`. Call this from your main loop. */
void          rover_step(float dt_s);

rover_params_t *rover_params(void);
bool          rover_save_params(void);

double        rover_lat(void);
double        rover_lon(void);
float         rover_heading_deg(void);
float         rover_speed_ms(void);
float         rover_battery_v(void);
bool          rover_armed(void);
rover_mode_t  rover_mode(void);
uint16_t      rover_active_waypoint(void);

bool          rover_arm(bool arm, const char **why);
bool          rover_set_mode(rover_mode_t mode, const char **why);
bool          rover_load_mission(const rover_wp_t *items, uint16_t count, const char **why);
uint16_t      rover_mission_count(void);
const rover_wp_t *rover_mission(void);

#endif
