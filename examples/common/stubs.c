/*
 * Stand-ins for the firmware an example would be compiled into.
 *
 * Every example is built against this in CI, so a broken example fails the build instead
 * of failing quietly on somebody's bench. Delete it when you copy an example: these are
 * your own functions.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

uint32_t now_ms(void) { return 0; }
void link_write(const uint8_t *b, size_t n) { (void)b; (void)n; }

double gps_lat(void) { return 0; }
double gps_lon(void) { return 0; }
float  gps_speed_ms(void) { return 0; }
float  heading_deg(void) { return 0; }
int    gps_fix(void) { return 0; }
int    gps_sats(void) { return 0; }

float  altitude_amsl(void) { return 0; }
float  altitude_agl(void) { return 0; }
float  climb_ms(void) { return 0; }
float  depth_m(void) { return 0; }
float  roll_rad(void) { return 0; }
float  pitch_rad(void) { return 0; }
float  yaw_rad(void) { return 0; }

float  battery_volts(void) { return 0; }
float  battery_amps(void) { return -1; }
int    battery_percent(void) { return -1; }

int    flight_mode(void) { return 0; }
int    active_item(void) { return 0; }
bool   is_armed(void) { return false; }
bool   set_mode(int mode) { (void)mode; return true; }
bool   arm(bool on) { (void)on; return true; }
bool   start_mission(void) { return true; }
bool   return_home(void) { return true; }
bool   takeoff(float alt) { (void)alt; return true; }

bool   config_save(void) { return true; }
int    mission_store(const void *items, uint16_t count) { (void)items; (void)count; return 0; }
uint16_t mission_read(void *items, uint16_t capacity) { (void)items; (void)capacity; return 0; }

int    calibration_action(const char *id, int action) { (void)id; (void)action; return 0; }
