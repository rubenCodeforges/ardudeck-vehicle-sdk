/*
 * ArduDeck Vehicle SDK, the whole public contract.
 *
 * Declare what your vehicle is, wire four callbacks, call ardudeck_tick() from a loop you
 * already have. Everything on the wire is this library's problem.
 *
 * Under-declare, then grow. A capability you leave out is a screen the ground station
 * hides. A capability you overclaim is an operator uploading a plan you will not fly.
 *
 * Apache-2.0. Safe to link into closed firmware.
 */

#ifndef ARDUDECK_H
#define ARDUDECK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ardudeck_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AD_PROFILE_VERSION 1

/** Sentinel for any uint8_t reading the vehicle does not have. */
#define AD_UNKNOWN 255u

/** Count the entries of a table declared in this translation unit. */
#define AD_COUNT(a) ((uint16_t)(sizeof(a) / sizeof((a)[0])))

/* ─── what the vehicle is ──────────────────────────────────────────────────── */

typedef enum {
  AD_FRAME_UNKNOWN = 0,
  AD_FRAME_MULTIROTOR = 1,
  AD_FRAME_FIXED_WING = 2,
  AD_FRAME_VTOL = 3,
  AD_FRAME_HELICOPTER = 4,
  AD_FRAME_ROVER = 5,
  AD_FRAME_SURFACE_BOAT = 6,
  AD_FRAME_SUBMARINE = 7,
  AD_FRAME_ANTENNA_TRACKER = 8,
} ad_frame_t;

#define AD_FEAT_PARAMS         (1u << 0)
#define AD_FEAT_MISSION        (1u << 1)
#define AD_FEAT_COMMANDS       (1u << 2)
#define AD_FEAT_HOME           (1u << 3)
#define AD_FEAT_CALIBRATION    (1u << 5)
#define AD_FEAT_TERRAIN        (1u << 8)
#define AD_FEAT_MISSION_READ   (1u << 9)
#define AD_FEAT_RC_REPORT      (1u << 10)

/*
 * Reserved. The bit numbers are fixed so they never move, but this profile version
 * defines no protocol for them and the SDK implements none of it. Setting one advertises
 * a screen that cannot work, which is the exact failure the whole design exists to
 * prevent, so the conformance tool reports it.
 */
#define AD_FEAT_GEOFENCE       (1u << 4)  /* reserved, not implemented */
#define AD_FEAT_MANUAL_CONTROL (1u << 6)  /* reserved, not implemented */
#define AD_FEAT_LOG_DOWNLOAD   (1u << 7)  /* reserved, not implemented */

#define AD_MODE_LOCAL_ONLY (1u << 0)
#define AD_MODE_ARMED_ONLY (1u << 1)
#define AD_MODE_TERMINAL   (1u << 2)

typedef struct {
  uint16_t    id;    /**< what this firmware reports as its mode */
  const char *name;  /**< up to 16 characters, shown in the picker */
  uint8_t     flags;
} ad_mode_t;

typedef struct {
  const char *vendor;
  const char *model;
  const char *firmware;
  const char *uid;   /**< stable per airframe, or NULL */
  ad_frame_t  frame;
  uint32_t    features;

  const uint16_t *mission_cmds;      /**< MAV_CMD values honoured, at most 32 */
  uint16_t        mission_cmd_count;
  uint16_t        mission_capacity;

  const ad_mode_t *modes;
  uint8_t          mode_count;
} ad_capability_t;

/* ─── parameters ───────────────────────────────────────────────────────────── */

typedef enum { AD_T_F32 = 0, AD_T_I32 = 1, AD_T_U8 = 2 } ad_ptype_t;

#define AD_PARAM_REBOOT   (1u << 0)
#define AD_PARAM_READONLY (1u << 1)
#define AD_PARAM_OPTIONS  (1u << 2)
#define AD_PARAM_ADVANCED (1u << 3)

typedef struct ad_param ad_param_t;

struct ad_param {
  const char *name;    /**< up to 16 chars, A-Z 0-9 _. Renaming breaks every saved file. */
  const char *unit;    /**< "m/s", "%", or "" for a pure gain */
  const char *help;    /**< one line: what changing it does */
  void       *storage; /**< NULL when accessors are used instead */
  const char *const *options;
  bool (*get)(const ad_param_t *p, float *out);
  bool (*set)(const ad_param_t *p, float value);
  float   min_value;
  float   max_value;
  float   increment;   /**< 0 for continuous */
  uint8_t type;
  uint8_t flags;
  uint8_t option_count;
};

#define AD_F32(NAME, PTR, MIN, MAX, UNIT, HELP)                                        \
  { .name = (NAME), .unit = (UNIT), .help = (HELP), .storage = (PTR),                  \
    .min_value = (MIN), .max_value = (MAX), .type = AD_T_F32 }

#define AD_I32(NAME, PTR, MIN, MAX, UNIT, HELP)                                        \
  { .name = (NAME), .unit = (UNIT), .help = (HELP), .storage = (PTR),                  \
    .min_value = (float)(MIN), .max_value = (float)(MAX), .increment = 1.0f,           \
    .type = AD_T_I32 }

#define AD_U8(NAME, PTR, MIN, MAX, UNIT, HELP)                                         \
  { .name = (NAME), .unit = (UNIT), .help = (HELP), .storage = (PTR),                  \
    .min_value = (float)(MIN), .max_value = (float)(MAX), .increment = 1.0f,           \
    .type = AD_T_U8 }

/** A value read once at boot. The editor offers a reboot after writing it. */
#define AD_F32_REBOOT(NAME, PTR, MIN, MAX, UNIT, HELP)                                 \
  { .name = (NAME), .unit = (UNIT), .help = (HELP), .storage = (PTR),                  \
    .min_value = (MIN), .max_value = (MAX), .type = AD_T_F32,                          \
    .flags = AD_PARAM_REBOOT }

/** Named choices, so the operator picks a word instead of looking up a number. */
#define AD_ENUM(NAME, GET, SET, OPTIONS, COUNT, HELP)                                  \
  { .name = (NAME), .unit = "", .help = (HELP), .options = (OPTIONS),                  \
    .option_count = (COUNT), .get = (GET), .set = (SET),                               \
    .min_value = 0.0f, .max_value = (float)((COUNT) - 1), .increment = 1.0f,           \
    .type = AD_T_U8, .flags = AD_PARAM_OPTIONS }

/* ─── missions ─────────────────────────────────────────────────────────────── */

#define AD_NAV_WAYPOINT          16
#define AD_NAV_LOITER_UNLIM      17
#define AD_NAV_LOITER_TIME       19
#define AD_NAV_RETURN_TO_LAUNCH  20
#define AD_NAV_LAND              21
#define AD_NAV_TAKEOFF           22
#define AD_NAV_SPLINE_WAYPOINT   82
#define AD_DO_SET_SPEED         178
#define AD_DO_SET_SERVO         183
#define AD_DO_DIGICAM_CONTROL   203

typedef enum {
  AD_ALT_ASL = 0,      /**< above mean sea level */
  AD_ALT_RELATIVE = 1, /**< above home */
  AD_ALT_TERRAIN = 2,  /**< above ground, needs AD_FEAT_TERRAIN */
} ad_alt_frame_t;

typedef struct {
  uint16_t seq;
  uint16_t command;
  uint8_t  alt_frame;
  double   lat;
  double   lon;
  float    alt; /**< metres, in alt_frame */
  float    p1, p2, p3, p4;
} ad_wp_t;

/* ─── commands ─────────────────────────────────────────────────────────────── */

/*
 * Commands a ground station actually sends, and nothing else.
 *
 * The arguments below are what YOUR callback receives, which is not always where MAVLink
 * puts them on the wire: DO_SET_MODE carries the mode in param2 behind a base-mode
 * bitmask in param1, and NAV_TAKEOFF carries altitude in param7. The SDK unpacks both, so
 * a[0] means what this list says it means.
 */
#define AD_CMD_ARM            400  /* a[0] 1 arm, 0 disarm. a[1] 21196 means forced. */
#define AD_CMD_SET_MODE       176  /* a[0] a mode id from YOUR table */
#define AD_CMD_RETURN_HOME     20  /* no arguments */
#define AD_CMD_TAKEOFF         22  /* a[0] altitude in metres */
#define AD_CMD_START_MISSION  300  /* no arguments; see the note in docs/api.md */

/* ─── calibration ──────────────────────────────────────────────────────────── */

typedef enum {
  AD_CAL_POSITIONAL = 0, /**< hold each named attitude until accepted */
  AD_CAL_COVERAGE = 1,   /**< move until every track reaches its target */
  AD_CAL_SWEEP = 2,      /**< drive one input through its range */
  AD_CAL_INSTANT = 3,    /**< hold still and wait */
} ad_cal_kind_t;

/* These warn the operator. They never authorise: the vehicle is the only gate. */
#define AD_CAL_REQ_DISARMED      (1u << 0)
#define AD_CAL_REQ_STATIONARY    (1u << 1)
#define AD_CAL_REQ_PROPS_OFF     (1u << 2)
#define AD_CAL_REQ_MOTORS_LIVE   (1u << 3)
#define AD_CAL_REQ_LOCAL_ONLY    (1u << 4)
#define AD_CAL_REQ_LEVEL_SURFACE (1u << 5)

typedef enum {
  AD_CAL_START = 0,
  AD_CAL_ACCEPT = 1, /**< the operator says the pose is held */
  AD_CAL_CANCEL = 2, /**< must always be honoured, even mid-routine */
  AD_CAL_SAVE = 3,
} ad_cal_action_t;

typedef struct {
  const char *name; /**< the instruction, for example "Nose down" */
  float roll_deg;
  float pitch_deg;
} ad_cal_pose_t;

typedef struct {
  const char *label; /**< shown beside the progress bar */
  const char *unit;  /**< "deg", "%", or "" for a count or ratio */
  float       needed;
} ad_cal_track_t;

#define AD_CAL_MAX_TRACKS 3

typedef struct {
  const char   *id;   /**< "compass", "accel", or your own. Unknown ids still render. */
  const char   *name; /**< screen title */
  ad_cal_kind_t kind;
  uint8_t       requirements;
  const char   *warning; /**< shown before Start, in your words */
  const char   *prompt;  /**< what to move, for AD_CAL_SWEEP */

  const ad_cal_pose_t *poses;
  uint8_t              pose_count;

  const ad_cal_track_t *tracks;
  uint8_t               track_count;
} ad_calibration_t;

typedef struct {
  uint8_t step;      /**< pose being waited for, for AD_CAL_POSITIONAL */
  bool    step_done; /**< that pose was just captured */
  float   track[AD_CAL_MAX_TRACKS];
  uint8_t percent;   /**< 0 to 100, or AD_UNKNOWN */
  const char *hint;  /**< optional coaching, or NULL */
} ad_cal_progress_t;

/* ─── messages to the operator ─────────────────────────────────────────────── */

typedef enum {
  AD_CRITICAL = 2,
  AD_WARNING = 4,
  AD_NOTICE = 5,
  AD_INFO = 6,
} ad_severity_t;

/* ─── setup ────────────────────────────────────────────────────────────────── */

/**
 * Everything the SDK needs from you.
 *
 * Every pointer here must outlive ardudeck_begin(). Nothing is copied, because copying
 * would mean allocating, and this library never does.
 */
typedef struct {
  const ad_capability_t *caps;

  const ad_param_t *params;
  uint16_t          param_count;

  const ad_calibration_t *calibrations;
  uint8_t                 calibration_count;

  /** Put these bytes on your link. May be called from tick() or receive(). */
  void (*send)(const uint8_t *buf, size_t len, void *user);

  /** Monotonic milliseconds. May wrap; the SDK handles it. */
  uint32_t (*now_ms)(void);

  /** Persist. Return false and the operator sees the field revert. */
  bool (*on_param_set)(const ad_param_t *p, float value, void *user);

  /** The whole plan at once, ordered, gap free, already filtered to mission_cmds. */
  bool (*on_mission)(const ad_wp_t *items, uint16_t count, char *why, size_t why_len,
                     void *user);

  /** Fill items for download. Return how many you wrote, or 0. */
  uint16_t (*on_mission_read)(ad_wp_t *items, uint16_t capacity, void *user);

  /** Acknowledged as soon as you return, so return before you reboot or block. */
  bool (*on_command)(uint16_t cmd, const float args[7], char *why, size_t why_len,
                     void *user);

  bool (*on_calibrate)(const char *cal_id, ad_cal_action_t action, char *why,
                       size_t why_len, void *user);

  void    *user;
  uint8_t  system_id;    /**< 0 uses AD_DEFAULT_SYSTEM_ID */
  uint8_t  component_id; /**< 0 uses AD_DEFAULT_COMPONENT_ID */
} ad_config_t;

void ardudeck_begin(const ad_config_t *cfg);

/** Feed inbound bytes. Any size, any split, including one byte at a time. */
void ardudeck_receive(const uint8_t *buf, size_t len);

/** Call at 20 Hz or faster. Sends what is due and runs the handshakes. */
void ardudeck_tick(uint32_t now_ms);

/* ─── state ────────────────────────────────────────────────────────────────── */

/** fix: 0 none, 2 two dimensional, 3 three dimensional. Absent is not zero. */
void ardudeck_position(double lat, double lon, float speed_ms, float heading_deg,
                       uint8_t fix, uint8_t sats);
void ardudeck_altitude(float amsl_m, float relative_m, float climb_ms);
void ardudeck_attitude(float roll, float pitch, float yaw); /**< radians */
void ardudeck_status(uint16_t mode, uint16_t active_item, float battery_v, bool armed);
void ardudeck_battery(float volts, float amps, int8_t percent); /**< -1 unknown */
void ardudeck_home(double lat, double lon, float alt);
void ardudeck_rc(const uint16_t *channels, uint8_t count, uint8_t rssi);

/** Unprompted words for the operator's message panel. */
void ardudeck_notify(ad_severity_t severity, const char *text);

/**
 * Tell the editor a value changed under it.
 *
 * A calibration that writes six offsets and says nothing leaves a stale screen, and the
 * operator's next save writes the old values back over the new ones.
 */
void ardudeck_param_changed(const char *name);

void ardudeck_cal_progress(const char *cal_id, const ad_cal_progress_t *progress);
void ardudeck_cal_done(const char *cal_id, bool ok, float quality, const char *detail);

/**
 * Cap a stream your link cannot carry.
 *
 * A 900 MHz modem at 19200 baud cannot serve 50 Hz attitude, and the honest answer is a
 * refused rate rather than a saturated link that drops heartbeats.
 */
typedef enum {
  AD_STREAM_POSITION = 0,
  AD_STREAM_ATTITUDE = 1,
  AD_STREAM_STATUS = 2,
  AD_STREAM_RC = 3,
  AD_STREAM_COUNT = 4,
} ad_stream_t;

void ardudeck_rate_limit(ad_stream_t stream, float max_hz);

/** True once any ground station has been heard from. Never required, sometimes useful. */
bool ardudeck_linked(void);

#ifdef __cplusplus
}
#endif

#endif /* ARDUDECK_H */
