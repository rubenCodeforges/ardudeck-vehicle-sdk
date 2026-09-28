/*
 * Compile-time sizing. Override any of these with -D, or by putting your own
 * ardudeck_conf.h earlier on the include path.
 *
 * Unused features compile out, so a telemetry-only build pays nothing for the mission
 * and calibration code.
 */

#ifndef ARDUDECK_CONF_H
#define ARDUDECK_CONF_H

/** Mission items the SDK will assemble. 0 compiles mission support out entirely. */
#ifndef AD_MAX_MISSION_ITEMS
#define AD_MAX_MISSION_ITEMS 64
#endif

/** Parameters the SDK will enumerate. 0 compiles parameter support out entirely. */
#ifndef AD_MAX_PARAMS
#define AD_MAX_PARAMS 128
#endif

/** Calibrations. 0 compiles calibration support out entirely. */
#ifndef AD_MAX_CALIBRATIONS
#define AD_MAX_CALIBRATIONS 8
#endif

#ifndef AD_WITH_COMMANDS
#define AD_WITH_COMMANDS 1
#endif

/** Reassembly buffer for one inbound frame. Never needs to exceed 280. */
#ifndef AD_RX_BUFFER
#define AD_RX_BUFFER 280
#endif

/** Silence after which ardudeck_linked() reports false. Your own failsafe should use
 *  ardudeck_silent_for() with its own number rather than this one. */
#ifndef AD_LINK_TIMEOUT_MS
#define AD_LINK_TIMEOUT_MS 5000
#endif

#ifndef AD_DEFAULT_SYSTEM_ID
#define AD_DEFAULT_SYSTEM_ID 1
#endif

/** 1 is MAV_COMP_ID_AUTOPILOT1, which is what a flight controller should be. */
#ifndef AD_DEFAULT_COMPONENT_ID
#define AD_DEFAULT_COMPONENT_ID 1
#endif

/**
 * How often the vehicle re-describes itself while nobody has answered.
 *
 * On a broadcast link there is no connect event, so a ground station that starts late
 * has to hear the manifest again. Once any traffic arrives the interval drops to
 * AD_MANIFEST_IDLE_MS, because by then somebody has it.
 */
#ifndef AD_MANIFEST_SEARCH_MS
#define AD_MANIFEST_SEARCH_MS 3000
#endif

#ifndef AD_MANIFEST_IDLE_MS
#define AD_MANIFEST_IDLE_MS 30000
#endif

/** Silence after which a transfer in progress is abandoned. */
#ifndef AD_TRANSFER_TIMEOUT_MS
#define AD_TRANSFER_TIMEOUT_MS 4000
#endif

/** Ceiling on outbound calibration progress, which firmware tends to produce far too fast. */
#ifndef AD_CAL_PROGRESS_HZ
#define AD_CAL_PROGRESS_HZ 5
#endif

#endif /* ARDUDECK_CONF_H */
