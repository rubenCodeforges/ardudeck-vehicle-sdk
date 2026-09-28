#include <string.h>

#include "ad_internal.h"

#if AD_MAX_CALIBRATIONS > 0

static uint8_t cal_count(void) {
  uint8_t n = ad_g.cfg ? ad_g.cfg->calibration_count : 0;
  return n > AD_MAX_CALIBRATIONS ? AD_MAX_CALIBRATIONS : n;
}

static const ad_calibration_t *cal_at(uint8_t index) {
  return index < cal_count() ? &ad_g.cfg->calibrations[index] : NULL;
}

static const ad_calibration_t *cal_by_id(const char *id) {
  for (uint8_t i = 0; i < cal_count(); i++) {
    const ad_calibration_t *c = cal_at(i);
    if (c && c->id && strncmp(c->id, id, 16) == 0) return c;
  }
  return NULL;
}

static void send_declare(uint8_t index) {
  const ad_calibration_t *c = cal_at(index);
  if (!c) return;
  ad_tx_begin();
  ad_put_u8(index);
  ad_put_u8(cal_count());
  ad_put_u8((uint8_t)c->kind);
  ad_put_u8(c->requirements);
  ad_put_u8(c->pose_count);
  ad_put_u8(c->track_count > AD_CAL_MAX_TRACKS ? AD_CAL_MAX_TRACKS : c->track_count);
  ad_put_str(c->id, 16);
  ad_put_str(c->name ? c->name : c->id, 24);
  ad_put_str(c->warning ? c->warning : "", 80);
  ad_put_str(c->prompt ? c->prompt : "", 60);
  ad_tx_send(AD_MSG_CAL_DECLARE);
}

static void send_pose(uint8_t index, uint8_t pose_index) {
  const ad_calibration_t *c = cal_at(index);
  if (!c || !c->poses || pose_index >= c->pose_count) return;
  const ad_cal_pose_t *p = &c->poses[pose_index];
  ad_tx_begin();
  ad_put_f32(p->roll_deg);
  ad_put_f32(p->pitch_deg);
  ad_put_u8(index);
  ad_put_u8(pose_index);
  ad_put_str(c->id, 16);
  ad_put_str(p->name ? p->name : "", 24);
  ad_tx_send(AD_MSG_CAL_POSE);
}

static void send_track(uint8_t index, uint8_t track_index) {
  const ad_calibration_t *c = cal_at(index);
  if (!c || !c->tracks || track_index >= c->track_count) return;
  const ad_cal_track_t *t = &c->tracks[track_index];
  ad_tx_begin();
  ad_put_f32(t->needed);
  ad_put_u8(index);
  ad_put_u8(track_index);
  ad_put_str(c->id, 16);
  ad_put_str(t->unit ? t->unit : "", 12);
  ad_put_str(t->label ? t->label : "", 32);
  ad_tx_send(AD_MSG_CAL_TRACK);
}

uint8_t ad_cal_declare_steps(void) {
  uint8_t total = 0;
  for (uint8_t i = 0; i < cal_count(); i++) {
    const ad_calibration_t *c = cal_at(i);
    uint8_t tracks = c->track_count > AD_CAL_MAX_TRACKS ? AD_CAL_MAX_TRACKS
                                                        : c->track_count;
    total = (uint8_t)(total + 1 + c->pose_count + tracks);
  }
  return total;
}

/** Walk the whole declaration set one message at a time, so no link ever sees a burst. */
void ad_cal_declare_step(uint8_t step) {
  for (uint8_t i = 0; i < cal_count(); i++) {
    const ad_calibration_t *c = cal_at(i);
    uint8_t tracks = c->track_count > AD_CAL_MAX_TRACKS ? AD_CAL_MAX_TRACKS
                                                        : c->track_count;
    if (step == 0) { send_declare(i); return; }
    step--;
    if (step < c->pose_count) { send_pose(i, step); return; }
    step = (uint8_t)(step - c->pose_count);
    if (step < tracks) { send_track(i, step); return; }
    step = (uint8_t)(step - tracks);
  }
}

void ad_cal_reset(void) {
  ad_g.cal_active = NULL;
  ad_g.cal_last_progress = 0;
}

void ad_cal_handle(const uint8_t *p, uint16_t len) {
  char id[17];
  uint8_t action = ad_get_u8(p, len, 2);
  ad_get_str(p, len, 3, 16, id, sizeof id);

  const ad_calibration_t *c = cal_by_id(id);
  if (!c) {
    ad_statustext(AD_WARNING, "Unknown calibration requested");
    return;
  }

  /*
   * The declaration warns the operator; it never authorises anything. Refusing here is
   * what actually protects the aircraft, because the ground station may be older than
   * this firmware, or lying, or simply not the one that drew the screen.
   */
  if (action == AD_CAL_START && (c->requirements & AD_CAL_REQ_LOCAL_ONLY)) {
    ad_statustext(AD_WARNING, "This calibration must be started at the vehicle");
    return;
  }
  if (action == AD_CAL_START && (c->requirements & AD_CAL_REQ_DISARMED) && ad_g.armed) {
    ad_statustext(AD_WARNING, "Disarm before calibrating");
    return;
  }

  char why[64];
  why[0] = '\0';
  bool ok = true;
  if (ad_g.cfg->on_calibrate) {
    ok = ad_g.cfg->on_calibrate(c->id, (ad_cal_action_t)action, why, sizeof why,
                                ad_g.cfg->user);
  }

  if (!ok) {
    ad_statustext(AD_WARNING, why[0] ? why : "Calibration refused");
    return;
  }

  if (action == AD_CAL_START) {
    ad_g.cal_active = c;
    ad_g.cal_last_progress = 0;
  } else if (action == AD_CAL_CANCEL) {
    ad_g.cal_active = NULL;
  }
}

void ardudeck_cal_progress(const char *cal_id, const ad_cal_progress_t *progress) {
  if (!ad_g.begun || !cal_id || !progress) return;

  /* Routines produce progress far faster than anyone can read it, and a saturated link
     drops the heartbeat, which looks like the vehicle died mid-calibration. */
  uint32_t min_gap = 1000u / AD_CAL_PROGRESS_HZ;
  if (!progress->step_done && ad_g.cal_last_progress != 0 &&
      (uint32_t)(ad_g.now - ad_g.cal_last_progress) < min_gap) {
    return;
  }
  ad_g.cal_last_progress = ad_g.now ? ad_g.now : 1;

  ad_tx_begin();
  for (uint8_t i = 0; i < AD_CAL_MAX_TRACKS; i++) ad_put_f32(progress->track[i]);
  ad_put_u8(progress->step);
  ad_put_u8(progress->step_done ? 1 : 0);
  ad_put_u8(progress->percent);
  ad_put_str(cal_id, 16);
  ad_put_str(progress->hint ? progress->hint : "", 48);
  ad_tx_send(AD_MSG_CAL_PROGRESS);
}

void ardudeck_cal_done(const char *cal_id, bool ok, float quality, const char *detail) {
  if (!ad_g.begun || !cal_id) return;
  ad_tx_begin();
  ad_put_f32(quality);
  ad_put_u8(ok ? 1 : 0);
  ad_put_str(cal_id, 16);
  ad_put_str(detail ? detail : "", 64);
  ad_tx_send(AD_MSG_CAL_RESULT);
  ad_g.cal_active = NULL;
}

#else /* AD_MAX_CALIBRATIONS == 0 */

void ad_cal_reset(void) {}
void ad_cal_handle(const uint8_t *p, uint16_t len) { (void)p; (void)len; }
void ad_cal_declare_step(uint8_t step) { (void)step; }
uint8_t ad_cal_declare_steps(void) { return 0; }

void ardudeck_cal_progress(const char *cal_id, const ad_cal_progress_t *progress) {
  (void)cal_id; (void)progress;
}
void ardudeck_cal_done(const char *cal_id, bool ok, float quality, const char *detail) {
  (void)cal_id; (void)ok; (void)quality; (void)detail;
}

#endif
