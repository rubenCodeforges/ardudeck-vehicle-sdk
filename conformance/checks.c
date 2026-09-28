#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ad_msg_defs.h"
#include "conform.h"

/* ─── plumbing ─────────────────────────────────────────────────────────────── */

static param_info_t *param_by_index(vehicle_t *v, uint16_t index) {
  for (int i = 0; i < v->params_seen; i++) {
    if (v->params[i].index == index) return &v->params[i];
  }
  if (v->params_seen >= MAX_PARAMS) return NULL;
  param_info_t *p = &v->params[v->params_seen++];
  memset(p, 0, sizeof *p);
  p->index = index;
  return p;
}

static cal_info_t *cal_by_id(vehicle_t *v, const char *id) {
  for (int i = 0; i < v->cals_seen; i++) {
    if (strcmp(v->cals[i].id, id) == 0) return &v->cals[i];
  }
  if (v->cals_seen >= MAX_CALS) return NULL;
  cal_info_t *c = &v->cals[v->cals_seen++];
  memset(c, 0, sizeof *c);
  snprintf(c->id, sizeof c->id, "%s", id);
  return c;
}

static void absorb(session_t *s, const mav_msg_t *m) {
  vehicle_t *v = &s->v;
  v->sysid = m->sysid;
  v->compid = m->compid;

  if (m->id == AD_MSG_HEARTBEAT.id) {
    v->heartbeats++;
    v->vehicle_type = mav_u8(m, 4);
    v->autopilot = mav_u8(m, 5);
    uint32_t t = now_ms();
    if (!v->first_heartbeat_ms) v->first_heartbeat_ms = t;
    v->last_heartbeat_ms = t;
    return;
  }

  if (m->id == AD_MSG_GPS_RAW_INT.id) {
    v->gps_raws++;
    v->fix = mav_u8(m, 28);
    return;
  }

  if (m->id == AD_MSG_GLOBAL_POSITION_INT.id) {
    v->positions++;
    v->last_lat = mav_i32(m, 4);
    v->last_lon = mav_i32(m, 8);
    if (v->gps_raws > 0 && v->fix < 2) v->position_without_fix = true;
    if (v->last_lat == 0 && v->last_lon == 0) v->zero_island = true;
    return;
  }

  if (m->id == AD_MSG_ATTITUDE.id) { v->attitudes++; return; }
  if (m->id == AD_MSG_SYS_STATUS.id) { v->sys_status++; return; }

  if (m->id == AD_MSG_MANIFEST.id) {
    v->have_manifest = true;
    v->features = mav_u32(m, 0);
    v->profile_version = mav_u16(m, 4);
    v->mission_capacity = mav_u16(m, 6);
    v->param_count = mav_u16(m, 8);
    v->frame = mav_u8(m, 10);
    v->mode_count = mav_u8(m, 11);
    v->mission_cmd_count = mav_u8(m, 12);
    v->cal_count = mav_u8(m, 13);
    mav_str(m, 14, 20, v->vendor, sizeof v->vendor);
    mav_str(m, 34, 20, v->model, sizeof v->model);
    mav_str(m, 54, 16, v->firmware, sizeof v->firmware);
    mav_str(m, 70, 24, v->uid, sizeof v->uid);
    return;
  }

  if (m->id == AD_MSG_MODE.id) {
    uint8_t index = mav_u8(m, 2);
    if (index >= MAX_MODES) return;
    mode_info_t *mode = &v->modes[index];
    mode->id = mav_u16(m, 0);
    mode->flags = mav_u8(m, 4);
    mav_str(m, 5, 16, mode->name, sizeof mode->name);
    if (!mode->seen) { mode->seen = true; v->modes_seen++; }
    return;
  }

  if (m->id == AD_MSG_MISSION_CMDS.id) {
    v->have_mission_cmds = true;
    v->mission_cmds_seen = mav_u8(m, 64);
    if (v->mission_cmds_seen > 32) v->mission_cmds_seen = 32;
    for (int i = 0; i < v->mission_cmds_seen; i++) {
      v->mission_cmds[i] = mav_u16(m, (uint16_t)(i * 2));
    }
    return;
  }

  if (m->id == AD_MSG_PARAM_VALUE.id) {
    param_info_t *p = param_by_index(v, mav_u16(m, 6));
    if (!p) return;
    p->value = mav_f32(m, 0);
    p->have_value = true;
    mav_str(m, 8, 16, p->name, sizeof p->name);
    return;
  }

  if (m->id == AD_MSG_PARAM_META.id) {
    param_info_t *p = param_by_index(v, mav_u16(m, 12));
    if (!p) return;
    p->have_meta = true;
    p->min_value = mav_f32(m, 0);
    p->max_value = mav_f32(m, 4);
    p->increment = mav_f32(m, 8);
    p->flags = mav_u8(m, 14);
    p->option_count = mav_u8(m, 15);
    mav_str(m, 16, 16, p->name, sizeof p->name);
    mav_str(m, 32, 12, p->unit, sizeof p->unit);
    mav_str(m, 44, 80, p->help, sizeof p->help);
    return;
  }

  if (m->id == AD_MSG_PARAM_OPTION.id) {
    param_info_t *p = param_by_index(v, mav_u16(m, 0));
    if (p) p->options_seen++;
    return;
  }

  if (m->id == AD_MSG_CAL_DECLARE.id) {
    char id[17];
    mav_str(m, 6, 16, id, sizeof id);
    cal_info_t *c = cal_by_id(v, id);
    if (!c) return;
    c->seen = true;
    c->kind = mav_u8(m, 2);
    c->requirements = mav_u8(m, 3);
    c->pose_count = mav_u8(m, 4);
    c->track_count = mav_u8(m, 5);
    mav_str(m, 22, 24, c->name, sizeof c->name);
    return;
  }

  if (m->id == AD_MSG_CAL_POSE.id) {
    char id[17];
    mav_str(m, 10, 16, id, sizeof id);
    cal_info_t *c = cal_by_id(v, id);
    if (c) c->poses_seen++;
    return;
  }

  if (m->id == AD_MSG_CAL_TRACK.id) {
    char id[17];
    mav_str(m, 6, 16, id, sizeof id);
    cal_info_t *c = cal_by_id(v, id);
    if (c) c->tracks_seen++;
    return;
  }
}

void pump(session_t *s, int ms) {
  uint32_t deadline = now_ms() + (uint32_t)ms;
  uint8_t buf[1024];
  mav_msg_t msg;

  while ((int32_t)(now_ms() - deadline) < 0) {
    int n = link_recv(s->link, buf, sizeof buf, 20);
    if (n <= 0) continue;
    for (int i = 0; i < n; i++) {
      if (mav_feed(&s->parser, buf[i], &msg)) absorb(s, &msg);
    }
  }
}

bool wait_for(session_t *s, uint32_t msgid, int timeout_ms, mav_msg_t *out) {
  uint32_t deadline = now_ms() + (uint32_t)timeout_ms;
  uint8_t buf[1024];
  mav_msg_t msg;

  while ((int32_t)(now_ms() - deadline) < 0) {
    int n = link_recv(s->link, buf, sizeof buf, 20);
    if (n <= 0) continue;
    for (int i = 0; i < n; i++) {
      if (!mav_feed(&s->parser, buf[i], &msg)) continue;
      absorb(s, &msg);
      if (msg.id == msgid) {
        if (out) *out = msg;
        return true;
      }
    }
  }
  return false;
}

void send_msg(session_t *s, uint32_t id, const uint8_t *payload, uint8_t len) {
  uint8_t extra = 0;
  if (!mav_extra(id, &extra)) return;
  uint8_t frame[300];
  size_t n = mav_build(frame, id, extra, payload, len, s->seq++, s->sysid, s->compid);
  link_send(s->link, frame, n);
}

/* ─── rung 0, position ─────────────────────────────────────────────────────── */

void check_rung0(session_t *s, rung_t *r) {
  vehicle_t *v = &s->v;
  pump(s, 5000);

  if (v->heartbeats == 0) {
    rung_fail(r, "no heartbeat in five seconds");
    rung_summary(r, "silent");
    return;
  }

  float span_s = (float)(v->last_heartbeat_ms - v->first_heartbeat_ms) / 1000.0f;
  float hz = span_s > 0.5f ? (float)(v->heartbeats - 1) / span_s : 0.0f;
  if (hz > 0.0f && (hz < 0.5f || hz > 2.0f)) {
    rung_fail(r, "heartbeat is %.1f Hz, the profile says 1 Hz", (double)hz);
  }

  /* Claiming ArduPilot or PX4 makes a ground station apply that firmware's
     conventions to a vehicle that does not share them. */
  if (v->autopilot == 3 || v->autopilot == 12) {
    rung_fail(r, "heartbeat claims autopilot %u; a third-party vehicle reports 0",
              v->autopilot);
  }

  if (v->attitudes == 0) rung_warn(r, "no ATTITUDE, so the horizon will not move");
  if (v->gps_raws == 0 && v->positions == 0) {
    rung_fail(r, "no position of any kind");
  }
  if (v->sys_status == 0) rung_warn(r, "no SYS_STATUS, so there is no battery reading");

  if (v->position_without_fix) {
    rung_fail(r, "sends a position while reporting no fix; absent is not zero");
  }
  if (v->zero_island) {
    rung_fail(r, "reported latitude 0, longitude 0, which is a real place in the sea");
  }

  rung_summary(r, "heartbeat %.1f Hz, %d position, %d attitude, fix %u", (double)hz,
               v->positions + v->gps_raws, v->attitudes, v->fix);
}

/* ─── rung 1, identity ─────────────────────────────────────────────────────── */

static void request(session_t *s, uint8_t what, uint16_t index) {
  uint8_t p[5];
  memset(p, 0, sizeof p);
  mav_put_u16(&p[0], index);
  p[2] = s->v.sysid ? s->v.sysid : 1;
  p[3] = s->v.compid ? s->v.compid : 1;
  p[4] = what;
  send_msg(s, AD_MSG_REQUEST.id, p, sizeof p);
}

void check_rung1(session_t *s, rung_t *r) {
  vehicle_t *v = &s->v;

  if (!v->have_manifest) {
    request(s, 0, 0xFFFF);
    pump(s, 4000);
  }

  if (!v->have_manifest) {
    rung_fail(r, "no manifest, so the vehicle never says what it is");
    rung_summary(r, "undeclared");
    return;
  }

  if (v->profile_version == 0 || v->profile_version > 1) {
    rung_warn(r, "profile version %u, this tool knows 1", v->profile_version);
  }
  if (v->vendor[0] == '\0') rung_fail(r, "vendor is empty");
  if (v->model[0] == '\0') rung_fail(r, "model is empty");
  if (v->firmware[0] == '\0') rung_warn(r, "firmware version is empty");
  if (v->frame == 0) rung_warn(r, "frame is AD_FRAME_UNKNOWN, so the icon is a guess");

  pump(s, 2000);
  if (v->modes_seen < v->mode_count) {
    rung_fail(r, "manifest promises %u modes, %d arrived", v->mode_count,
              v->modes_seen);
  }
  for (int i = 0; i < v->mode_count && i < MAX_MODES; i++) {
    if (v->modes[i].seen && v->modes[i].name[0] == '\0') {
      rung_fail(r, "mode %u has no name, so the picker shows a number", v->modes[i].id);
    }
  }
  for (int i = 0; i < v->mode_count && i < MAX_MODES; i++) {
    for (int j = i + 1; j < v->mode_count && j < MAX_MODES; j++) {
      if (v->modes[i].seen && v->modes[j].seen && v->modes[i].id == v->modes[j].id) {
        rung_fail(r, "two modes share id %u", v->modes[i].id);
      }
    }
  }

  if (v->features & AD_FEAT_MISSION_BIT) {
    if (!v->have_mission_cmds || v->mission_cmds_seen == 0) {
      rung_fail(r, "declares missions but lists no mission commands");
    }
    if (v->mission_capacity == 0) {
      rung_fail(r, "declares missions but a capacity of zero");
    }
  }
  if ((v->features & AD_FEAT_PARAMS_BIT) && v->param_count == 0) {
    rung_fail(r, "declares parameters but the table is empty");
  }
  if ((v->features & AD_FEAT_CAL_BIT) && v->cal_count == 0) {
    rung_fail(r, "declares calibration but no calibrations");
  }

  /* Advertising a screen that cannot work is the failure this whole contract exists to
     prevent, so it is a failure here rather than a note. */
  uint32_t reserved = v->features & AD_FEAT_RESERVED_BITS;
  if (reserved) {
    rung_fail(r, "declares feature bits 0x%x, which this profile version defines no "
                 "protocol for. The operator gets a screen that cannot work.", reserved);
  }

  rung_summary(r, "%s %s fw %s, %d modes named, %d mission commands", v->vendor,
               v->model, v->firmware, v->modes_seen, v->mission_cmds_seen);
}

/* ─── rung 2, parameters ───────────────────────────────────────────────────── */

void check_rung2(session_t *s, rung_t *r) {
  vehicle_t *v = &s->v;

  if (!(v->features & AD_FEAT_PARAMS_BIT)) {
    r->result = R_SKIP;
    rung_summary(r, "not declared");
    return;
  }

  uint8_t req[2] = {v->sysid ? v->sysid : 1, v->compid ? v->compid : 1};
  send_msg(s, AD_MSG_PARAM_REQUEST_LIST.id, req, sizeof req);

  /* One message per tick is the recommended pacing, so a big table takes a while. */
  int budget = 4000 + v->param_count * 60;
  pump(s, budget);

  int with_value = 0, with_meta = 0, no_help = 0, no_unit = 0, bad_range = 0;
  int missing_options = 0;
  char first_no_help[3][17] = {{0}};
  int named = 0;

  for (int i = 0; i < v->params_seen; i++) {
    param_info_t *p = &v->params[i];
    if (p->have_value) with_value++;
    if (!p->have_meta) continue;
    with_meta++;

    if (p->help[0] == '\0') {
      if (named < 3) snprintf(first_no_help[named++], 17, "%s", p->name);
      no_help++;
    }
    /* An empty unit is legitimate for a pure gain, so this is a warning. */
    if (p->unit[0] == '\0') no_unit++;
    if (!(p->min_value < p->max_value)) bad_range++;
    if ((p->flags & 0x04) && p->options_seen < p->option_count) missing_options++;
  }

  if (with_value < v->param_count) {
    rung_fail(r, "manifest promises %u parameters, %d values arrived", v->param_count,
              with_value);
  }
  if (with_meta < with_value) {
    rung_fail(r, "%d parameters arrived without metadata, so they are numbers in a list",
              with_value - with_meta);
  }
  if (no_help) {
    rung_fail(r, "%d parameters have no description%s%s%s%s", no_help,
              named ? " -> " : "", named > 0 ? first_no_help[0] : "",
              named > 1 ? ", " : "", named > 1 ? first_no_help[1] : "");
  }
  if (bad_range) {
    rung_fail(r, "%d parameters have a range where min is not below max", bad_range);
  }
  if (missing_options) {
    rung_fail(r, "%d enum parameters never sent all their option labels",
              missing_options);
  }
  if (no_unit) {
    rung_warn(r, "%d parameters have no unit; correct for a pure gain, wrong otherwise",
              no_unit);
  }

  /* Write a value back to itself: proves the write path answers, changes nothing. */
  if (with_value > 0) {
    param_info_t *p = NULL;
    for (int i = 0; i < v->params_seen; i++) {
      if (v->params[i].have_value && !(v->params[i].flags & 0x02)) {
        p = &v->params[i];
        break;
      }
    }
    if (p) {
      uint8_t set[23];
      memset(set, 0, sizeof set);
      mav_put_f32(&set[0], p->value);
      set[4] = v->sysid ? v->sysid : 1;
      set[5] = v->compid ? v->compid : 1;
      memcpy(&set[6], p->name, strlen(p->name));
      set[22] = 9;
      p->have_value = false;
      send_msg(s, AD_MSG_PARAM_SET.id, set, sizeof set);
      if (!wait_for(s, AD_MSG_PARAM_VALUE.id, 2000, NULL)) {
        rung_fail(r, "a write to %s was never answered, so the editor cannot tell "
                     "whether it took", p->name);
      }
    }
  }

  rung_summary(r, "%d parameters, %d with metadata", with_value, with_meta);
}

/* ─── rung 3, missions ─────────────────────────────────────────────────────── */

static void mission_item(uint8_t *p, uint16_t seq, uint16_t cmd, uint8_t sysid,
                         uint8_t compid, double lat, double lon) {
  memset(p, 0, 37);
  mav_put_i32(&p[16], (int32_t)(lat * 1e7));
  mav_put_i32(&p[20], (int32_t)(lon * 1e7));
  mav_put_f32(&p[24], 30.0f);
  mav_put_u16(&p[28], seq);
  mav_put_u16(&p[30], cmd);
  p[32] = sysid;
  p[33] = compid;
  p[34] = 3; /* GLOBAL_RELATIVE_ALT */
  p[35] = seq == 0 ? 1 : 0;
  p[36] = 1;
}

/**
 * Wait for whichever of two messages arrives first.
 *
 * A mission transfer alternates between a request for the next item and a final
 * acknowledgement, and waiting for only one of them throws the other away, which
 * deadlocks the transfer. That bug is easy to write and hard to see.
 */
static bool wait_either(session_t *s, uint32_t a, uint32_t b, int timeout_ms,
                        mav_msg_t *out) {
  uint32_t deadline = now_ms() + (uint32_t)timeout_ms;
  uint8_t buf[1024];
  mav_msg_t msg;

  while ((int32_t)(now_ms() - deadline) < 0) {
    int n = link_recv(s->link, buf, sizeof buf, 20);
    if (n <= 0) continue;
    for (int i = 0; i < n; i++) {
      if (!mav_feed(&s->parser, buf[i], &msg)) continue;
      absorb(s, &msg);
      if (msg.id == a || msg.id == b) {
        *out = msg;
        return true;
      }
    }
  }
  return false;
}

/** Upload a plan and return the MISSION_ACK result, or -1 when nothing answered. */
static int upload(session_t *s, uint16_t count, uint16_t cmd, bool skip_one,
                  rung_t *r) {
  uint8_t sysid = s->v.sysid ? s->v.sysid : 1;
  uint8_t compid = s->v.compid ? s->v.compid : 1;

  uint8_t hdr[4];
  memset(hdr, 0, sizeof hdr);
  mav_put_u16(&hdr[0], count);
  hdr[2] = sysid;
  hdr[3] = compid;
  send_msg(s, AD_MSG_MISSION_COUNT.id, hdr, sizeof hdr);

  bool skipped_once = false;
  for (int guard = 0; guard < (int)count * 4 + 12; guard++) {
    mav_msg_t m;
    if (!wait_either(s, AD_MSG_MISSION_REQUEST_INT.id, AD_MSG_MISSION_ACK.id, 3000,
                     &m)) {
      return -1;
    }
    if (m.id == AD_MSG_MISSION_ACK.id) return mav_u8(&m, 2);

    uint16_t want = mav_u16(&m, 0);

    if (skip_one && !skipped_once && want == 0 && count > 1) {
      /* Answer once with the wrong item. A lost frame is ordinary, and the vehicle
         should ask again rather than abandon the whole transfer. */
      skipped_once = true;
      uint8_t wrong[37];
      mission_item(wrong, 1, cmd, sysid, compid, 52.5, 13.4);
      send_msg(s, AD_MSG_MISSION_ITEM_INT.id, wrong, sizeof wrong);
      if (!wait_either(s, AD_MSG_MISSION_REQUEST_INT.id, AD_MSG_MISSION_ACK.id, 2000,
                       &m) ||
          m.id != AD_MSG_MISSION_REQUEST_INT.id) {
        rung_fail(r, "an out-of-order item ended the transfer; a lost frame should be "
                     "re-requested, not fatal");
        return -1;
      }
      want = mav_u16(&m, 0);
      if (want != 0) {
        rung_fail(r, "after an out-of-order item the vehicle asked for %u, but it is "
                     "still missing 0", want);
        return -1;
      }
    }

    uint8_t item[37];
    mission_item(item, want, cmd, sysid, compid, 52.5 + want * 0.001, 13.4);
    send_msg(s, AD_MSG_MISSION_ITEM_INT.id, item, sizeof item);
  }
  return -1;
}

void check_rung3(session_t *s, rung_t *r) {
  vehicle_t *v = &s->v;

  if (!(v->features & AD_FEAT_MISSION_BIT)) {
    r->result = R_SKIP;
    rung_summary(r, "not declared");
    return;
  }
  if (v->mission_cmds_seen == 0) {
    rung_fail(r, "no declared mission commands, so nothing can be planned");
    rung_summary(r, "no commands");
    return;
  }

  bool can_read = (v->features & AD_FEAT_MISSION_READ_BIT) != 0;
  if (!can_read && !s->allow_mission_write) {
    r->result = R_SKIP;
    rung_summary(r, "skipped: cannot read the stored plan back, so testing would "
                    "overwrite it. Re-run with --allow-mission-write");
    return;
  }

  uint16_t stored = 0;
  if (can_read) {
    uint8_t req[2] = {v->sysid ? v->sysid : 1, v->compid ? v->compid : 1};
    send_msg(s, AD_MSG_MISSION_REQUEST_LIST.id, req, sizeof req);
    mav_msg_t m;
    if (wait_for(s, AD_MSG_MISSION_COUNT.id, 3000, &m)) stored = mav_u16(&m, 0);
    uint8_t ack[3] = {v->sysid ? v->sysid : 1, v->compid ? v->compid : 1, 0};
    send_msg(s, AD_MSG_MISSION_ACK.id, ack, sizeof ack);
  }

  uint16_t good = v->mission_cmds[0];
  uint16_t n = v->mission_capacity < 3 ? v->mission_capacity : 3;

  int result = upload(s, n, good, true, r);
  if (result < 0) {
    rung_fail(r, "a plan of %u items was never acknowledged", n);
  } else if (result != 0) {
    rung_fail(r, "a plan using only declared commands was refused, result %d", result);
  }

  /* Something nobody declares. If the vehicle accepts it, the planner will happily
     upload a plan the vehicle cannot fly. */
  uint16_t bogus = 31000;
  bool declared = false;
  for (int i = 0; i < v->mission_cmds_seen; i++) {
    if (v->mission_cmds[i] == bogus) declared = true;
  }
  if (!declared) {
    int refused = upload(s, 1, bogus, false, r);
    if (refused == 0) {
      rung_fail(r, "accepted mission command %u, which it never declared", bogus);
    } else if (refused < 0) {
      rung_warn(r, "an undeclared mission command was neither accepted nor refused");
    }
  }

  if (v->mission_capacity < 60000) {
    uint8_t hdr[4];
    memset(hdr, 0, sizeof hdr);
    mav_put_u16(&hdr[0], (uint16_t)(v->mission_capacity + 1));
    hdr[2] = v->sysid ? v->sysid : 1;
    hdr[3] = v->compid ? v->compid : 1;
    send_msg(s, AD_MSG_MISSION_COUNT.id, hdr, sizeof hdr);

    mav_msg_t ack;
    if (wait_for(s, AD_MSG_MISSION_ACK.id, 2000, &ack)) {
      if (mav_u8(&ack, 2) != 4) {
        rung_fail(r, "a plan one item over capacity was answered %u, not NO_SPACE",
                  mav_u8(&ack, 2));
      }
    } else {
      rung_fail(r, "a plan over capacity started transferring instead of being "
                   "refused up front");
    }
  }

  /* Put back what was there. Leaving a test plan on a vehicle is not acceptable. */
  if (can_read) {
    if (stored > 0) {
      upload(s, stored, good, false, r);
      rung_note(r, "restored the %u items that were stored", stored);
    } else {
      uint8_t clear[2] = {v->sysid ? v->sysid : 1, v->compid ? v->compid : 1};
      send_msg(s, AD_MSG_MISSION_CLEAR_ALL.id, clear, sizeof clear);
      pump(s, 500);
    }
  } else {
    rung_warn(r, "a test plan was left on the vehicle; it cannot read one back to "
                 "restore");
  }

  rung_summary(r, "%u items round-tripped, capacity %u, %d commands declared", n,
               v->mission_capacity, v->mission_cmds_seen);
}

/* ─── rung 4, commands ─────────────────────────────────────────────────────── */

void check_rung4(session_t *s, rung_t *r) {
  vehicle_t *v = &s->v;

  if (!(v->features & AD_FEAT_COMMANDS_BIT)) {
    r->result = R_SKIP;
    rung_summary(r, "not declared");
    return;
  }

  /*
   * Only commands that cannot move anything. Arm, disarm and mode change are never
   * sent: a conformance run must be safe to do with the aircraft on the bench and
   * propellers fitted.
   */
  uint8_t cmd[33];
  memset(cmd, 0, sizeof cmd);
  mav_put_u16(&cmd[28], 31000); /* nothing defines this */
  cmd[30] = v->sysid ? v->sysid : 1;
  cmd[31] = v->compid ? v->compid : 1;

  uint32_t sent = now_ms();
  send_msg(s, AD_MSG_COMMAND_LONG.id, cmd, sizeof cmd);

  mav_msg_t ack;
  if (!wait_for(s, AD_MSG_COMMAND_ACK.id, 2000, &ack)) {
    rung_fail(r, "an unknown command got no answer at all, so the operator sees a "
                 "button that does nothing");
    rung_summary(r, "silent");
    return;
  }

  uint32_t took = now_ms() - sent;
  if (mav_u8(&ack, 2) == 0) {
    rung_fail(r, "accepted command 31000, which does not exist");
  }
  if (took > 500) {
    rung_warn(r, "took %u ms to acknowledge; over 500 ms reads as ignored and the "
                 "operator presses again", took);
  }

  rung_summary(r, "unknown command refused in %u ms", took);
}

/* ─── calibration ──────────────────────────────────────────────────────────── */

void check_calibration(session_t *s, rung_t *r) {
  vehicle_t *v = &s->v;

  if (!(v->features & AD_FEAT_CAL_BIT)) {
    r->result = R_SKIP;
    rung_summary(r, "not declared");
    return;
  }

  if (v->cals_seen < v->cal_count) {
    request(s, 2, 0xFFFF);
    pump(s, 3000);
  }

  if (v->cals_seen < v->cal_count) {
    rung_fail(r, "manifest promises %u calibrations, %d arrived", v->cal_count,
              v->cals_seen);
  }

  int startable = -1;
  for (int i = 0; i < v->cals_seen; i++) {
    cal_info_t *c = &v->cals[i];
    if (c->name[0] == '\0') rung_fail(r, "calibration '%s' has no title", c->id);
    if (c->kind == 0 && c->pose_count == 0) {
      rung_fail(r, "'%s' is positional but declares no poses", c->id);
    }
    if (c->kind == 1 && c->track_count == 0) {
      rung_fail(r, "'%s' is coverage but declares no tracks", c->id);
    }
    if (c->poses_seen < c->pose_count) {
      rung_fail(r, "'%s' promises %u poses, %d arrived", c->id, c->pose_count,
                c->poses_seen);
    }
    if (c->tracks_seen < c->track_count) {
      rung_fail(r, "'%s' promises %u tracks, %d arrived", c->id, c->track_count,
                c->tracks_seen);
    }
    if (c->track_count > MAX_TRACKS) {
      rung_fail(r, "'%s' declares %u tracks, the profile allows %d", c->id,
                c->track_count, MAX_TRACKS);
    }
    /* Never start anything that spins a motor or has to be run at the vehicle. */
    if (startable < 0 && !(c->requirements & (0x08 | 0x10))) startable = i;
  }

  if (startable >= 0) {
    cal_info_t *c = &v->cals[startable];
    uint8_t ctl[19];
    memset(ctl, 0, sizeof ctl);
    ctl[0] = v->sysid ? v->sysid : 1;
    ctl[1] = v->compid ? v->compid : 1;
    ctl[2] = 0; /* start */
    memcpy(&ctl[3], c->id, strlen(c->id));
    send_msg(s, AD_MSG_CAL_CONTROL.id, ctl, sizeof ctl);
    pump(s, 800);

    ctl[2] = 2; /* cancel, immediately */
    send_msg(s, AD_MSG_CAL_CONTROL.id, ctl, sizeof ctl);
    pump(s, 800);
    rung_note(r, "started and cancelled '%s'; a cancel must always be honoured", c->id);
  } else if (v->cals_seen > 0) {
    rung_note(r, "every calibration needs motors or a person at the vehicle, so none "
                 "was started");
  }

  rung_summary(r, "%d declared", v->cals_seen);
}
