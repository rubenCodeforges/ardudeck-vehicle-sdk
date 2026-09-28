#include <string.h>

#include "ad_internal.h"

#if AD_MAX_MISSION_ITEMS > 0

#define MAV_MISSION_ACCEPTED          0
#define MAV_MISSION_ERROR             1
#define MAV_MISSION_UNSUPPORTED_FRAME 2
#define MAV_MISSION_UNSUPPORTED       3
#define MAV_MISSION_NO_SPACE          4
#define MAV_MISSION_INVALID_SEQUENCE 13
#define MAV_MISSION_DENIED           15

/*
 * A geofence and a rally point set travel over the very same mission protocol, told
 * apart only by mission_type. Ignoring it means a fence upload is delivered as a flight
 * plan, and the vehicle flies the boundary it was told to stay inside.
 *
 * The field is an extension, so a sender that omits it means MISSION. Readers zero-pad,
 * which gives exactly that.
 */
#define MAV_MISSION_TYPE_MISSION 0

#define MAV_FRAME_GLOBAL              0
#define MAV_FRAME_GLOBAL_RELATIVE_ALT 3
#define MAV_FRAME_GLOBAL_TERRAIN_ALT 10

static uint8_t to_mav_frame(uint8_t alt_frame) {
  switch (alt_frame) {
    case AD_ALT_ASL:     return MAV_FRAME_GLOBAL;
    case AD_ALT_TERRAIN: return MAV_FRAME_GLOBAL_TERRAIN_ALT;
    default:             return MAV_FRAME_GLOBAL_RELATIVE_ALT;
  }
}

static uint8_t from_mav_frame(uint8_t frame) {
  switch (frame) {
    case MAV_FRAME_GLOBAL:
    case 4: /* GLOBAL_INT */
      return AD_ALT_ASL;
    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
    case 11: /* GLOBAL_TERRAIN_ALT_INT */
      return AD_ALT_TERRAIN;
    default:
      return AD_ALT_RELATIVE;
  }
}

static uint16_t capacity(void) {
  uint16_t declared = ad_g.cfg->caps ? ad_g.cfg->caps->mission_capacity : 0;
  if (declared == 0 || declared > AD_MAX_MISSION_ITEMS) declared = AD_MAX_MISSION_ITEMS;
  return declared;
}

static bool command_honoured(uint16_t command) {
  const ad_capability_t *c = ad_g.cfg->caps;
  if (!c || !c->mission_cmds || c->mission_cmd_count == 0) return false;
  for (uint16_t i = 0; i < c->mission_cmd_count; i++) {
    if (c->mission_cmds[i] == command) return true;
  }
  return false;
}

static void send_ack_typed(uint8_t result, uint8_t mission_type) {
  ad_tx_begin();
  ad_put_u8(ad_g.gcs_sysid);
  ad_put_u8(ad_g.gcs_compid);
  ad_put_u8(result);
  ad_put_u8(mission_type);

  /*
   * MISSION_ACK's base payload is three bytes and mission_type is an extension, so the
   * generated length would drop it. Sent at the full length instead: v2 truncation
   * removes the trailing zero for an ordinary mission, leaving the same three bytes on
   * the wire, while a refusal of a fence or rally upload keeps the type the sender
   * routes on.
   */
  ad_msg_def_t def = AD_MSG_MISSION_ACK;
  def.length = 4;
  ad_tx_send(def);
}

static void send_ack(uint8_t result) {
  send_ack_typed(result, MAV_MISSION_TYPE_MISSION);
}

static void send_request(uint16_t seq) {
  ad_tx_begin();
  ad_put_u16(seq);
  ad_put_u8(ad_g.gcs_sysid);
  ad_put_u8(ad_g.gcs_compid);
  ad_tx_send(AD_MSG_MISSION_REQUEST_INT);
  ad_g.mission_deadline = ad_g.now + AD_TRANSFER_TIMEOUT_MS;
}

static void send_count(uint16_t count) {
  ad_tx_begin();
  ad_put_u16(count);
  ad_put_u8(ad_g.gcs_sysid);
  ad_put_u8(ad_g.gcs_compid);
  ad_tx_send(AD_MSG_MISSION_COUNT);
}

static void send_item(uint16_t seq) {
  if (seq >= ad_g.mission_expect) return;
  const ad_wp_t *w = &ad_g.mission[seq];
  ad_tx_begin();
  ad_put_f32(w->p1);
  ad_put_f32(w->p2);
  ad_put_f32(w->p3);
  ad_put_f32(w->p4);
  ad_put_i32((int32_t)(w->lat * 1e7));
  ad_put_i32((int32_t)(w->lon * 1e7));
  ad_put_f32(w->alt);
  ad_put_u16(w->seq);
  ad_put_u16(w->command);
  ad_put_u8(ad_g.gcs_sysid);
  ad_put_u8(ad_g.gcs_compid);
  ad_put_u8(to_mav_frame(w->alt_frame));
  ad_put_u8(seq == 0 ? 1 : 0); /* current */
  ad_put_u8(1);                /* autocontinue */
  ad_tx_send(AD_MSG_MISSION_ITEM_INT);
  ad_g.mission_deadline = ad_g.now + AD_TRANSFER_TIMEOUT_MS;
}

void ad_mission_reset(void) {
  ad_g.mission_state = AD_MISSION_IDLE;
  ad_g.mission_expect = 0;
  ad_g.mission_at = 0;
  ad_g.mission_deadline = 0;
}

static void deliver(void) {
  char why[64];
  why[0] = '\0';

  bool ok = true;
  if (ad_g.cfg->on_mission) {
    ok = ad_g.cfg->on_mission(ad_g.mission, ad_g.mission_expect, why, sizeof why,
                              ad_g.cfg->user);
  }
  send_ack(ok ? MAV_MISSION_ACCEPTED : MAV_MISSION_DENIED);
  if (!ok && why[0] != '\0') ad_statustext(AD_WARNING, why);
  ad_mission_reset();
}

bool ad_mission_handle(uint32_t msgid, const uint8_t *p, uint16_t len) {
  const ad_capability_t *caps = ad_g.cfg->caps;
  bool supported = caps && (caps->features & AD_FEAT_MISSION);

  if (msgid == AD_MSG_MISSION_COUNT.id) {
    uint8_t type = ad_get_u8(p, len, 4);
    if (type != MAV_MISSION_TYPE_MISSION) {
      send_ack_typed(MAV_MISSION_UNSUPPORTED, type);
      return true;
    }
    if (!supported) { send_ack(MAV_MISSION_UNSUPPORTED); return true; }
    uint16_t count = ad_get_u16(p, len, 0);
    if (count > capacity()) { send_ack(MAV_MISSION_NO_SPACE); return true; }
    if (count == 0) {
      ad_g.mission_expect = 0;
      deliver();
      return true;
    }
    ad_g.mission_state = AD_MISSION_RECEIVING;
    ad_g.mission_expect = count;
    ad_g.mission_at = 0;
    send_request(0);
    return true;
  }

  if (msgid == AD_MSG_MISSION_ITEM_INT.id) {
    if (ad_get_u8(p, len, 37) != MAV_MISSION_TYPE_MISSION) return true;
    if (ad_g.mission_state != AD_MISSION_RECEIVING) return true;

    uint16_t seq = ad_get_u16(p, len, 28);
    if (seq != ad_g.mission_at) {
      /* Out of order means a lost frame, not a broken plan. Ask again for the one we
         are still waiting on rather than abandoning the transfer. */
      send_request(ad_g.mission_at);
      return true;
    }

    uint16_t command = ad_get_u16(p, len, 30);
    if (!command_honoured(command)) {
      send_ack(MAV_MISSION_UNSUPPORTED);
      ad_mission_reset();
      return true;
    }

    uint8_t frame = ad_get_u8(p, len, 34);
    uint8_t alt_frame = from_mav_frame(frame);
    if (alt_frame == AD_ALT_TERRAIN && !(caps->features & AD_FEAT_TERRAIN)) {
      send_ack(MAV_MISSION_UNSUPPORTED_FRAME);
      ad_mission_reset();
      return true;
    }

    ad_wp_t *w = &ad_g.mission[seq];
    w->p1 = ad_get_f32(p, len, 0);
    w->p2 = ad_get_f32(p, len, 4);
    w->p3 = ad_get_f32(p, len, 8);
    w->p4 = ad_get_f32(p, len, 12);
    w->lat = (double)ad_get_i32(p, len, 16) / 1e7;
    w->lon = (double)ad_get_i32(p, len, 20) / 1e7;
    w->alt = ad_get_f32(p, len, 24);
    w->seq = seq;
    w->command = command;
    w->alt_frame = alt_frame;

    ad_g.mission_at++;
    if (ad_g.mission_at >= ad_g.mission_expect) deliver();
    else send_request(ad_g.mission_at);
    return true;
  }

  if (msgid == AD_MSG_MISSION_REQUEST_LIST.id) {
    uint8_t type = ad_get_u8(p, len, 2);
    if (type != MAV_MISSION_TYPE_MISSION) {
      send_ack_typed(MAV_MISSION_UNSUPPORTED, type);
      return true;
    }
    if (!supported || !(caps->features & AD_FEAT_MISSION_READ) ||
        !ad_g.cfg->on_mission_read) {
      send_ack(MAV_MISSION_UNSUPPORTED);
      return true;
    }
    ad_g.mission_expect =
        ad_g.cfg->on_mission_read(ad_g.mission, capacity(), ad_g.cfg->user);
    ad_g.mission_state = ad_g.mission_expect ? AD_MISSION_SENDING : AD_MISSION_IDLE;
    send_count(ad_g.mission_expect);
    ad_g.mission_deadline = ad_g.now + AD_TRANSFER_TIMEOUT_MS;
    return true;
  }

  if (msgid == AD_MSG_MISSION_REQUEST_INT.id) {
    if (ad_get_u8(p, len, 4) != MAV_MISSION_TYPE_MISSION) return true;
    if (ad_g.mission_state != AD_MISSION_SENDING) return true;
    send_item(ad_get_u16(p, len, 0));
    return true;
  }

  if (msgid == AD_MSG_MISSION_ACK.id) {
    if (ad_g.mission_state == AD_MISSION_SENDING) ad_mission_reset();
    return true;
  }

  if (msgid == AD_MSG_MISSION_CLEAR_ALL.id) {
    uint8_t type = ad_get_u8(p, len, 2);
    if (type != MAV_MISSION_TYPE_MISSION) {
      send_ack_typed(MAV_MISSION_UNSUPPORTED, type);
      return true;
    }
    if (!supported) { send_ack(MAV_MISSION_UNSUPPORTED); return true; }
    ad_g.mission_expect = 0;
    ad_g.mission_at = 0;
    bool ok = true;
    if (ad_g.cfg->on_mission) {
      char why[64];
      why[0] = '\0';
      ok = ad_g.cfg->on_mission(ad_g.mission, 0, why, sizeof why, ad_g.cfg->user);
    }
    send_ack(ok ? MAV_MISSION_ACCEPTED : MAV_MISSION_ERROR);
    ad_mission_reset();
    return true;
  }

  return false;
}

void ad_mission_tick(void) {
  if (ad_g.mission_state == AD_MISSION_IDLE) return;
  if (!ad_elapsed(ad_g.mission_deadline)) return;

  /* A transfer that stops halfway must not hold the vehicle hostage: the next attempt
     starts clean, and a half-received plan is never delivered as if it were whole. */
  if (ad_g.mission_state == AD_MISSION_RECEIVING) {
    send_ack(MAV_MISSION_INVALID_SEQUENCE);
    ad_statustext(AD_NOTICE, "Mission upload timed out");
  }
  ad_mission_reset();
}

#else /* AD_MAX_MISSION_ITEMS == 0 */

void ad_mission_reset(void) {}
void ad_mission_tick(void) {}
bool ad_mission_handle(uint32_t msgid, const uint8_t *p, uint16_t len) {
  (void)msgid; (void)p; (void)len;
  return false;
}

#endif
