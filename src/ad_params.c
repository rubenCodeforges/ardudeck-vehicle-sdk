#include <string.h>

#include "ad_internal.h"

#if AD_MAX_PARAMS > 0

#define MAV_PARAM_TYPE_UINT8  1
#define MAV_PARAM_TYPE_INT32  6
#define MAV_PARAM_TYPE_REAL32 9

static uint16_t param_count(void) {
  uint16_t n = ad_g.cfg ? ad_g.cfg->param_count : 0;
  return n > AD_MAX_PARAMS ? AD_MAX_PARAMS : n;
}

static const ad_param_t *param_at(uint16_t index) {
  return index < param_count() ? &ad_g.cfg->params[index] : NULL;
}

static uint8_t mav_type(const ad_param_t *p) {
  switch (p->type) {
    case AD_T_I32: return MAV_PARAM_TYPE_INT32;
    case AD_T_U8:  return MAV_PARAM_TYPE_UINT8;
    default:       return MAV_PARAM_TYPE_REAL32;
  }
}

static float read_value(const ad_param_t *p) {
  float v = 0.0f;
  if (p->get) {
    if (!p->get(p, &v)) return 0.0f;
    return v;
  }
  if (!p->storage) return 0.0f;
  switch (p->type) {
    case AD_T_I32: { int32_t i; memcpy(&i, p->storage, sizeof i); return (float)i; }
    case AD_T_U8:  { uint8_t u; memcpy(&u, p->storage, sizeof u); return (float)u; }
    default:       { float f;   memcpy(&f, p->storage, sizeof f); return f; }
  }
}

static bool write_value(const ad_param_t *p, float v) {
  if (p->flags & AD_PARAM_READONLY) return false;

  /* Clamp rather than refuse: an operator who typed 200 into a 0 to 100 field meant the
     top of the range, and a silent rejection reads as a broken screen. */
  if (v < p->min_value) v = p->min_value;
  if (v > p->max_value) v = p->max_value;

  if (p->set) return p->set(p, v);
  if (!p->storage) return false;

  switch (p->type) {
    case AD_T_I32: { int32_t i = (int32_t)(v < 0 ? v - 0.5f : v + 0.5f);
                     memcpy(p->storage, &i, sizeof i); break; }
    case AD_T_U8:  { uint8_t u = (uint8_t)(v < 0 ? 0 : v + 0.5f);
                     memcpy(p->storage, &u, sizeof u); break; }
    default:       { memcpy(p->storage, &v, sizeof v); break; }
  }
  return true;
}

void ad_params_send_value(uint16_t index) {
  const ad_param_t *p = param_at(index);
  if (!p) return;
  ad_tx_begin();
  ad_put_f32(read_value(p));
  ad_put_u16(param_count());
  ad_put_u16(index);
  ad_put_str(p->name, 16);
  ad_put_u8(mav_type(p));
  ad_tx_send(AD_MSG_PARAM_VALUE);
}

static void send_meta(uint16_t index) {
  const ad_param_t *p = param_at(index);
  if (!p) return;
  ad_tx_begin();
  ad_put_f32(p->min_value);
  ad_put_f32(p->max_value);
  ad_put_f32(p->increment);
  ad_put_u16(index);
  ad_put_u8(p->flags);
  ad_put_u8(p->option_count);
  ad_put_str(p->name, 16);
  ad_put_str(p->unit ? p->unit : "", 12);
  ad_put_str(p->help ? p->help : "", 80);
  ad_tx_send(AD_MSG_PARAM_META);
}

static void send_option(uint16_t index, uint8_t option) {
  const ad_param_t *p = param_at(index);
  if (!p || option >= p->option_count || !p->options) return;
  ad_tx_begin();
  ad_put_u16(index);
  ad_put_u8(option);
  ad_put_u8(p->option_count);
  ad_put_str(p->name, 16);
  ad_put_str(p->options[option] ? p->options[option] : "", 32);
  ad_tx_send(AD_MSG_PARAM_OPTION);
}

static int16_t index_of(const char *name) {
  for (uint16_t i = 0; i < param_count(); i++) {
    const ad_param_t *p = param_at(i);
    if (p && p->name && strncmp(p->name, name, 16) == 0) return (int16_t)i;
  }
  return -1;
}

void ad_params_announce(const char *name) {
  int16_t i = index_of(name);
  if (i >= 0) ad_params_send_value((uint16_t)i);
}

void ad_params_reset(void) {
  ad_g.param_streaming = false;
  ad_g.param_next = 0;
  ad_g.meta_streaming = false;
  ad_g.meta_next = 0;
  ad_g.meta_option_next = 0;
}

void ad_params_handle(uint32_t msgid, const uint8_t *p, uint16_t len) {
  char name[17];

  if (msgid == AD_MSG_PARAM_REQUEST_LIST.id) {
    ad_g.param_streaming = true;
    ad_g.param_next = 0;
    ad_g.meta_streaming = true;
    ad_g.meta_next = 0;
    ad_g.meta_option_next = 0;
    return;
  }

  if (msgid == AD_MSG_PARAM_REQUEST_READ.id) {
    int16_t index = (int16_t)ad_get_u16(p, len, 0);
    if (index < 0) {
      ad_get_str(p, len, 4, 16, name, sizeof name);
      index = index_of(name);
    }
    if (index >= 0) {
      ad_params_send_value((uint16_t)index);
      send_meta((uint16_t)index);
    }
    return;
  }

  if (msgid == AD_MSG_PARAM_SET.id) {
    float value = ad_get_f32(p, len, 0);
    ad_get_str(p, len, 6, 16, name, sizeof name);
    int16_t index = index_of(name);
    if (index < 0) return;

    const ad_param_t *param = param_at((uint16_t)index);
    bool ok = write_value(param, value);
    if (ok && ad_g.cfg->on_param_set) {
      ok = ad_g.cfg->on_param_set(param, read_value(param), ad_g.cfg->user);
    }
    /*
     * Answer either way, with whatever the value is now. A refused or clamped write that
     * reports nothing leaves the editor showing a number the vehicle does not hold.
     */
    (void)ok;
    ad_params_send_value((uint16_t)index);
    return;
  }
}

void ad_params_tick(void) {
  /* One message per tick. A hundred parameters in one burst overruns a serial link and
     the ground station re-requests the whole list, which is slower than going slowly. */
  if (ad_g.param_streaming) {
    if (ad_g.param_next < param_count()) {
      ad_params_send_value(ad_g.param_next++);
      return;
    }
    ad_g.param_streaming = false;
  }

  if (ad_g.meta_streaming) {
    if (ad_g.meta_next >= param_count()) {
      ad_g.meta_streaming = false;
      return;
    }
    const ad_param_t *p = param_at(ad_g.meta_next);
    if (ad_g.meta_option_next == 0) {
      send_meta(ad_g.meta_next);
      ad_g.meta_option_next = 1;
      if (!p || p->option_count == 0) {
        ad_g.meta_next++;
        ad_g.meta_option_next = 0;
      }
      return;
    }
    uint8_t option = (uint8_t)(ad_g.meta_option_next - 1);
    send_option(ad_g.meta_next, option);
    ad_g.meta_option_next++;
    if (!p || option + 1 >= p->option_count) {
      ad_g.meta_next++;
      ad_g.meta_option_next = 0;
    }
  }
}

#else /* AD_MAX_PARAMS == 0 */

void ad_params_reset(void) {}
void ad_params_tick(void) {}
void ad_params_send_value(uint16_t index) { (void)index; }
void ad_params_announce(const char *name) { (void)name; }
void ad_params_handle(uint32_t msgid, const uint8_t *p, uint16_t len) {
  (void)msgid; (void)p; (void)len;
}

#endif
