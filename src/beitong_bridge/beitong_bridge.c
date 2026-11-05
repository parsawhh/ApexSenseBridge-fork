// Beitong KP70C2 <-> DualSense protocol translation.
#include "beitong/beitong_protocol.h"

#include <string.h>

static const uint8_t kLinearX[5] = {42, 84, 128, 167, 255};
static const uint8_t kLinearY[5] = {42, 84, 128, 167, 255};
static const uint8_t kIntenseX[5] = {2, 60, 120, 190, 255};
static const uint8_t kIntenseY[5] = {40, 140, 180, 235, 255};
static const uint8_t kGentleX[5] = {30, 60, 120, 190, 255};
static const uint8_t kGentleY[5] = {10, 30, 80, 190, 220};

static uint8_t clamp_u8(int value, int minimum, int maximum) {
    if (value < minimum) return (uint8_t)minimum;
    if (value > maximum) return (uint8_t)maximum;
    return (uint8_t)value;
}

static int scale_range(int value, int in_min, int in_max, int out_min, int out_max) {
    if (in_max == in_min) return out_min;
    const float ratio = (float)(value - in_min) / (float)(in_max - in_min);
    const int scaled = out_min + (int)(ratio * (float)(out_max - out_min) + 0.5f);
    return scaled < out_min ? out_min : (scaled > out_max ? out_max : scaled);
}

static uint8_t apply_spline(int curve, uint8_t input) {
    const uint8_t *x = kIntenseX;
    const uint8_t *y = kIntenseY;
    if (curve == 0) { x = kLinearX; y = kLinearY; }
    else if (curve == 2) { x = kGentleX; y = kGentleY; }
    if (input <= x[0]) return (uint8_t)scale_range(input, 0, x[0], 0, y[0]);
    for (int i = 0; i < 4; ++i) {
        if (input <= x[i + 1]) return (uint8_t)scale_range(input, x[i], x[i + 1], y[i], y[i + 1]);
    }
    return y[4];
}

uint32_t beitong_crc32(uint8_t seed, const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    crc ^= seed;
    for (int bit = 0; bit < 8; ++bit) crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
    return ~crc;
}

void beitong_bridge_init(beitong_bridge_state_t *b, bool is_bluetooth) {
    if (!b) return;
    memset(b, 0, sizeof(*b));
    b->is_bluetooth = is_bluetooth;
    b->trigger_force_gain = 1.40f;
    b->trigger_vibe_gain = 1.45f;
    b->rumble_gain = 1.30f;
    b->min_force_floor = 20;
    b->curve_type = 1;
    b->lt_deadzone_start = b->rt_deadzone_start = 25;
    b->lt_deadzone_end = b->rt_deadzone_end = 255;
    b->lt_mode = b->rt_mode = DS_TRIGGER_OFF;
    b->ds_input_report[0] = 0x01;
    b->ds_input_report[1] = b->ds_input_report[2] = 0x80;
    b->ds_input_report[3] = b->ds_input_report[4] = 0x80;
    b->ds_input_report[8] = 0x08;
}

void beitong_bridge_set_gains(beitong_bridge_state_t *b, float force_gain, float vibe_gain, float rumble_gain, uint8_t min_floor) {
    if (!b) return;
    if (force_gain > 0.1f) b->trigger_force_gain = force_gain;
    if (vibe_gain > 0.1f) b->trigger_vibe_gain = vibe_gain;
    if (rumble_gain > 0.1f) b->rumble_gain = rumble_gain;
    b->min_force_floor = min_floor;
}

void beitong_bridge_set_curve(beitong_bridge_state_t *b, int curve_type) {
    if (b) b->curve_type = curve_type >= 0 && curve_type <= 2 ? curve_type : 1;
}

void beitong_bridge_set_deadzones(beitong_bridge_state_t *b, uint8_t lt_start, uint8_t lt_end, uint8_t rt_start, uint8_t rt_end) {
    if (!b) return;
    b->lt_deadzone_start = lt_start;
    b->lt_deadzone_end = lt_end;
    b->rt_deadzone_start = rt_start;
    b->rt_deadzone_end = rt_end;
}

static void encode_feedback(uint8_t position, uint8_t strength, uint8_t *mode, uint8_t *params) {
    memset(params, 0, 10);
    if (!strength) { *mode = DS_TRIGGER_OFF; return; }
    *mode = DS_TRIGGER_FEEDBACK;
    const unsigned start = position > 9 ? 9 : position;
    const uint8_t encoded = (uint8_t)(strength > 8 ? 7 : strength - 1);
    uint16_t zones = 0;
    uint32_t forces = 0;
    for (unsigned i = start; i < 10; ++i) {
        zones |= (uint16_t)(1u << i);
        forces |= (uint32_t)encoded << (3u * i);
    }
    params[0] = (uint8_t)zones; params[1] = (uint8_t)(zones >> 8);
    for (unsigned i = 0; i < 4; ++i) params[2 + i] = (uint8_t)(forces >> (8u * i));
}

static void encode_weapon(uint8_t start, uint8_t end, uint8_t strength, uint8_t *mode, uint8_t *params) {
    memset(params, 0, 10);
    if (!strength) { *mode = DS_TRIGGER_OFF; return; }
    *mode = DS_TRIGGER_WEAPON;
    if (start < 2) start = 2;
    if (start > 7) start = 7;
    if (end <= start) end = (uint8_t)(start + 1);
    if (end > 8) end = 8;
    const uint16_t zones = (uint16_t)((1u << start) | (1u << end));
    params[0] = (uint8_t)zones; params[1] = (uint8_t)(zones >> 8);
    params[2] = (uint8_t)((strength > 8 ? 8 : strength) - 1);
}

static void encode_vibration(uint8_t position, uint8_t amplitude, uint8_t frequency, uint8_t *mode, uint8_t *params) {
    memset(params, 0, 10);
    if (!amplitude || !frequency) { *mode = DS_TRIGGER_OFF; return; }
    *mode = DS_TRIGGER_VIBRATION;
    const unsigned start = position > 9 ? 9 : position;
    const uint8_t encoded = (uint8_t)(amplitude > 8 ? 7 : amplitude - 1);
    uint16_t zones = 0; uint32_t amps = 0;
    for (unsigned i = start; i < 10; ++i) { zones |= (uint16_t)(1u << i); amps |= (uint32_t)encoded << (3u * i); }
    params[0] = (uint8_t)zones; params[1] = (uint8_t)(zones >> 8);
    for (unsigned i = 0; i < 4; ++i) params[2 + i] = (uint8_t)(amps >> (8u * i));
    params[8] = frequency;
}

static uint32_t read_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void translate_ds_trigger(beitong_bridge_state_t *b, uint8_t side, uint8_t ds_mode, const uint8_t *p, uint8_t *packet) {
    memset(packet, 0, 10);
    packet[0] = 0x02; packet[1] = 0x56; packet[2] = side;
    if (ds_mode == DS_TRIGGER_OFF || ds_mode == 0) { packet[3] = BT_TRIGGER_OFF; return; }
    const uint16_t zones = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    int start = 0;
    while (start < 9 && !(zones & (1u << start))) ++start;
    if (ds_mode == DS_TRIGGER_FEEDBACK || ds_mode == DS_TRIGGER_BOW) {
        packet[3] = BT_TRIGGER_RACE_RESISTANCE;
        const uint8_t strength = (uint8_t)(((read_le32(p + 2) >> (3 * start)) & 7u) + 1u);
        int force = (int)(scale_range(strength, 1, 8, 15, 100) * b->trigger_force_gain + 0.5f);
        if (force < b->min_force_floor) force = b->min_force_floor;
        packet[4] = (uint8_t)scale_range(start, 0, 9, 0, 100); packet[5] = clamp_u8(force, 0, 100);
    } else if (ds_mode == DS_TRIGGER_WEAPON) {
        packet[3] = BT_TRIGGER_SHOT_BREAKPOINT;
        int end = 7; bool got_start = false;
        for (int i = 0; i < 10; ++i) if (zones & (1u << i)) { if (!got_start) { start = i; got_start = true; } else { end = i; break; } }
        const uint8_t strength = (uint8_t)((p[2] & 7u) + 1u);
        int force = (int)(scale_range(strength, 1, 8, 25, 100) * b->trigger_force_gain + 0.5f);
        if (force < b->min_force_floor) force = b->min_force_floor;
        packet[4] = (uint8_t)scale_range(start, 0, 9, 0, 100);
        packet[5] = (uint8_t)scale_range(end, 0, 9, packet[4] + 10, 100);
        packet[6] = clamp_u8(force, 0, 100); packet[7] = packet[6] / 2; packet[8] = 1;
    } else if (ds_mode == DS_TRIGGER_VIBRATION || ds_mode == DS_TRIGGER_MACHINE || ds_mode == DS_TRIGGER_GALLOPING) {
        packet[3] = BT_TRIGGER_GUN_VIBRATION;
        const uint8_t amp = (uint8_t)(((read_le32(p + 2) >> (3 * start)) & 7u) + 1u);
        int strength = (int)(scale_range(amp, 1, 8, 20, 100) * b->trigger_vibe_gain + 0.5f);
        if (strength < b->min_force_floor) strength = b->min_force_floor;
        packet[4] = (uint8_t)scale_range(start, 0, 9, 0, 100);
        packet[5] = (uint8_t)scale_range(p[8] ? p[8] : 100, 10, 240, 5, 100);
        packet[6] = clamp_u8(strength, 0, 100);
    } else packet[3] = BT_TRIGGER_OFF;
}

int beitong_translate_game_to_beitong(beitong_bridge_state_t *b, const uint8_t *report, size_t len, beitong_translation_result_t *result) {
    if (!b || !report || !result) return 0;
    memset(result, 0, sizeof(*result));
    const ds_output_common_t *common = NULL;
    if (report[0] == 0x02) {
        if (len < sizeof(ds_usb_output_report_t)) return 0;
        common = (const ds_output_common_t *)(report + 1);
    } else if (report[0] == 0x31) {
        if (len < sizeof(ds_bt_output_report_t)) return 0;
        common = (const ds_output_common_t *)(report + 3);
    } else return 0;

    if (common->valid_flag0 & DS_FLAG0_RIGHT_TRIGGER_ENABLE) {
        translate_ds_trigger(b, BT_TRIGGER_RIGHT, common->right_trigger_mode, common->right_trigger_params, result->trigger_pkts[result->trigger_count].data);
        result->trigger_pkts[result->trigger_count++].len = 9;
    }
    if (common->valid_flag0 & DS_FLAG0_LEFT_TRIGGER_ENABLE) {
        translate_ds_trigger(b, BT_TRIGGER_LEFT, common->left_trigger_mode, common->left_trigger_params, result->trigger_pkts[result->trigger_count].data);
        result->trigger_pkts[result->trigger_count++].len = 9;
    }
    if (common->valid_flag0 & DS_FLAG0_COMPATIBLE_VIBRATION) {
        const uint8_t motor = common->motor_left > common->motor_right ? common->motor_left : common->motor_right;
        const int amplified = (int)(apply_spline(b->curve_type, motor) * b->rumble_gain + 0.5f);
        result->rumble_pkt.data[0] = 0x02; result->rumble_pkt.data[1] = 0x76;
        result->rumble_pkt.data[2] = 0x01; result->rumble_pkt.data[3] = BT_RUMBLE_SUBCMD_RIGHT_A;
        result->rumble_pkt.data[4] = clamp_u8(amplified, 0, 100); result->rumble_pkt.data[5] = 0;
        result->rumble_pkt.len = result->rumble_len = 6;
    }
    return (int)(result->trigger_count + (result->rumble_len ? 1 : 0));
}

static void translate_bt_trigger(beitong_bridge_state_t *b, uint8_t mode, const uint8_t *p, size_t len, uint8_t *target_mode, uint8_t *target_params) {
    if (mode == BT_TRIGGER_OFF || !len) { *target_mode = DS_TRIGGER_OFF; memset(target_params, 0, 10); return; }
    if (mode == BT_TRIGGER_RACE_RESISTANCE) {
        const uint8_t pos = len > 0 ? p[0] : 0, force = len > 1 ? p[1] : 50;
        int strength = (int)(scale_range(force, 0, 100, 1, 8) * b->trigger_force_gain + 0.5f);
        if (strength < b->min_force_floor) strength = b->min_force_floor;
        encode_feedback((uint8_t)scale_range(pos, 0, 100, 0, 9), clamp_u8(strength, 1, 8), target_mode, target_params);
    } else if (mode == BT_TRIGGER_SHOT_BREAKPOINT) {
        const uint8_t start = len > 0 ? p[0] : 20, end = len > 1 ? p[1] : 70, force = len > 2 ? p[2] : 80;
        const uint8_t ds_start = (uint8_t)scale_range(start, 0, 100, 2, 7);
        const uint8_t ds_end = (uint8_t)scale_range(end, 0, 100, ds_start + 1, 8);
        int strength = (int)(scale_range(force, 0, 100, 1, 8) * b->trigger_force_gain + 0.5f);
        if (strength < b->min_force_floor) strength = b->min_force_floor;
        encode_weapon(ds_start, ds_end, clamp_u8(strength, 1, 8), target_mode, target_params);
    } else if (mode == BT_TRIGGER_GUN_VIBRATION) {
        const uint8_t pos = len > 0 ? p[0] : 0, freq = len > 1 ? p[1] : 50, amp = len > 2 ? p[2] : 90;
        int strength = (int)(scale_range(amp, 0, 100, 1, 8) * b->trigger_vibe_gain + 0.5f);
        if (strength < b->min_force_floor) strength = b->min_force_floor;
        encode_vibration((uint8_t)scale_range(pos, 0, 100, 0, 9), clamp_u8(strength, 1, 8), (uint8_t)scale_range(freq, 0, 100, 10, 240), target_mode, target_params);
    } else { *target_mode = DS_TRIGGER_OFF; memset(target_params, 0, 10); }
}

static int build_current_report(beitong_bridge_state_t *b, uint8_t *out, size_t cap) {
    if (!b || !out) return 0;
    if (b->is_bluetooth) {
        if (cap < sizeof(ds_bt_output_report_t)) return 0;
        ds_bt_output_report_t rep; memset(&rep, 0, sizeof(rep));
        rep.report_id = 0x31; rep.seq_tag = (uint8_t)((b->bt_sequence & 0x0F) << 4); b->bt_sequence = (uint8_t)((b->bt_sequence + 1) & 0x0F); rep.tag = DS_OUTPUT_TAG_BT;
        rep.common.valid_flag0 = DS_FLAG0_COMPATIBLE_VIBRATION | DS_FLAG0_RIGHT_TRIGGER_ENABLE | DS_FLAG0_LEFT_TRIGGER_ENABLE;
        rep.common.motor_right = b->motor_right; rep.common.motor_left = b->motor_left;
        rep.common.right_trigger_mode = b->rt_mode; memcpy(rep.common.right_trigger_params, b->rt_params, 10);
        rep.common.left_trigger_mode = b->lt_mode; memcpy(rep.common.left_trigger_params, b->lt_params, 10);
        rep.crc32 = beitong_crc32(DS_BT_CRC_SEED, (const uint8_t *)&rep, offsetof(ds_bt_output_report_t, crc32));
        memcpy(out, &rep, sizeof(rep)); return (int)sizeof(rep);
    }
    if (cap < sizeof(ds_usb_output_report_t)) return 0;
    ds_usb_output_report_t rep; memset(&rep, 0, sizeof(rep)); rep.report_id = 0x02;
    rep.common.valid_flag0 = DS_FLAG0_COMPATIBLE_VIBRATION | DS_FLAG0_RIGHT_TRIGGER_ENABLE | DS_FLAG0_LEFT_TRIGGER_ENABLE;
    rep.common.motor_right = b->motor_right; rep.common.motor_left = b->motor_left;
    rep.common.right_trigger_mode = b->rt_mode; memcpy(rep.common.right_trigger_params, b->rt_params, 10);
    rep.common.left_trigger_mode = b->lt_mode; memcpy(rep.common.left_trigger_params, b->lt_params, 10);
    memcpy(out, &rep, sizeof(rep)); return (int)sizeof(rep);
}

int beitong_translate_packet(beitong_bridge_state_t *b, const uint8_t *in, size_t len, uint8_t *out, size_t cap) {
    if (!b || !in || len < 4 || in[0] != 0x02) return 0;
    if (in[1] == 0x56) {
        uint8_t *target_mode = in[2] == BT_TRIGGER_LEFT ? &b->lt_mode : &b->rt_mode;
        uint8_t *target_params = in[2] == BT_TRIGGER_LEFT ? b->lt_params : b->rt_params;
        translate_bt_trigger(b, in[3], in + 4, len - 4, target_mode, target_params);
        return build_current_report(b, out, cap);
    }
    if (in[1] == 0x76 && len >= 5 && in[2] == 0x01) {
        const uint8_t sub = in[3], input = in[4];
        const uint8_t value = clamp_u8((int)(apply_spline(b->curve_type, input) * b->rumble_gain + 0.5f), 0, 100);
        if (sub == 0 || sub == 2 || sub == 4) b->motor_right = value;
        else if (sub == 1 || sub == 3 || sub == 5) b->motor_left = value;
        else return 0;
        return build_current_report(b, out, cap);
    }
    return 0;
}

int beitong_translate_beitong_to_game(beitong_bridge_state_t *b, const uint8_t *in, size_t len, uint8_t *out, size_t cap) {
    if (!b || !in || len < 12 || !out || cap < 64) return 0;
    uint8_t rep[64] = {0}; rep[0] = 0x01;
    memcpy(rep + 1, in, 6);
    rep[7] = b->bt_sequence++;
    const uint16_t buttons = (uint16_t)in[6] | ((uint16_t)in[7] << 8);
    const bool up = (buttons & (1u << 0)) != 0, down = (buttons & (1u << 1)) != 0;
    const bool left = (buttons & (1u << 2)) != 0, right = (buttons & (1u << 3)) != 0;
    uint8_t dpad = 8;
    if (up && right) dpad = 1; else if (down && right) dpad = 3; else if (down && left) dpad = 5; else if (up && left) dpad = 7;
    else if (up) dpad = 0; else if (right) dpad = 2; else if (down) dpad = 4; else if (left) dpad = 6;
    rep[8] = dpad;
    if (buttons & (1u << 6)) rep[8] |= 1u << 4;
    if (buttons & (1u << 4)) rep[8] |= 1u << 5;
    if (buttons & (1u << 5)) rep[8] |= 1u << 6;
    if (buttons & (1u << 7)) rep[8] |= 1u << 7;
    if (buttons & (1u << 8)) rep[9] |= 1u << 0;
    if (buttons & (1u << 9)) rep[9] |= 1u << 1;
    if (rep[5] > 0x10) rep[9] |= 1u << 2;
    if (rep[6] > 0x10) rep[9] |= 1u << 3;
    if (buttons & (1u << 10)) rep[9] |= 1u << 4;
    if (buttons & (1u << 11)) rep[9] |= 1u << 5;
    if (buttons & (1u << 12)) rep[9] |= 1u << 6;
    if (buttons & (1u << 13)) rep[9] |= 1u << 7;
    if (buttons & (1u << 14)) rep[10] |= 1u;
    rep[53] = 0x0A;
    memcpy(out, rep, sizeof(rep)); memcpy(b->ds_input_report, rep, sizeof(rep)); ++b->input_updates;
    return 64;
}
