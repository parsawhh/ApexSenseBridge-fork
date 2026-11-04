// ============================================================================
// Beitong Bridge Core Implementation
// Ported from beitong-dualsense-bridge to ApexSenseBridge framework
// ============================================================================

#include "beitong/beitong_protocol.h"
// Forward declarations
void translate_bt_trigger(beitong_bridge_state_t *b, uint8_t side, uint8_t mode, const uint8_t *params, size_t param_len, uint8_t *target_mode, uint8_t *target_params);
#include <string.h>
#include <stdlib.h>

// CRC32 implementation (seeded, matches Beitong spec)
uint32_t beitong_crc32(uint8_t seed, const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    crc ^= seed;
    for (int b = 0; b < 8; b++) {
        crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320) : (crc >> 1);
    }
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320) : (crc >> 1);
        }
    }
    return ~crc;
}

// Initialize bridge state with Beitong defaults
void beitong_bridge_init(beitong_bridge_state_t *b, bool is_bluetooth) {
    if (!b) return;
    
    memset(b, 0, sizeof(*b));
    b->is_bluetooth = is_bluetooth;
    b->bt_sequence = 0;
    
    // High performance authoritative force profile (Beitong defaults)
    b->trigger_force_gain = 1.40f;    // Race resistance gain
    b->trigger_vibe_gain  = 1.45f;    // Recoil pulse gain
    b->rumble_gain        = 1.30f;    // Grip rumble gain
    b->min_force_floor    = 20;       // 20% minimum floor for authoritative resistance
    
    b->curve_type = 1;                // CURVE_INTENSE
    b->sync_trigger_with_grip = false;
    
    b->lt_deadzone_start = 25;
    b->rt_deadzone_start = 25;
    
    // Default neutral DualSense Input Report
    b->ds_input_report[0] = 0x01;     // Report ID
    b->ds_input_report[1] = 0x80;     // Left Stick X center
    b->ds_input_report[2] = 0x80;     // Left Stick Y center
    b->ds_input_report[3] = 0x80;     // Right Stick X center
    b->ds_input_report[4] = 0x80;     // Right Stick Y center
    b->ds_input_report[8] = 0x08;     // D-pad neutral
}

// Set gain profiles
void beitong_bridge_set_gains(beitong_bridge_state_t *b, float force_gain, float vibe_gain, float rumble_gain, uint8_t min_floor) {
    if (!b) return;
    if (force_gain > 0.1f) b->trigger_force_gain = force_gain;
    if (vibe_gain > 0.1f)  b->trigger_vibe_gain = vibe_gain;
    if (rumble_gain > 0.1f) b->rumble_gain = rumble_gain;
    b->min_force_floor = min_floor;
}

// Set curve type (0=Linear, 1=Intense, 2=Gentle)
void beitong_bridge_set_curve(beitong_bridge_state_t *b, int curve_type) {
    if (!b) return;
    b->curve_type = curve_type;
}

// Set deadzones
void beitong_bridge_set_deadzones(beitong_bridge_state_t *b, uint8_t lt_start, uint8_t lt_end, uint8_t rt_start, uint8_t rt_end) {
    if (!b) return;
    b->lt_deadzone_start = lt_start;
    b->lt_deadzone_end = lt_end;
    b->rt_deadzone_start = rt_start;
    b->rt_deadzone_end = rt_end;
}

// Scale a value from one range to another
static int scale_range(int val, int in_min, int in_max, int out_min, int out_max) {
    if (in_max == in_min) return out_min;
    float ratio = (float)(val - in_min) / (float)(in_max - in_min);
    int scaled = out_min + (int)(ratio * (float)(out_max - out_min) + 0.5f);
    if (scaled < out_min) return out_min;
    if (scaled > out_max) return out_max;
    return scaled;
}

// Clamp a value
static uint8_t clamp_u8(int val, int min_val, int max_val) {
    if (val < min_val) return (uint8_t)min_val;
    if (val > max_val) return (uint8_t)max_val;
    return (uint8_t)val;
}

// Apply spline curve for haptics
static uint8_t apply_spline(const int curve_type, uint8_t input_val) {
    // Intense preset: {2, 60, 120, 190, 255} -> {40, 140, 180, 235, 255}
    // Gentle preset: {30, 60, 120, 190, 255} -> {10, 30, 80, 190, 220}
    static const uint8_t INTENSE_X[5] = {2, 60, 120, 190, 255};
    static const uint8_t INTENSE_Y[5] = {40, 140, 180, 235, 255};
    static const uint8_t GENTLE_X[5]  = {30, 60, 120, 190, 255};
    static const uint8_t GENTLE_Y[5]  = {10, 30, 80, 190, 220};
    
    if (input_val <= 2) {  // Linear portion
        return (uint8_t)((input_val * (INTENSE_Y[1] - INTENSE_Y[0]) / (INTENSE_X[1] - INTENSE_X[0])) + INTENSE_Y[0]);
    }
    if (input_val <= 60) {
        return (uint8_t)(((input_val - 2) * (INTENSE_Y[2] - INTENSE_Y[1]) / (INTENSE_X[2] - INTENSE_X[1])) + INTENSE_Y[1]);
    }
    if (input_val <= 120) {
        return (uint8_t)(((input_val - 60) * (INTENSE_Y[3] - INTENSE_Y[2]) / (INTENSE_X[3] - INTENSE_X[2])) + INTENSE_Y[2]);
    }
    if (input_val <= 190) {
        return (uint8_t)(((input_val - 120) * (INTENSE_Y[4] - INTENSE_Y[3]) / (INTENSE_X[4] - INTENSE_X[3])) + INTENSE_Y[3]);
    }
    return 255;
}

// ============================================================================
// GAME -> BEITONG TRANSLATION
// Translates Sony DualSense Report 0x02 into Beitong 0x02 trigger packets
// ============================================================================

// Encode DS feedback mode into Beitong format
static void encode_ds_feedback(uint8_t start_pos, uint8_t strength, uint8_t *out_mode, uint8_t *out_params) {
    if (strength == 0) {
        *out_mode = DS_TRIGGER_OFF;
        memset(out_params, 0, 10);
        return;
    }
    *out_mode = DS_TRIGGER_FEEDBACK;
    memset(out_params, 0, 10);
    
    uint8_t pos = (start_pos > 9) ? 9 : start_pos;
    uint8_t str_val = (strength > 7) ? 7 : strength;  // 1..8 -> 0..7
    
    uint16_t active_zones = 0;
    uint32_t force_zones = 0;
    for (int i = pos; i < 10; i++) {
        active_zones |= (1 << i);
        force_zones |= ((uint32_t)str_val << (3 * i));
    }
    
    out_params[0] = (uint8_t)(active_zones & 0xFF);
    out_params[1] = (uint8_t)((active_zones >> 8) & 0xFF);
    out_params[2] = (uint8_t)(force_zones & 0xFF);
    out_params[3] = (uint8_t)((force_zones >> 8) & 0xFF);
    out_params[4] = (uint8_t)((force_zones >> 16) & 0xFF);
    out_params[5] = (uint8_t)((force_zones >> 24) & 0xFF);
}

// Encode DS weapon mode into Beitong format
static void encode_ds_weapon(uint8_t start_pos, uint8_t end_pos, uint8_t strength, uint8_t *out_mode, uint8_t *out_params) {
    if (strength == 0) {
        *out_mode = DS_TRIGGER_OFF;
        memset(out_params, 0, 10);
        return;
    }
    *out_mode = DS_TRIGGER_WEAPON;
    memset(out_params, 0, 10);
    
    uint8_t start = (start_pos > 7) ? 7 : start_pos;
    start = (start < 2) ? 2 : start;  // Clamp to 2..7
    uint8_t end = (end_pos > 8) ? 8 : end_pos;
    end = (end < start + 1) ? start + 1 : end;  // end > start
    
    uint8_t str_val = (strength > 7) ? 7 : strength;  // 1..8 -> 0..7
    
    uint16_t zones = (1 << start) | (1 << end);
    out_params[0] = (uint8_t)(zones & 0xFF);
    out_params[1] = (uint8_t)((zones >> 8) & 0xFF);
    out_params[2] = str_val;
}

// Encode DS vibration mode into Beitong format
static void encode_ds_vibration(uint8_t start_pos, uint8_t amplitude, uint8_t frequency, uint8_t *out_mode, uint8_t *out_params) {
    if (amplitude == 0 || frequency == 0) {
        *out_mode = DS_TRIGGER_OFF;
        memset(out_params, 0, 10);
        return;
    }
    *out_mode = DS_TRIGGER_VIBRATION;
    memset(out_params, 0, 10);
    
    uint8_t pos = (start_pos > 9) ? 9 : start_pos;
    uint8_t amp_val = (amplitude > 7) ? 7 : amplitude;  // 1..8 -> 0..7
    
    uint16_t active_zones = 0;
    uint32_t amp_zones = 0;
    for (int i = pos; i < 10; i++) {
        active_zones |= (1 << i);
        amp_zones |= ((uint32_t)amp_val << (3 * i));
    }
    
    out_params[0] = (uint8_t)(active_zones & 0xFF);
    out_params[1] = (uint8_t)((active_zones >> 8) & 0xFF);
    out_params[2] = (uint8_t)(amp_zones & 0xFF);
    out_params[3] = (uint8_t)((amp_zones >> 8) & 0xFF);
    out_params[4] = (uint8_t)((amp_zones >> 16) & 0xFF);
    out_params[5] = (uint8_t)((amp_zones >> 24) & 0xFF);
    out_params[8] = frequency;
}

// Translate a single trigger block from DualSense to Beitong format
static void translate_ds_trigger_block_to_bt(beitong_bridge_state_t *b, uint8_t side,
                                              uint8_t ds_mode, const uint8_t *ds_params,
                                              uint8_t *out_pkt) {
    if (!b || !out_pkt) return;
    memset(out_pkt, 0, 10);
    out_pkt[0] = 0x02;              // Report ID
    out_pkt[1] = 0x56;              // Trigger command
    out_pkt[2] = side;              // 0 = LT, 1 = RT
    
    // Mode OFF or zero mode
    if (ds_mode == DS_TRIGGER_OFF || ds_mode == 0) {
        out_pkt[3] = BT_TRIGGER_OFF;
        return;
    }
    
    // Mode 0x21 (Feedback) or 0x22 (Bow) -> Beitong Race Resistance (BT_TRIGGER_RACE_RESISTANCE = 1)
    if (ds_mode == DS_TRIGGER_FEEDBACK || ds_mode == DS_TRIGGER_BOW) {
        out_pkt[3] = BT_TRIGGER_RACE_RESISTANCE;
        
        // Extract active start zone from bitmask (bytes 0..1)
        uint16_t active_zones = (uint16_t)ds_params[0] | ((uint16_t)ds_params[1] << 8);
        int start_zone = 0;
        for (int z = 0; z < 10; z++) {
            if (active_zones & (1 << z)) {
                start_zone = z;
                break;
            }
        }
        uint8_t bt_start = scale_range(start_zone, 0, 9, 0, 100);
        
        // Extract 3-bit strength from force_zones (bytes 2..5)
        uint32_t force_zones = (uint32_t)ds_params[2] | ((uint32_t)ds_params[3] << 8) |
                               ((uint32_t)ds_params[4] << 16) | ((uint32_t)ds_params[5] << 24);
        uint8_t ds_strength = ((force_zones >> (3 * start_zone)) & 0x07) + 1;  // 1..8
        int bt_raw_force = scale_range(ds_strength, 1, 8, 15, 100);
        
        // Apply gain & floor
        int boosted_force = (int)((float)bt_raw_force * b->trigger_force_gain + 0.5f);
        if (boosted_force < b->min_force_floor) boosted_force = b->min_force_floor;
        uint8_t bt_force = clamp_u8(boosted_force, 0, 100);
        
        out_pkt[4] = bt_start;
        out_pkt[5] = bt_force;
        out_pkt[6] = 0;  // Padding
        return;
    }
    
    // Mode 0x25 (Weapon) -> Beitong Shot Breakpoint (BT_TRIGGER_SHOT_BREAKPOINT = 2)
    if (ds_mode == DS_TRIGGER_WEAPON) {
        out_pkt[3] = BT_TRIGGER_SHOT_BREAKPOINT;
        
        uint16_t zones = (uint16_t)ds_params[0] | ((uint16_t)ds_params[1] << 8);
        int start_zone = 2;
        int end_zone = 7;
        int found_first = 0;
        for (int z = 0; z < 10; z++) {
            if (zones & (1 << z)) {
                if (!found_first) { start_zone = z; found_first = 1; }
                else { end_zone = z; break; }
            }
        }
        
        uint8_t ds_str = (ds_params[2] & 0x07) + 1;  // 1..8
        uint8_t bt_start = scale_range(start_zone, 0, 9, 0, 100);
        uint8_t bt_click = scale_range(end_zone, 0, 9, bt_start + 10, 100);
        
        int bt_raw_force = scale_range(ds_str, 1, 8, 25, 100);
        int boosted_force = (int)((float)bt_raw_force * b->trigger_force_gain + 0.5f);
        if (boosted_force < b->min_force_floor) boosted_force = b->min_force_floor;
        uint8_t bt_force = clamp_u8(boosted_force, 0, 100);
        
        out_pkt[4] = bt_start;
        out_pkt[5] = bt_click;
        out_pkt[6] = bt_force;
        out_pkt[7] = bt_force / 2;  // Return spring strength
        out_pkt[8] = 1;             // Telemetry flag
        return;
    }
    
    // Mode 0x26 (Vibration) / 0x27 (Machine) -> Beitong Gun Vibration (BT_TRIGGER_GUN_VIBRATION = 3)
    if (ds_mode == DS_TRIGGER_VIBRATION || ds_mode == DS_TRIGGER_MACHINE) {
        out_pkt[3] = BT_TRIGGER_GUN_VIBRATION;
        
        uint16_t active_zones = (uint16_t)ds_params[0] | ((uint16_t)ds_params[1] << 8);
        int start_zone = 0;
        for (int z = 0; z < 10; z++) {
            if (active_zones & (1 << z)) { start_zone = z; break; }
        }
        uint8_t bt_start = scale_range(start_zone, 0, 9, 0, 100);
        
        uint32_t amp_zones = (uint32_t)ds_params[2] | ((uint32_t)ds_params[3] << 8) |
                             ((uint32_t)ds_params[4] << 16) | ((uint32_t)ds_params[5] << 24);
        uint8_t ds_amp = ((amp_zones >> (3 * start_zone)) & 0x07) + 1;  // 1..8
        uint8_t ds_freq = ds_params[8];  // Hertz rate
        
        int bt_raw_amp = scale_range(ds_amp, 1, 8, 20, 100);
        int boosted_amp = (int)((float)bt_raw_amp * b->trigger_vibe_gain + 0.5f);
        if (boosted_amp < b->min_force_floor) boosted_amp = b->min_force_floor;
        uint8_t bt_amp = clamp_u8(boosted_amp, 0, 100);
        
        uint8_t bt_freq = scale_range(ds_freq > 0 ? ds_freq : 100, 10, 240, 5, 100);
        
        out_pkt[4] = bt_start;
        out_pkt[5] = bt_freq;
        out_pkt[6] = bt_amp;
        out_pkt[7] = 0;
        return;
    }
    
    out_pkt[3] = BT_TRIGGER_OFF;
}

// Main translation function: DualSense Report 0x02 -> Beitong packets
int beitong_translate_game_to_beitong(beitong_bridge_state_t *b,
                                       const uint8_t *ds_report, size_t ds_len,
                                       beitong_translation_result_t *result) {
    if (!b || !ds_report || ds_len < 32 || !result) return 0;
    
    memset(result, 0, sizeof(*result));
    
    // Identify report type
    const uint8_t report_id = ds_report[0];
    const ds_output_common_t *common = NULL;
    
    if (report_id == 0x02) {
        // USB Report
        if (ds_len < sizeof(ds_output_common_t) + 20) return 0;  // minimum size
        common = (const ds_output_common_t *)(ds_report);
    } else if (report_id == 0x31) {
        // Bluetooth Report
        if (ds_len < sizeof(ds_output_common_t) + 31) return 0;
        common = (const ds_output_common_t *)(ds_report);
    } else {
        return 0;
    }
    
    int pkt_count = 0;
    
    // 1. Right Trigger
    if (common->valid_flag0 & DS_FLAG0_RIGHT_TRIGGER_ENABLE) {
        translate_ds_trigger_block_to_bt(b, 1, common->right_trigger_mode,
                                          common->right_trigger_params,
                                          result->trigger_pkts[pkt_count].data);
        result->trigger_pkts[pkt_count].len = 9;  // 0x02 + 0x56 + side + mode + 4 params
        pkt_count++;
    }
    
    // 2. Left Trigger
    if (common->valid_flag0 & DS_FLAG0_LEFT_TRIGGER_ENABLE) {
        translate_ds_trigger_block_to_bt(b, 0, common->left_trigger_mode,
                                          common->left_trigger_params,
                                          result->trigger_pkts[pkt_count].data);
        result->trigger_pkts[pkt_count].len = 9;
        pkt_count++;
    }
    
    // 3. Grip Rumble (motor_right / motor_left)
    if (common->valid_flag0 & DS_FLAG0_COMPATIBLE_VIBRATION) {
        uint8_t max_motor = common->motor_left > common->motor_right ? common->motor_left : common->motor_right;
        
        // Apply curve + gain
        uint8_t mapped_val = apply_spline(b->curve_type, max_motor);
        int boosted_rumble = (int)((float)mapped_val * b->rumble_gain + 0.5f);
        uint8_t final_rumble = clamp_u8(boosted_rumble, 0, 100);
        
        result->rumble_pkt.data[0] = 0x02;
        result->rumble_pkt.data[1] = 0x76;
        result->rumble_pkt.data[2] = 0x01;
        result->rumble_pkt.data[3] = 0x04;  // CMD 4: Group A rumble
        result->rumble_pkt.data[4] = final_rumble;
        result->rumble_pkt.data[5] = 0x00;
        result->rumble_pkt.len = 6;
        pkt_count++;
    }
    
    return pkt_count;
}

// ============================================================================
// BEITONG -> GAME TRANSLATION
// Translates Beitong 0x02 packets back into DualSense Report 0x01 Input
// ============================================================================

// Translate Beitong input to DualSense input report
int beitong_translate_beitong_to_game(beitong_bridge_state_t *b,
                                      const uint8_t *bt_input_data, size_t bt_len,
                                      uint8_t *out_ds_input_report, size_t out_max_len) {
    if (!b || !bt_input_data || bt_len < 12 || !out_ds_input_report || out_max_len < 64) return 0;
    
    uint8_t rep[64];
    memset(rep, 0, sizeof(rep));
    rep[0] = 0x01;  // Report ID
    
    // Map Sticks (Beitong bytes 0..3 LX, LY, RX, RY centered at 0x80)
    // Beitong uses 0-100 range, DualSense uses 0-255 centered at 0x80
    rep[1] = bt_input_data[0];  // LX (0-100 -> 0x00-0x64, we'll offset)
    rep[2] = bt_input_data[1];  // LY
    rep[3] = bt_input_data[2];  // RX
    rep[4] = bt_input_data[3];  // RY
    
    // Map Analog Triggers (LT at bt[4], RT at bt[5])
    // Beitong triggers are 0-100% DualSense is 0-255
    rep[5] = bt_input_data[4];  // LT (0-255 scale after conversion)
    rep[6] = bt_input_data[5];  // RT (0-255 scale)
    
    // Map Sequence Counter
    rep[7] = (uint8_t)(b->bt_sequence++);
    
    // Map Buttons (Beitong buttons bitmask at bt[6..7])
    // DualSense Byte 8 layout:
    // Bits 0-3: D-pad (0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW, 8=None)
    // Bit 4: Square (X on Xbox/Beitong)
    // Bit 5: Cross (A)
    // Bit 6: Circle (B)
    // Bit 7: Triangle (Y)
    uint16_t bt_btns = (uint16_t)bt_input_data[6] | ((uint16_t)bt_input_data[7] << 8);
    
    uint8_t dpad = 8;  // Neutral
    bool up    = (bt_btns & (1 << 0)) != 0;
    bool down  = (bt_btns & (1 << 1)) != 0;
    bool left  = (bt_btns & (1 << 2)) != 0;
    bool right = (bt_btns & (1 << 3)) != 0;
    
    if (up && right) dpad = 1;
    else if (down && right) dpad = 3;
    else if (down && left) dpad = 5;
    else if (up && left) dpad = 7;
    else if (up) dpad = 0;
    else if (right) dpad = 2;
    else if (down) dpad = 4;
    else if (left) dpad = 6;
    
    rep[8] = dpad & 0x0F;
    if (bt_btns & (1 << 6)) rep[8] |= (1 << 4);  // X -> Square
    if (bt_btns & (1 << 4)) rep[8] |= (1 << 5);  // A -> Cross
    if (bt_btns & (1 << 5)) rep[8] |= (1 << 6);  // B -> Circle
    if (bt_btns & (1 << 7)) rep[8] |= (1 << 7);  // Y -> Triangle
    
    // Byte 9:
    // Bit 0: L1 (LB)
    // Bit 1: R1 (RB)
    // Bit 2: L2 (Digital threshold)
    // Bit 3: R2 (Digital threshold)
    // Bit 4: Create / Share (Back)
    // Bit 5: Options (Start)
    // Bit 6: L3 (LS)
    // Bit 7: R3 (RS)
    if (bt_btns & (1 << 8))  rep[9] |= (1 << 0);  // LB -> L1
    if (bt_btns & (1 << 9))  rep[9] |= (1 << 1);  // RB -> R1
    if (rep[5] > 0x10)       rep[9] |= (1 << 2);  // L2
    if (rep[6] > 0x10)       rep[9] |= (1 << 3);  // R2
    if (bt_btns & (1 << 10)) rep[9] |= (1 << 4);  // Back -> Share
    if (bt_btns & (1 << 11)) rep[9] |= (1 << 5);  // Start -> Options
    if (bt_btns & (1 << 12)) rep[9] |= (1 << 6);  // LS -> L3
    if (bt_btns & (1 << 13)) rep[9] |= (1 << 7);  // RS -> R3
    
    // Byte 10: PS button & Touchpad click
    if (bt_btns & (1 << 14)) rep[10] |= (1 << 0);  // Home -> PS
    
    // Battery state
    rep[53] = 0x0A;  // 100% battery, discharging
    
    memcpy(out_ds_input_report, rep, 64);
    memcpy(b->ds_input_report, rep, 64);
    return 64;
}

// ============================================================================
// BUILD CURRENT REPORT FOR BEITONG CONTROLLER
// Builds the output report that gets fed to the Beitong controller
// ============================================================================

int beitong_build_current_report(beitong_bridge_state_t *b, uint8_t *out_data, size_t out_max_len) {
    if (!b || !out_data || out_max_len < sizeof(ds_usb_output_report_t)) return 0;
    
    ds_usb_output_report_t rep;
    memset(&rep, 0, sizeof(rep));
    rep.report_id = 0x02;
    
    rep.common.valid_flag0 = DS_FLAG0_COMPATIBLE_VIBRATION | DS_FLAG0_RIGHT_TRIGGER_ENABLE | DS_FLAG0_LEFT_TRIGGER_ENABLE;
    rep.common.motor_right = b->motor_right;
    rep.common.motor_left = b->motor_left;
    
    rep.common.right_trigger_mode = b->rt_mode;
    memcpy(rep.common.right_trigger_params, b->rt_params, 10);
    
    rep.common.left_trigger_mode = b->lt_mode;
    memcpy(rep.common.left_trigger_params, b->lt_params, 10);
    
    memcpy(out_data, &rep, sizeof(rep));
    return (int)sizeof(rep);
}

// ============================================================================
// TRANSLATE BEITONG PACKET -> BUILD CURRENT REPORT
// Handles incoming Beitong packets and converts to DualSense report
// ============================================================================

int beitong_translate_packet(beitong_bridge_state_t *b, const uint8_t *in_data, size_t in_len,
                              uint8_t *out_data, size_t out_max_len) {
    if (!b || !in_data || in_len < 4 || in_data[0] != 0x02) return 0;
    
    uint8_t cmd = in_data[1];
    
    // 1. Trigger Command: 02 56 <side> <mode> <params...>
    if (cmd == 0x56) {
        uint8_t side = in_data[2];  // 0 = Left, 1 = Right
        uint8_t mode = in_data[3];
        const uint8_t *params = in_data + 4;
        size_t param_len = in_len - 4;
        
        if (side == 0) {
            translate_bt_trigger(b, 0, mode, params, param_len, &b->lt_mode, b->lt_params);
        } else {
            translate_bt_trigger(b, 1, mode, params, param_len, &b->rt_mode, b->rt_params);
        }
        return beitong_build_current_report(b, out_data, out_max_len);
    }
    
    // 2. Rumble Command: 02 76 01 <subcmd> <params...>
    if (cmd == 0x76 && in_len >= 4 && in_data[2] == 0x01) {
        uint8_t sub = in_data[3];
        uint8_t val = (in_len > 4) ? in_data[4] : 0;
        
        // Apply spline curve + gain multiplier
        uint8_t mapped_val = apply_spline(b->curve_type, val);
        int boosted_rumble = (int)((float)mapped_val * b->rumble_gain + 0.5f);
        uint8_t final_rumble = clamp_u8(boosted_rumble, 0, 100);
        
        if (sub == 4 || sub == 0 || sub == 2) {
            b->motor_right = final_rumble;
        } else if (sub == 5 || sub == 1 || sub == 3) {
            b->motor_left = final_rumble;
        }
        
        return beitong_build_current_report(b, out_data, out_max_len);
    }
    
    return 0;
}

// Helper: translate BT trigger from Beitong format back to DualSense params
void translate_bt_trigger(beitong_bridge_state_t *b, uint8_t side, uint8_t mode,
                                  const uint8_t *params, size_t param_len,
                                  uint8_t *target_mode, uint8_t *target_params) {
    if (!b || !target_mode || !target_params) return;
    
    if (mode == BT_TRIGGER_OFF || param_len == 0) {
        *target_mode = DS_TRIGGER_OFF;
        memset(target_params, 0, 10);
        return;
    }
    
    if (mode == BT_TRIGGER_RACE_RESISTANCE) {
        // Mode 1: [start_pos (0-100), force (0-100)]
        uint8_t p_pos = param_len > 0 ? params[0] : 0;
        uint8_t p_force = param_len > 1 ? params[1] : 50;
        
        uint8_t ds_pos = scale_range(p_pos, 0, 100, 0, 9);
        int raw_str = scale_range(p_force, 0, 100, 1, 8);
        int boosted_str = (int)((float)raw_str * b->trigger_force_gain + 0.5f);
        if (boosted_str < b->min_force_floor) boosted_str = b->min_force_floor;
        uint8_t ds_str = (boosted_str > 8) ? 8 : boosted_str;
        
        encode_ds_feedback(ds_pos, ds_str, target_mode, target_params);
    }
    else if (mode == BT_TRIGGER_SHOT_BREAKPOINT) {
        // Mode 2: [start_pos, click_pos, force]
        uint8_t p_start = param_len > 0 ? params[0] : 20;
        uint8_t p_end = param_len > 1 ? params[1] : 70;
        uint8_t p_force = param_len > 2 ? params[2] : 80;
        
        uint8_t ds_start = scale_range(p_start, 0, 100, 2, 7);
        uint8_t ds_end = scale_range(p_end, 0, 100, ds_start + 1, 8);
        int raw_force = scale_range(p_force, 0, 100, 1, 8);
        int boosted_force = (int)((float)raw_force * b->trigger_force_gain + 0.5f);
        if (boosted_force < b->min_force_floor) boosted_force = b->min_force_floor;
        uint8_t ds_force = (boosted_force > 8) ? 8 : boosted_force;
        
        encode_ds_weapon(ds_start, ds_end, ds_force, target_mode, target_params);
    }
    else if (mode == BT_TRIGGER_GUN_VIBRATION) {
        // Mode 3: [start_pos, freq, strength]
        uint8_t p_pos = param_len > 0 ? params[0] : 0;
        uint8_t p_freq = param_len > 1 ? params[1] : 50;
        uint8_t p_amp = param_len > 2 ? params[2] : 90;
        
        uint8_t ds_pos = scale_range(p_pos, 0, 100, 0, 9);
        int raw_amp = scale_range(p_amp, 0, 100, 1, 8);
        int boosted_amp = (int)((float)raw_amp * b->trigger_vibe_gain + 0.5f);
        if (boosted_amp < b->min_force_floor) boosted_amp = b->min_force_floor;
        uint8_t ds_amp = (boosted_amp > 8) ? 8 : boosted_amp;
        
        uint8_t ds_freq = scale_range(p_freq, 0, 100, 10, 240);
        encode_ds_vibration(ds_pos, ds_amp, ds_freq, target_mode, target_params);
    }
    else {
        *target_mode = DS_TRIGGER_OFF;
        memset(target_params, 0, 10);
    }
}