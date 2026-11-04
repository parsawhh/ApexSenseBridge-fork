#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// Beitong KP70C2 Hardware IDs
// ============================================================================
#define BEITONG_VENDOR_ID                0x20BC
#define BEITONG_PID_KP70C2_PAD           0x518C
#define BEITONG_PID_KP70C2_DONGLE        0x518D
#define BEITONG_PID_ASURA_KP70           0x5126
#define BEITONG_PID_ZEUS_KP70            0x5125

// ============================================================================
// DualSense Hardware IDs (emulated)
// ============================================================================
#define SONY_VENDOR_ID                   0x054C
#define DUALSENSE_PRODUCT_ID             0x0CE6

// ============================================================================
// DualSense Output Report Valid Flags
// ============================================================================
#define DS_FLAG0_COMPATIBLE_VIBRATION    (1 << 0)
#define DS_FLAG0_HAPTICS_SELECT          (1 << 1)
#define DS_FLAG0_RIGHT_TRIGGER_ENABLE    (1 << 2)
#define DS_FLAG0_LEFT_TRIGGER_ENABLE     (1 << 3)

// ============================================================================
// Beitong Adaptive Trigger Protocol
// ============================================================================
// Format: 02 56 <side> <mode> <params...>
// side: 0x00 = Left Trigger (LT), 0x01 = Right Trigger (RT)

#define DS_TRIGGER_OFF                       0x05 // Resets stops to neutral
#define DS_TRIGGER_FEEDBACK                  0x21 // Continuous resistance curve
#define DS_TRIGGER_BOW                       0x22 // Tension build + snap-back
#define DS_TRIGGER_GALLOPING                 0x23 // Rhythmic step oscillation
#define DS_TRIGGER_WEAPON                    0x25 // Semi-auto weapon click & sudden release
#define DS_TRIGGER_VIBRATION                 0x26 // High-frequency cycling vibration
#define DS_TRIGGER_MACHINE                   0x27 // Dual-frequency mechanical rumble

typedef enum {
    BT_TRIGGER_OFF           = 0x00,
    BT_TRIGGER_RACE_RESISTANCE = 0x01,   // Continuous damping
    BT_TRIGGER_SHOT_BREAKPOINT = 0x02,   // Two-stage tactile wall
    BT_TRIGGER_GUN_VIBRATION = 0x03      // Recoil pulsing
} bt_trigger_mode_t;

// Format: 02 76 01 <subcmd> <params...>  (Rumble)
// subcmd: 0=right, 1=left, 2=right, 3=left, 4=right group A
typedef enum {
    BT_RUMBLE_SUBCMD_RIGHT_A  = 0x04,
    BT_RUMBLE_SUBCMD_LEFT_A   = 0x05,
    BT_RUMBLE_SUBCMD_RIGHT_B  = 0x02,
    BT_RUMBLE_SUBCMD_LEFT_B   = 0x03,
    BT_RUMBLE_SUBCMD_NEUTRAL  = 0x00
} bt_rumble_subcmd_t;

// ============================================================================
// DualSense USB Output Report (Report ID 0x02)
// ============================================================================
typedef struct {
    uint8_t  report_id;           // 0x02
    uint8_t  valid_flag0;
    uint8_t  motor_right;        // High frequency / light grip rumble (0-255)
    uint8_t  motor_left;         // Low frequency / heavy grip rumble (0-255)
    uint8_t  headphone_volume;
    uint8_t  speaker_volume;
    uint8_t  internal_mic_volume;
    uint8_t  audio_flags;
    uint8_t  mute_button_led;
    uint8_t  power_save_control;

    // Right Trigger (11 bytes: mode + 10 params)
    uint8_t  right_trigger_mode;
    uint8_t  right_trigger_params[10];

    // Left Trigger (11 bytes: mode + 10 params)
    uint8_t  left_trigger_mode;
    uint8_t  left_trigger_params[10];

    uint32_t host_timestamp;
    uint8_t  reduce_motor_power;
    uint8_t  audio_flags2;
    uint8_t  valid_flag2;
    uint8_t  haptics_flags;
    uint8_t  reserved3;
    uint8_t  lightbar_setup;
    uint8_t  led_brightness;
    uint8_t  player_leds;
    uint8_t  lightbar_red;
    uint8_t  lightbar_green;
    uint8_t  lightbar_blue;
} ds_output_common_t;

typedef struct {
    uint8_t report_id;            // 0x02
    ds_output_common_t common;
    uint8_t padding[16];          // Pads to 64 bytes total
} ds_usb_output_report_t;

// ============================================================================
// DualSense Bluetooth Output Report (Report ID 0x31)
// ============================================================================
typedef struct {
    uint8_t report_id;            // 0x31
    uint8_t seq_tag;              // (seq << 4) | 0x00
    uint8_t tag;                  // 0x10
    ds_output_common_t common;
    uint8_t padding[24];
    uint32_t crc32;               // 0xA2 Seeded CRC32
} ds_bt_output_report_t;

// ============================================================================
// DualSense Input Report (Report ID 0x01) - what host sees
// ============================================================================
typedef struct {
    uint8_t  report_id;           // 0x01
    int16_t  lx;                  // Left stick X (-128..127, centered at 0)
    int16_t  ly;                  // Left stick Y
    int16_t  rx;                  // Right stick X
    int16_t  ry;                  // Right stick Y
    uint8_t  l2;                  // Left trigger (0-255)
    uint8_t  r2;                  // Right trigger (0-255)
    uint8_t  dpad;                // D-pad (0-7, 8=neutral)
    uint8_t  buttons;             // Face buttons (15 bits)
    uint8_t  extra_buttons;       // Additional buttons (16 bits)
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    uint8_t  battery_percentage;
    uint8_t  charge_state;        // 0=discharging, 1=charging, 2=full
} ds_input_report_t;

// ============================================================================
// Controllers state tracked by the bridge
// ============================================================================
typedef struct {
    bool is_bluetooth;
    uint8_t bt_sequence;
    
    // Deadzone settings
    uint8_t lt_deadzone_start;
    uint8_t lt_deadzone_end;
    uint8_t rt_deadzone_start;
    uint8_t rt_deadzone_end;
    
    // Gain profiles (Beitong defaults)
    float trigger_force_gain;     // 1.40f default
    float trigger_vibe_gain;      // 1.45f default
    float rumble_gain;            // 1.30f default
    uint8_t min_force_floor;      // 20 default
    
    // Whether to sync trigger with grip rumble
    bool sync_trigger_with_grip;
    
    // Emulated DualSense input report (what host sees)
    uint8_t ds_input_report[64];
    
    // Curve type
    int curve_type;               // 0=Linear, 1=Intense, 2=Gentle
    
    // Motor states (Beitong side)
    uint8_t motor_left;
    uint8_t motor_right;
    
    // Trigger mode state
    uint8_t lt_mode;
    uint8_t lt_params[10];
    uint8_t rt_mode;
    uint8_t rt_params[10];
    
    // Sequence tracking
    uint64_t input_updates;
    uint64_t output_reports;
} beitong_bridge_state_t;

// ============================================================================
// Translation result - packets to send to Beitong controller
// ============================================================================
#define MAX_BT_PACKETS 3
typedef struct {
    uint8_t data[10];
    size_t len;
} beitong_trigger_pkt_t;

typedef struct {
    beitong_trigger_pkt_t trigger_pkts[MAX_BT_PACKETS];
    size_t trigger_count;
    beitong_trigger_pkt_t rumble_pkt;
    size_t rumble_len;
} beitong_translation_result_t;

/* Prototypes */
void beitong_bridge_init(beitong_bridge_state_t *b, bool is_bluetooth);
void beitong_bridge_set_gains(beitong_bridge_state_t *b, float force_gain, float vibe_gain, float rumble_gain, uint8_t min_floor);
void beitong_bridge_set_curve(beitong_bridge_state_t *b, int curve_type);
void beitong_bridge_set_deadzones(beitong_bridge_state_t *b, uint8_t lt_start, uint8_t lt_end, uint8_t rt_start, uint8_t rt_end);

int beitong_translate_game_to_beitong(beitong_bridge_state_t *b,
                                       const uint8_t *ds_report, size_t ds_len,
                                       beitong_translation_result_t *result);

int beitong_translate_beitong_to_game(beitong_bridge_state_t *b,
                                      const uint8_t *bt_input_data, size_t bt_len,
                                      uint8_t *out_ds_input_report, size_t out_max_len);

uint32_t beitong_crc32(uint8_t seed, const uint8_t *data, size_t len);

