#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Supported Beitong identities. 0x20DD is an alternate vendor ID seen on
// regional/rebranded hardware using the same controller family.
#define BEITONG_VENDOR_ID             0x20BC
#define BEITONG_ALTER_VENDOR_ID       0x20DD
#define BEITONG_PID_ZEUS_KP70         0x5125
#define BEITONG_PID_ASURA_KP70        0x5126
#define BEITONG_PID_KP70_WIRELESS     0x5128
#define BEITONG_PID_KP70C1_PAD        0x518A
#define BEITONG_PID_KP70C1_DONGLE     0x518B
#define BEITONG_PID_KP70C2_PAD        0x518C
#define BEITONG_PID_KP70C2_DONGLE     0x518D

#define SONY_VENDOR_ID                0x054C
#define DUALSENSE_PRODUCT_ID          0x0CE6
#define DUALSENSE_EDGE_PRODUCT_ID     0x0DF2

#define DS_FLAG0_COMPATIBLE_VIBRATION (1u << 0)
#define DS_FLAG0_HAPTICS_SELECT       (1u << 1)
#define DS_FLAG0_RIGHT_TRIGGER_ENABLE (1u << 2)
#define DS_FLAG0_LEFT_TRIGGER_ENABLE  (1u << 3)

#define DS_TRIGGER_OFF                0x05
#define DS_TRIGGER_FEEDBACK           0x21
#define DS_TRIGGER_BOW                0x22
#define DS_TRIGGER_GALLOPING          0x23
#define DS_TRIGGER_WEAPON             0x25
#define DS_TRIGGER_VIBRATION          0x26
#define DS_TRIGGER_MACHINE            0x27

#define BT_TRIGGER_LEFT               0x00
#define BT_TRIGGER_RIGHT              0x01

typedef enum {
    BT_TRIGGER_OFF = 0x00,
    BT_TRIGGER_RACE_RESISTANCE = 0x01,
    BT_TRIGGER_SHOT_BREAKPOINT = 0x02,
    BT_TRIGGER_GUN_VIBRATION = 0x03
} bt_trigger_mode_t;

typedef enum {
    BT_RUMBLE_SUBCMD_RIGHT_A = 0x04,
    BT_RUMBLE_SUBCMD_LEFT_A = 0x05,
    BT_RUMBLE_SUBCMD_RIGHT_B = 0x02,
    BT_RUMBLE_SUBCMD_LEFT_B = 0x03,
    BT_RUMBLE_SUBCMD_NEUTRAL = 0x00
} bt_rumble_subcmd_t;

// Packed DualSense output report common payload. The report ID belongs to
// the outer USB/BT report, not this payload. Keep packed: these bytes are a
// wire protocol and include unaligned 32-bit fields.
#pragma pack(push, 1)
typedef struct {
    uint8_t valid_flag0;
    uint8_t valid_flag1;
    uint8_t motor_right;
    uint8_t motor_left;
    uint8_t headphone_volume;
    uint8_t speaker_volume;
    uint8_t internal_mic_volume;
    uint8_t audio_flags;
    uint8_t mute_button_led;
    uint8_t power_save_control;
    uint8_t right_trigger_mode;
    uint8_t right_trigger_params[10];
    uint8_t left_trigger_mode;
    uint8_t left_trigger_params[10];
    uint32_t host_timestamp;
    uint8_t reduce_motor_power;
    uint8_t audio_flags2;
    uint8_t valid_flag2;
    uint8_t haptics_flags;
    uint8_t reserved3;
    uint8_t lightbar_setup;
    uint8_t led_brightness;
    uint8_t player_leds;
    uint8_t lightbar_red;
    uint8_t lightbar_green;
    uint8_t lightbar_blue;
} ds_output_common_t;

typedef struct {
    uint8_t report_id;
    ds_output_common_t common;
    uint8_t padding[16];
} ds_usb_output_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t seq_tag;
    uint8_t tag;
    ds_output_common_t common;
    uint8_t padding[24];
    uint32_t crc32;
} ds_bt_output_report_t;
#pragma pack(pop)

#define DS_OUTPUT_TAG_BT 0x10
#define DS_BT_CRC_SEED 0xA2

typedef struct {
    bool is_bluetooth;
    uint8_t bt_sequence;
    uint8_t lt_deadzone_start;
    uint8_t lt_deadzone_end;
    uint8_t rt_deadzone_start;
    uint8_t rt_deadzone_end;
    float trigger_force_gain;
    float trigger_vibe_gain;
    float rumble_gain;
    uint8_t min_force_floor;
    bool sync_trigger_with_grip;
    uint8_t ds_input_report[64];
    int curve_type;
    uint8_t motor_left;
    uint8_t motor_right;
    uint8_t lt_mode;
    uint8_t lt_params[10];
    uint8_t rt_mode;
    uint8_t rt_params[10];
    uint64_t input_updates;
    uint64_t output_reports;
} beitong_bridge_state_t;

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

#ifdef __cplusplus
extern "C" {
#endif

void beitong_bridge_init(beitong_bridge_state_t *b, bool is_bluetooth);
void beitong_bridge_set_gains(beitong_bridge_state_t *b, float force_gain, float vibe_gain, float rumble_gain, uint8_t min_floor);
void beitong_bridge_set_curve(beitong_bridge_state_t *b, int curve_type);
void beitong_bridge_set_deadzones(beitong_bridge_state_t *b, uint8_t lt_start, uint8_t lt_end, uint8_t rt_start, uint8_t rt_end);
int beitong_translate_game_to_beitong(beitong_bridge_state_t *b, const uint8_t *ds_report, size_t ds_len, beitong_translation_result_t *result);
int beitong_translate_beitong_to_game(beitong_bridge_state_t *b, const uint8_t *bt_input_data, size_t bt_len, uint8_t *out_ds_input_report, size_t out_max_len);
int beitong_translate_packet(beitong_bridge_state_t *b, const uint8_t *in_data, size_t in_len, uint8_t *out_data, size_t out_max_len);
uint32_t beitong_crc32(uint8_t seed, const uint8_t *data, size_t len);

#ifdef __cplusplus
}

static_assert(sizeof(ds_output_common_t) == 47, "DualSense common payload must match wire format");
static_assert(sizeof(ds_usb_output_report_t) == 64, "DualSense USB output report must be 64 bytes");
static_assert(sizeof(ds_bt_output_report_t) == 78, "DualSense Bluetooth output report must be 78 bytes");
static_assert(offsetof(ds_bt_output_report_t, crc32) == 74, "DualSense Bluetooth CRC must start at byte 74");
#else
_Static_assert(sizeof(ds_output_common_t) == 47, "DualSense common payload must match wire format");
_Static_assert(sizeof(ds_usb_output_report_t) == 64, "DualSense USB output report must be 64 bytes");
_Static_assert(sizeof(ds_bt_output_report_t) == 78, "DualSense Bluetooth output report must be 78 bytes");
_Static_assert(offsetof(ds_bt_output_report_t, crc32) == 74, "DualSense Bluetooth CRC must start at byte 74");
#endif
