#pragma once

#include "beitong/beitong_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    beitong_bridge_state_t bridge;
    bool initialized;
    struct {
        bool trigger_left_active;
        bool trigger_right_active;
        uint8_t left_trigger_mode;
        uint8_t left_trigger_params[10];
        uint8_t right_trigger_mode;
        uint8_t right_trigger_params[10];
        uint8_t grip_rumble;
    } current_state;
    uint8_t last_game_input[64];
} beitong_adapter_t;

bool beitong_adapter_init(beitong_adapter_t *adapter, bool is_bluetooth);
// feedback_data must be a complete DualSense output report (USB 0x02 or BT 0x31).
void beitong_adapter_process_feedback(beitong_adapter_t *adapter,
                                      const uint8_t *feedback_data, size_t feedback_len);
// Legacy single-command interface: returns the current RT command, or LT when
// RT is neutral. The bidirectional translation API returns all generated packets.
void beitong_adapter_build_output(beitong_adapter_t *adapter,
                                  uint8_t *out_report, size_t out_max);
void beitong_adapter_process_input(beitong_adapter_t *adapter,
                                   const uint8_t *bt_input_data, size_t bt_len);
void beitong_adapter_get_game_input(beitong_adapter_t *adapter,
                                    uint8_t *out_report, size_t max_len);
bool beitong_adapter_send_trigger(beitong_adapter_t *adapter, uint8_t side,
                                  uint8_t mode, const uint8_t *params,
                                  size_t param_len);
void beitong_adapter_set_rumble(beitong_adapter_t *adapter, uint8_t grip_rumble);
void beitong_adapter_cleanup(beitong_adapter_t *adapter);

#ifdef __cplusplus
}
#endif
