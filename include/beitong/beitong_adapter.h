// ============================================================================
// Beitong Bridge Adapter for ApexSenseBridge
// Ported from beitong-dualsense-bridge protocol, integrated with ApexSenseBridge
// NO references to Flydigi or APEX remaining - pure Beitong/Kunpeng 70C2 implementation
// ============================================================================

#ifndef BEITONG_ADAPTER_H
#define BEITONG_ADAPTER_H

#include "beitong/beitong_protocol.h"
#include "beitong/beitong_bridge.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Adapter Context - bridges Beitong protocol with ApexSenseBridge expectations
// ============================================================================
typedef struct {
    beitong_bridge_state_t bridge;      // Core Beitong bridge
    bool initialized;                     // Whether adapter is ready

    // Mapping from ApexSenseBridge Feedback to Beitong commands
    struct {
        bool trigger_left_active;
        bool trigger_right_active;
        uint8_t left_trigger_mode;
        uint8_t left_trigger_params[10];
        uint8_t right_trigger_mode;
        uint8_t right_trigger_params[10];
        uint8_t grip_rumble;          // 0-100%
    } current_state;

    // Mapping from Beitong to ApexSenseBridge input report
    uint8_t last_game_input[64];          // Last emulated DualSense input report
} beitong_adapter_t;

// ============================================================================
// Adapter API (declarations; implementations in src/beitong_bridge/beitong_adapter.c)
// ============================================================================
bool beitong_adapter_init(beitong_adapter_t *adapter, bool is_bluetooth);
void beitong_adapter_process_feedback(beitong_adapter_t *adapter,
                                      const uint8_t *feedback_data, size_t feedback_len);
void beitong_adapter_build_output(beitong_adapter_t *adapter, uint8_t *out_report, size_t out_max);
void beitong_adapter_process_input(beitong_adapter_t *adapter, const uint8_t *bt_input_data, size_t bt_len);
void beitong_adapter_get_game_input(beitong_adapter_t *adapter, uint8_t *out_report, size_t max_len);
bool beitong_adapter_send_trigger(beitong_adapter_t *adapter, uint8_t side,
                                   uint8_t mode, const uint8_t *params, size_t param_len);
void beitong_adapter_set_rumble(beitong_adapter_t *adapter, uint8_t grip_rumble);
void beitong_adapter_cleanup(beitong_adapter_t *adapter);

#ifdef __cplusplus
}
#endif

#endif // BEITONG_ADAPTER_H
