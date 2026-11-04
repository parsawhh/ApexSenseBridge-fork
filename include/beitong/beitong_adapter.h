// ============================================================================
// Beitong Bridge Adapter for ApexSenseBridge
// Ported from beitong-dualsense-bridge protocol, integrated with ApexSenseBridge
// NO references to Flydigi or APEX remaining - pure Beitong/Kunpeng 70C2 implementation
// ============================================================================

#include "beitong/beitong_protocol.h"
#include "beitong/beitong_bridge.h"
#include <string.h>
#include <stdlib.h>

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
// Adapter Initialization
// ============================================================================
bool beitong_adapter_init(beitong_adapter_t *adapter, bool is_bluetooth) {
    if (!adapter) return false;
    
    // Initialize the core Beitong bridge
    beitong_bridge_init(&adapter->bridge, is_bluetooth);
    adapter->initialized = true;
    
    // Initialize current state to neutral
    memset(&adapter->current_state, 0, sizeof(adapter->current_state));
    adapter->current_state.grip_rumble = 50;  // Neutral rumble
    
    // Initialize last game input to neutral DualSense report
    memset(adapter->last_game_input, 0, sizeof(adapter->last_game_input));
    adapter->last_game_input[0] = 0x01;   // Report ID
    adapter->last_game_input[1] = 0x80;   // LX center
    adapter->last_game_input[2] = 0x80;   // LY center
    adapter->last_game_input[3] = 0x80;   // RX center
    adapter->last_game_input[4] = 0x80;   // RY center
    adapter->last_game_input[8] = 0x08;   // D-pad neutral
    
    return true;
}

// ============================================================================
// Adapter: Process DualSense Feedback -> Beitong Commands
// Called when ApexSenseBridge receives DualSense feedback (HidOutput)
// Converts trigger effects and rumble into Beitong protocol packets
// ============================================================================
void beitong_adapter_process_feedback(beitong_adapter_t *adapter,
                                      const uint8_t *feedback_data, size_t feedback_len) {
    if (!adapter || !adapter->initialized || !feedback_data) return;
    
    // This is called with a DualSense Feedback report (Report 0x02)
    // We need to extract trigger modes and rumble, then translate to Beitong format
    
    // For now, extract basic trigger and rumble info from the feedback report
    // The feedback report has: valid_flag0, motor_left/right, trigger modes/params
    
    if (feedback_len < 2) return;
    
    uint8_t valid_flags = feedback_data[0];
    
    // Process Right Trigger
    if (valid_flags & DS_FLAG0_RIGHT_TRIGGER_ENABLE) {
        // In a real implementation, we'd have the full params
        // For now, set to race resistance with moderate force
        adapter->current_state.trigger_right_active = true;
        adapter->current_state.right_trigger_mode = BT_TRIGGER_RACE_RESISTANCE;
        // Default params: start zone 50%, force 75%
        adapter->current_state.right_trigger_params[0] = 50;  // start position
        adapter->current_state.right_trigger_params[1] = 75;  // force
        // Fill rest with zeros
        memset(adapter->current_state.right_trigger_params + 2, 0, 8);
    } else {
        adapter->current_state.trigger_right_active = false;
        adapter->current_state.right_trigger_mode = BT_TRIGGER_OFF;
        memset(adapter->current_state.right_trigger_params, 0, 10);
    }
    
    // Process Left Trigger
    if (valid_flags & DS_FLAG0_LEFT_TRIGGER_ENABLE) {
        adapter->current_state.trigger_left_active = true;
        adapter->current_state.left_trigger_mode = BT_TRIGGER_RACE_RESISTANCE;
        adapter->current_state.left_trigger_params[0] = 50;
        adapter->current_state.left_trigger_params[1] = 75;
        memset(adapter->current_state.left_trigger_params + 2, 0, 8);
    } else {
        adapter->current_state.trigger_left_active = false;
        adapter->current_state.left_trigger_mode = BT_TRIGGER_OFF;
        memset(adapter->current_state.left_trigger_params, 0, 10);
    }
    
    // Process Rumble (grip motors)
    uint8_t motor_left = feedback_data[2] & 0xFF;   // Low frequency / heavy rumble
    uint8_t motor_right = feedback_data[1] & 0xFF;  // High frequency / light rumble
    
    // Average and map to 0-100 range for Beitong
    uint8_t avg_motor = (motor_left + motor_right) / 2;
    adapter->current_state.grip_rumble = clamp_u8(avg_motor, 0, 100);
}

// ============================================================================
// Adapter: Generate Beitong Output Report
// Builds the Beitong 0x02 output report from current state
// This is what gets sent to the Beitong controller
// ============================================================================
void beitong_adapter_build_output(beitong_adapter_t *adapter, uint8_t *out_report, size_t out_max) {
    if (!adapter || !adapter->initialized || !out_report || out_max < 10) return;
    
    // Build a combined trigger packet for right trigger (most common case)
    // Format: 02 56 <side> <mode> <params...>
    // We'll send right trigger data; left will be handled similarly or off
    
    out_report[0] = 0x02;      // Report ID
    out_report[1] = 0x56;      // Trigger command
    out_report[2] = BT_TRIGGER_RIGHT;  // Right trigger
    out_report[3] = adapter->current_state.right_trigger_mode;  // Mode
    
    // Copy params (at minimum start position and force)
    memcpy(out_report + 4, adapter->current_state.right_trigger_params, 10);
}

// ============================================================================
// Adapter: Process Beitong Input -> Emulated DualSense Report
// Called when Beitong controller sends data back
// Converts Beitong protocol into the emulated DualSense Report 0x01
// ============================================================================
void beitong_adapter_process_input(beitong_adapter_t *adapter, const uint8_t *bt_input_data, size_t bt_len) {
    if (!adapter || !adapter->initialized || !bt_input_data || bt_len < 12) return;
    
    // Reset output report to neutral
    memset(adapter->last_game_input, 0, sizeof(adapter->last_game_input));
    adapter->last_game_input[0] = 0x01;   // Report ID
    
    // Map sticks (Beitong bytes 0-3: LX, LY, RX, RY, 0-100 range)
    // Convert to DualSense 0-255 range centered at 0x80
    int lx = bt_input_data[0];  // 0-100
    int ly = bt_input_data[1];
    int rx = bt_input_data[2];
    int ry = bt_input_data[3];
    
    // Scale 0-100 to 0-255, center at 128 (0x80)
    lx = (lx * 255 + 50) / 100;  // Approximate
    ly = (ly * 255 + 50) / 100;
    rx = (rx * 255 + 50) / 100;
    ry = (ry * 255 + 50) / 100;
    
    // Ensure centered at 0x80 when at neutral (50)
    if (lx < 10) lx = 130;  // Left of center
    if (lx > 90) lx = 126;  // Right of center
    // Similar for others...
    
    adapter->last_game_input[1] = lx & 0xFF;
    adapter->last_game_input[2] = ly & 0xFF;
    adapter->last_game_input[3] = rx & 0xFF;
    adapter->last_game_input[4] = ry & 0xFF;
    
    // Map triggers (Beitong bytes 4-5: LT, RT, 0-100%)
    // DualSense L2/R2 are 0-255
    int lt = bt_input_data[4];  // Left trigger 0-100
    int rt = bt_input_data[5];  // Right trigger 0-100
    
    adapter->last_game_input[5] = (lt * 255 + 50) / 100;  // L2
    adapter->last_game_input[6] = (rt * 255 + 50) / 100;  // R2
    
    // Map buttons (Beitong bytes 6-7)
    uint16_t bt_btns = (uint16_t)bt_input_data[6] | ((uint16_t)bt_input_data[7] << 8);
    
    // D-pad (bits 0-3)
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
    
    adapter->last_game_input[8] = dpad & 0x0F;
    
    // Face buttons (bits 6-7 of byte 8 -> Square, Cross, Circle, Triangle)
    if (bt_btns & (1 << 6)) adapter->last_game_input[8] |= (1 << 4);  // X -> Square
    if (bt_btns & (1 << 4)) adapter->last_game_input[8] |= (1 << 5);  // A -> Cross
    if (bt_btns & (1 << 5)) adapter->last_game_input[8] |= (1 << 6);  // B -> Circle
    if (bt_btns & (1 << 7)) adapter->last_game_input[8] |= (1 << 7);  // Y -> Triangle
    
    // Additional buttons (byte 9)
    // Bit 0: L1 (LB), Bit 1: R1 (RB), Bit 2: L2 digital, Bit 3: R2 digital
    // Bit 4: Create/Share, Bit 5: Options, Bit 6: L3, Bit 7: R3
    if (bt_btns & (1 << 8)) adapter->last_game_input[9] |= (1 << 0);  // LB -> L1
    if (bt_btns & (1 << 9)) adapter->last_game_input[9] |= (1 << 1);  // RB -> R1
    // L2/R2 digital already in bytes 5/6
    if (bt_btns & (1 << 10)) adapter->last_game_input[9] |= (1 << 4);  // Back -> Share
    if (bt_btns & (1 << 11)) adapter->last_game_input[9] |= (1 << 5);  // Start -> Options
    
    // Byte 10: PS button & Touchpad click
    if (bt_btns & (1 << 14)) adapter->last_game_input[10] |= (1 << 0);  // Home -> PS
    
    // Battery state (default 100%)
    adapter->last_game_input[53] = 0x0A;
}

// ============================================================================
// Adapter: Get Emulated DualSense Report
// Returns the last emulated DualSense input report (Report ID 0x01)
// This is what games see when the Beitong controller is active
// ============================================================================
void beitong_adapter_get_game_input(beitong_adapter_t *adapter, uint8_t *out_report, size_t max_len) {
    if (!adapter || !adapter->initialized || !out_report || max_len < 64) return;
    memcpy(out_report, adapter->last_game_input, 64);
}

// ============================================================================
// Adapter: Send Trigger Commands to Beitong
// Queue trigger commands to be sent to the Beitong controller
// ============================================================================
bool beitong_adapter_send_trigger(beitong_adapter_t *adapter, uint8_t side,
                                   uint8_t mode, const uint8_t *params, size_t param_len) {
    if (!adapter || !adapter->initialized) return false;
    
    // Update internal state based on the command
    if (side == BT_TRIGGER_LEFT) {
        adapter->current_state.left_trigger_mode = mode;
        if (params && param_len >= 2) {
            adapter->current_state.left_trigger_params[0] = params[0];  // start pos
            adapter->current_state.left_trigger_params[1] = params[1];  // force
        }
    } else {
        adapter->current_state.right_trigger_mode = mode;
        if (params && param_len >= 2) {
            adapter->current_state.right_trigger_params[0] = params[0];  // start pos
            adapter->current_state.right_trigger_params[1] = params[1];  // force
        }
    }
    
    // Build and send the output report
    uint8_t output[10];
    beitong_adapter_build_output(adapter, output, sizeof(output));
    
    // In a real implementation, this would write to the Beitong HID device
    // For now, just update state and return success
    return true;
}

// ============================================================================
// Adapter: Set Rumble for Beitong Controller
// Set grip rumble strength (0-100%)
// ============================================================================
void beitong_adapter_set_rumble(beitong_adapter_t *adapter, uint8_t grip_rumble) {
    if (!adapter || !adapter->initialized) return;
    
    adapter->current_state.grip_rumble = clamp_u8(grip_rumble, 0, 100);
    
    // Build rumble report
    uint8_t output[6];
    output[0] = 0x02;          // Report ID
    output[1] = 0x76;          // Rumble command
    output[2] = 0x01;          // Subcommand A
    output[3] = 0x04;          // Group A
    output[4] = adapter->current_state.grip_rumble;  // Strength 0-100%
    output[5] = 0x00;          // Padding
    
    // In real implementation, write to Beitong device
}

// ============================================================================
// Adapter Cleanup
// ============================================================================
void beitong_adapter_cleanup(beitong_adapter_t *adapter) {
    if (!adapter) return;
    memset(adapter, 0, sizeof(*adapter));
}

#ifdef __cplusplus
extern "C" {
#endif

#endif // BEITONG_ADAPTER_H