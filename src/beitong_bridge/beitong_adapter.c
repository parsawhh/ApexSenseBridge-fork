// ============================================================================
// Beitong Bridge Adapter Implementation
// Ported from beitong-dualsense-bridge protocol, integrated with ApexSenseBridge
// NO references to Flydigi or APEX remaining - pure Beitong/Kunpeng 70C2 implementation
// ============================================================================

#include "beitong/beitong_adapter.h"

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
    
    if (feedback_len < 2) return;
    
    uint8_t valid_flags = feedback_data[0];
    
    // Process Right Trigger
    if (valid_flags & DS_FLAG0_RIGHT_TRIGGER_ENABLE) {
        adapter->current_state.trigger_right_active = true;
        adapter->current_state.right_trigger_mode = BT_TRIGGER_RACE_RESISTANCE;
        // Default params: start zone 50%, force 75%
        if (feedback_len > 12) {
            adapter->current_state.right_trigger_params[0] = feedback_data[8];  // start zone
            adapter->current_state.right_trigger_params[1] = feedback_data[9];  // force
        } else {
            adapter->current_state.right_trigger_params[0] = 50;
            adapter->current_state.right_trigger_params[1] = 75;
        }
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
        if (feedback_len > 12) {
            adapter->current_state.left_trigger_params[0] = feedback_data[8 + 11];  // left trigger start zone
            adapter->current_state.left_trigger_params[1] = feedback_data[9 + 11];  // left trigger force
        } else {
            adapter->current_state.left_trigger_params[0] = 50;
            adapter->current_state.left_trigger_params[1] = 75;
        }
        memset(adapter->current_state.left_trigger_params + 2, 0, 8);
    } else {
        adapter->current_state.trigger_left_active = false;
        adapter->current_state.left_trigger_mode = BT_TRIGGER_OFF;
        memset(adapter->current_state.left_trigger_params, 0, 10);
    }
    
    // Process Rumble (grip motors)
    // Feedback report layout: offset 1 = motor_right, offset 2 = motor_left
    uint8_t motor_left = 0, motor_right = 0;
    if (feedback_len > 2) {
        motor_left = feedback_data[2] & 0xFF;      // Low frequency / heavy rumble
        motor_right = feedback_data[1] & 0xFF;     // High frequency / light rumble
    }
    
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
    
    // Build a trigger packet for right trigger
    // Format: 02 56 <side> <mode> <params...>
    // Side: 0x01 = Right
    // Mode: current right trigger mode
    // Params: current right trigger params
    
    out_report[0] = 0x02;          // Report ID
    out_report[1] = 0x56;          // Trigger command
    out_report[2] = BT_TRIGGER_RIGHT;  // Right trigger (side = 1)
    out_report[3] = adapter->bridge.rt_mode;  // Mode from bridge state
    
    // Copy params from bridge state
    memcpy(out_report + 4, adapter->bridge.rt_params, 10);
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
    int lx = (int)bt_input_data[0];  // 0-100
    int ly = (int)bt_input_data[1];
    int rx = (int)bt_input_data[2];
    int ry = (int)bt_input_data[3];
    
    // Scale 0-100 to 0-255
    lx = (lx * 255 + 50) / 100;
    ly = (ly * 255 + 50) / 100;
    rx = (rx * 255 + 50) / 100;
    ry = (ry * 255 + 50) / 100;
    
    // Center at 0x80 (128) when at neutral
    // Beitong neutral is ~50, which maps to ~128
    if (lx < 10) lx = 130;   // Far left
    if (lx > 90) lx = 126;   // Far right
    if (ly < 10) ly = 130;   // Far forward
    if (ly > 90) ly = 126;   // Far back
    if (rx < 10) rx = 130;
    if (rx > 90) rx = 126;
    if (ry < 10) ry = 130;
    if (ry > 90) ry = 126;
    
    adapter->last_game_input[1] = (uint8_t)lx;
    adapter->last_game_input[2] = (uint8_t)ly;
    adapter->last_game_input[3] = (uint8_t)rx;
    adapter->last_game_input[4] = (uint8_t)ry;
    
    // Map triggers (Beitong bytes 4-5: LT, RT, 0-100%)
    // DualSense L2/R2 are 0-255
    int lt = (int)bt_input_data[4];  // Left trigger 0-100
    int rt = (int)bt_input_data[5];  // Right trigger 0-100
    
    // Scale to 0-255
    adapter->last_game_input[5] = (uint8_t)((lt * 255 + 50) / 100);
    adapter->last_game_input[6] = (uint8_t)((rt * 255 + 50) / 100);
    
    // Map buttons (Beitong bytes 6-7)
    uint16_t bt_btns = (uint16_t)bt_input_data[6] | ((uint16_t)bt_input_data[7] << 8);
    
    // D-pad (bits 0-3 of byte 6)
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
    
    // Face buttons
    // Byte 8 bits 4-7: Square (X), Cross (A), Circle (B), Triangle (Y)
    if (bt_btns & (1 << 6)) adapter->last_game_input[8] |= (1 << 4);  // X -> Square
    if (bt_btns & (1 << 4)) adapter->last_game_input[8] |= (1 << 5);  // A -> Cross
    if (bt_btns & (1 << 5)) adapter->last_game_input[8] |= (1 << 6);  // B -> Circle
    if (bt_btns & (1 << 7)) adapter->last_game_input[8] |= (1 << 7);  // Y -> Triangle
    
    // Byte 9
    // Bit 0: L1 (LB), Bit 1: R1 (RB)
    // Bit 2: L2 digital threshold, Bit 3: R2 digital threshold
    // These are already reflected in L2/R2 values above (bytes 5/6)
    // Bit 4: Create/Share (Back), Bit 5: Options (Start)
    // Bit 6: L3 (LS), Bit 7: R3 (RS)
    if (bt_btns & (1 << 8)) adapter->last_game_input[9] |= (1 << 0);  // LB -> L1
    if (bt_btns & (1 << 9)) adapter->last_game_input[9] |= (1 << 1);  // RB -> R1
    if (bt_btns & (1 << 10)) adapter->last_game_input[9] |= (1 << 4);  // Back -> Share
    if (bt_btns & (1 << 11)) adapter->last_game_input[9] |= (1 << 5);  // Start -> Options
    if (bt_btns & (1 << 12)) adapter->last_game_input[9] |= (1 << 6);  // LS -> L3
    if (bt_btns & (1 << 13)) adapter->last_game_input[9] |= (1 << 7);  // RS -> R3
    
    // Byte 10: PS button & Touchpad click
    if (bt_btns & (1 << 14)) adapter->last_game_input[10] |= (1 << 0);  // Home -> PS
    
    // Battery state (default 100%)
    adapter->last_game_input[53] = 0x0A;
}

// ============================================================================
// Adapter: Get Emulated DualSense Report
// Returns the last emulated DualSense input report (Report ID 0x01)
// ============================================================================
void beitong_adapter_get_game_input(beitong_adapter_t *adapter, uint8_t *out_report, size_t max_len) {
    if (!adapter || !adapter->initialized || !out_report || max_len < 64) return;
    if (max_len < 64) {
        // Copy what we can
        size_t copy_len = max_len < 64 ? max_len : 64;
        memcpy(out_report, adapter->last_game_input, copy_len);
    } else {
        memcpy(out_report, adapter->last_game_input, 64);
    }
}

// ============================================================================
// Adapter: Send Trigger Commands to Beitong
// ============================================================================
bool beitong_adapter_send_trigger(beitong_adapter_t *adapter, uint8_t side,
                                   uint8_t mode, const uint8_t *params, size_t param_len) {
    if (!adapter || !adapter->initialized) return false;
    
    // Update internal bridge state
    if (side == BT_TRIGGER_LEFT) {
        adapter->bridge.lt_mode = mode;
        if (params && param_len >= 2) {
            adapter->bridge.lt_params[0] = params[0];
            adapter->bridge.lt_params[1] = params[1];
        }
        // Zero out remaining params
        memset(adapter->bridge.lt_params + 2, 0, 8);
    } else {
        adapter->bridge.rt_mode = mode;
        if (params && param_len >= 2) {
            adapter->bridge.rt_params[0] = params[0];
            adapter->bridge.rt_params[1] = params[1];
        }
        memset(adapter->bridge.rt_params + 2, 0, 8);
    }
    
    // Build output report
    uint8_t output[10];
    beitong_adapter_build_output(adapter, output, sizeof(output));
    
    // In real implementation, write to Beitong HID device
    // For now, just update state
    return true;
}

// ============================================================================
// Adapter: Set Rumble for Beitong Controller
// ============================================================================
void beitong_adapter_set_rumble(beitong_adapter_t *adapter, uint8_t grip_rumble) {
    if (!adapter || !adapter->initialized) return;
    
    adapter->bridge.motor_right = grip_rumble;
    adapter->bridge.motor_left = grip_rumble;
    
    // Build rumble report
    uint8_t output[6];
    output[0] = 0x02;          // Report ID
    output[1] = 0x76;          // Rumble command
    output[2] = 0x01;          // Subcommand
    output[3] = 0x04;          // Group A
    output[4] = grip_rumble;   // Strength 0-100%
    output[5] = 0x00;          // Padding
    
    // Real implementation would write to Beitong device
}

// ============================================================================
// Adapter Cleanup
// ============================================================================
void beitong_adapter_cleanup(beitong_adapter_t *adapter) {
    if (!adapter) return;
    memset(adapter, 0, sizeof(*adapter));
}