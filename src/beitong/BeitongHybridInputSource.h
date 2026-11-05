#pragma once

#include "core/DeviceInfo.h"
#include "dualsense/DualSenseInput.h"
#include "platform/HidTransport.h"
#include "platform/PhysicalInputSource.h"
#include "platform/XInputGamepad.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace asb::beitong {

// Beitong physical input.
//
// Two sources are fused because the controller splits the job between them:
//
//   * the HID gamepad collection (usage 0001/0005) streams a 14-byte report at
//     roughly 2.2 kHz, which carries the four 16-bit stick axes and the button
//     bitmask;
//   * the same report cannot express the analog triggers: bytes 8/9 encode a
//     single differential axis, value = 128 + (LT - RT) / 2 (measured on
//     hardware), so pulling both triggers fully is indistinguishable from
//     pulling neither. XInput is the only source that reports LT and RT
//     independently.
//
// Sticks therefore come from the fast HID stream and triggers from XInput.
// Buttons and the D-pad stay on XInput as well, because byte 11 of the HID
// report is only partially decoded.
class BeitongHybridInputSource final : public platform::PhysicalInputSource {
public:
    static std::unique_ptr<BeitongHybridInputSource> open(
        const HidDeviceInfo& hidInterface,
        std::optional<unsigned int> xinputIndex,
        std::string& error);

    ~BeitongHybridInputSource() override;

    BeitongHybridInputSource(const BeitongHybridInputSource&) = delete;
    BeitongHybridInputSource& operator=(const BeitongHybridInputSource&) = delete;

    platform::PhysicalInputStatus waitForState(
        dualsense::DualSenseInputState& state,
        std::chrono::milliseconds timeout,
        std::string& error) override;

    [[nodiscard]] std::string_view backendName() const noexcept override {
        return "beitong-hid+xinput-triggers";
    }
    [[nodiscard]] bool eventDriven() const noexcept override { return false; }
    [[nodiscard]] platform::PhysicalInputSourceStats stats() const noexcept override {
        return stats_;
    }
    void setBatteryState(std::uint8_t batteryPercent,
                         std::uint8_t chargeState) noexcept override {
        batteryPercent_ = batteryPercent;
        chargeState_ = chargeState;
    }

    // Byte offsets inside the 14-byte gamepad report, measured by correlating
    // the stream against XInput on hardware.
    static constexpr std::size_t kHidReportLength = 14;
    static constexpr std::size_t kLeftStickOffset = 0;
    static constexpr std::size_t kLeftStickYOffset = 2;
    static constexpr std::size_t kRightStickOffset = 4;
    static constexpr std::size_t kRightStickYOffset = 6;

private:
    BeitongHybridInputSource(platform::HidTransport* hid,
                             std::unique_ptr<platform::XInputGamepad> xinput) noexcept;

    // Returns true when a complete stick sample was parsed.
    bool drainHid(std::uint8_t& lx, std::uint8_t& ly,
                  std::uint8_t& rx, std::uint8_t& ry);

    platform::HidTransport* hid_ = nullptr;
    std::unique_ptr<platform::XInputGamepad> xinput_{};
    platform::PhysicalInputSourceStats stats_{};

    std::uint8_t stickLx_ = 0x80;
    std::uint8_t stickLy_ = 0x80;
    std::uint8_t stickRx_ = 0x80;
    std::uint8_t stickRy_ = 0x80;
    std::chrono::steady_clock::time_point stickObservedAt_{};

    std::uint8_t batteryPercent_ = 100;
    std::uint8_t chargeState_ = 0;
};

// Converts one unsigned 16-bit report axis (0..65535, centred at 0x8000) into
// the DualSense 0..255 convention. Both stick axes use the same polarity: the
// report value grows towards right/down exactly like the DualSense byte does.
[[nodiscard]] std::uint8_t beitongAxisToDualSense(std::uint16_t value) noexcept;

} // namespace asb::beitong
