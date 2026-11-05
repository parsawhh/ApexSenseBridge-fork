#pragma once

#include "core/VendorDevice.h"
#include "platform/PhysicalInputSource.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace asb::platform {
class XInputGamepad;
}

namespace asb::beitong {

// Physical input for a Beitong pad, read from the XInput endpoint that the
// wired controller / 2.4 GHz dongle exposes. The vendor HID collection carries
// trigger and rumble commands only; sticks, buttons and triggers themselves
// arrive through XInput.
class BeitongXInputSource final : public platform::PhysicalInputSource {
public:
    ~BeitongXInputSource() override;

    static std::unique_ptr<BeitongXInputSource> open(
        std::uint16_t vendorId,
        std::uint16_t productId,
        std::optional<unsigned int> xinputIndex,
        std::string& error);

    platform::PhysicalInputStatus waitForState(
        dualsense::DualSenseInputState& state,
        std::chrono::milliseconds timeout,
        std::string& error) override;

    [[nodiscard]] std::string_view backendName() const noexcept override;
    [[nodiscard]] bool eventDriven() const noexcept override;
    [[nodiscard]] platform::PhysicalInputSourceStats stats() const noexcept override;
    // XInput reports LT and RT independently, so no combined-axis proxy is
    // needed and the independent stream is always considered ready.
    [[nodiscard]] bool requiresIndependentTriggers() const noexcept override { return false; }
    [[nodiscard]] bool independentTriggersReady() const noexcept override { return true; }

private:
    explicit BeitongXInputSource(std::unique_ptr<platform::XInputGamepad> gamepad);

    std::unique_ptr<platform::XInputGamepad> gamepad_;
    platform::PhysicalInputSourceStats stats_{};
};

} // namespace asb::beitong
