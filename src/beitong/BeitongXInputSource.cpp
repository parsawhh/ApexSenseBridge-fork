#include "beitong/BeitongXInputSource.h"

#include "platform/XInputGamepad.h"

#include <thread>
#include <utility>

namespace asb::beitong {

BeitongXInputSource::BeitongXInputSource(
    std::unique_ptr<platform::XInputGamepad> gamepad)
    : gamepad_(std::move(gamepad)) {}

BeitongXInputSource::~BeitongXInputSource() = default;

std::unique_ptr<BeitongXInputSource> BeitongXInputSource::open(
    std::uint16_t vendorId,
    std::uint16_t productId,
    std::optional<unsigned int> xinputIndex,
    std::string& error) {
    auto gamepad = platform::openXInputGamepadForDevice(
        vendorId, productId, xinputIndex, error);
    if (!gamepad) {
        if (error.empty()) {
            error = "No XInput endpoint belongs to the selected Beitong controller.";
        }
        return {};
    }
    return std::unique_ptr<BeitongXInputSource>(
        new BeitongXInputSource(std::move(gamepad)));
}

platform::PhysicalInputStatus BeitongXInputSource::waitForState(
    dualsense::DualSenseInputState& state,
    std::chrono::milliseconds timeout,
    std::string& error) {
    error.clear();
    // XInput has no blocking read, so the caller's timeout is the polling
    // interval. This keeps the bridge loop paced exactly like the existing
    // non-event-driven Windows input sources.
    if (timeout.count() > 0) {
        std::this_thread::sleep_for(timeout);
    }
    if (!gamepad_ || !gamepad_->poll(state, error)) {
        // A missing XInput state is reported as a timeout, not a hard failure:
        // the controller may be re-enumerating or asleep.
        ++stats_.timeouts;
        error.clear();
        return platform::PhysicalInputStatus::Timeout;
    }
    ++stats_.reports;
    return platform::PhysicalInputStatus::State;
}

std::string_view BeitongXInputSource::backendName() const noexcept {
    return "beitong-xinput";
}

bool BeitongXInputSource::eventDriven() const noexcept {
    return false;
}

platform::PhysicalInputSourceStats BeitongXInputSource::stats() const noexcept {
    return stats_;
}

} // namespace asb::beitong
