#include "beitong/BeitongHybridInputSource.h"

#include <algorithm>
#include <array>

namespace asb::beitong {
namespace {

// A stick sample older than this no longer overrides XInput: the HID stream is
// event driven, so silence means the sticks are still, not that the source died.
constexpr auto kStickFreshness = std::chrono::milliseconds(80);

// Bound the per-call drain so a 2 kHz stream cannot starve the session loop.
constexpr int kMaxReportsPerCall = 64;

std::uint16_t readAxis(const std::uint8_t* report, std::size_t offset) noexcept {
    return static_cast<std::uint16_t>(report[offset]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(report[offset + 1]) << 8);
}

} // namespace

std::uint8_t beitongAxisToDualSense(std::uint16_t value) noexcept {
    // 0x8000 (32768) is the measured centre and maps to 0x80.
    const std::uint32_t scaled = (static_cast<std::uint32_t>(value) * 255U + 32767U) / 65535U;
    return static_cast<std::uint8_t>((std::min)(scaled, 255U));
}

BeitongHybridInputSource::BeitongHybridInputSource(
    platform::HidTransport* hid, std::unique_ptr<platform::XInputGamepad> xinput) noexcept
    : hid_(hid), xinput_(std::move(xinput)) {}

BeitongHybridInputSource::~BeitongHybridInputSource() {
    if (hid_) {
        platform::destroyHidTransport(hid_);
        hid_ = nullptr;
    }
}

std::unique_ptr<BeitongHybridInputSource> BeitongHybridInputSource::open(
    const HidDeviceInfo& hidInterface,
    std::optional<unsigned int> xinputIndex,
    std::string& error) {
    std::unique_ptr<platform::XInputGamepad> gamepad =
        platform::openXInputGamepadForDevice(
            hidInterface.vendorId, hidInterface.productId, xinputIndex, error);
    if (!gamepad) {
        return {};
    }

    platform::HidTransport* hid = platform::createHidTransport(hidInterface, error);
    if (!hid || !hid->isOpen()) {
        if (hid) platform::destroyHidTransport(hid);
        if (error.empty()) {
            error = "The Beitong stick HID interface could not be opened.";
        }
        return {};
    }

    return std::unique_ptr<BeitongHybridInputSource>(
        new BeitongHybridInputSource(hid, std::move(gamepad)));
}

bool BeitongHybridInputSource::drainHid(std::uint8_t& lx, std::uint8_t& ly,
                                        std::uint8_t& rx, std::uint8_t& ry) {
    if (!hid_) return false;

    bool parsed = false;
    for (int attempt = 0; attempt < kMaxReportsPerCall; ++attempt) {
        std::array<std::uint8_t, 64> buffer{};
        std::size_t bytesRead = 0;
        std::string ignored;
        const auto status = hid_->readInputReport(
            buffer, std::chrono::milliseconds(0), bytesRead, ignored);
        if (status != platform::HidReadStatus::Data) {
            break;
        }
        if (bytesRead < kHidReportLength) {
            continue;
        }
        lx = beitongAxisToDualSense(readAxis(buffer.data(), kLeftStickOffset));
        ly = beitongAxisToDualSense(readAxis(buffer.data(), kLeftStickYOffset));
        rx = beitongAxisToDualSense(readAxis(buffer.data(), kRightStickOffset));
        ry = beitongAxisToDualSense(readAxis(buffer.data(), kRightStickYOffset));
        parsed = true;
    }
    return parsed;
}

platform::PhysicalInputStatus BeitongHybridInputSource::waitForState(
    dualsense::DualSenseInputState& state,
    std::chrono::milliseconds timeout,
    std::string& error) {
    dualsense::DualSenseInputState polled{};
    std::string pollError;
    if (!xinput_ || !xinput_->poll(polled, pollError)) {
        error = pollError.empty() ? "The Beitong XInput source stopped responding."
                                  : pollError;
        return platform::PhysicalInputStatus::Disconnected;
    }

    std::uint8_t lx = 0;
    std::uint8_t ly = 0;
    std::uint8_t rx = 0;
    std::uint8_t ry = 0;
    if (drainHid(lx, ly, rx, ry)) {
        stickLx_ = lx;
        stickLy_ = ly;
        stickRx_ = rx;
        stickRy_ = ry;
        stickObservedAt_ = std::chrono::steady_clock::now();
    }

    state = polled;
    if (stickObservedAt_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::steady_clock::now() - stickObservedAt_ <= kStickFreshness) {
        state.lx = stickLx_;
        state.ly = stickLy_;
        state.rx = stickRx_;
        state.ry = stickRy_;
    }
    state.batteryPercent = batteryPercent_;
    state.chargeState = chargeState_;

    ++stats_.reports;
    error.clear();
    (void)timeout;
    return platform::PhysicalInputStatus::State;
}

} // namespace asb::beitong
