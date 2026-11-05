#include "beitong/BeitongInputSource.h"

#include <array>
#include <cstddef>

namespace asb::beitong {
namespace {

constexpr std::size_t kDualSenseInputReportLength = 64;
constexpr std::uint8_t kDualSenseInputReportId = 0x01;

} // namespace

void decodeDualSenseInputReport(const std::uint8_t* report,
                                std::size_t length,
                                dualsense::DualSenseInputState& state) {
    if (report == nullptr || length < kDualSenseInputReportLength) return;
    if (report[0] != kDualSenseInputReportId) return;

    state.lx = report[1];
    state.ly = report[2];
    state.rx = report[3];
    state.ry = report[4];
    state.l2 = report[5];
    state.r2 = report[6];
    state.dpad = static_cast<std::uint8_t>(report[8] & 0x0F);

    std::uint16_t buttons = 0;
    const auto wire = [&report](std::size_t index, std::uint8_t mask) {
        return (report[index] & mask) != 0;
    };
    if (wire(8, 0x10)) buttons |= dualsense::button::kSquare;
    if (wire(8, 0x20)) buttons |= dualsense::button::kCross;
    if (wire(8, 0x40)) buttons |= dualsense::button::kCircle;
    if (wire(8, 0x80)) buttons |= dualsense::button::kTriangle;
    if (wire(9, 0x01)) buttons |= dualsense::button::kL1;
    if (wire(9, 0x02)) buttons |= dualsense::button::kR1;
    if (wire(9, 0x04)) buttons |= dualsense::button::kL2;
    if (wire(9, 0x08)) buttons |= dualsense::button::kR2;
    if (wire(9, 0x10)) buttons |= dualsense::button::kCreate;
    if (wire(9, 0x20)) buttons |= dualsense::button::kOptions;
    if (wire(9, 0x40)) buttons |= dualsense::button::kL3;
    if (wire(9, 0x80)) buttons |= dualsense::button::kR3;
    if (wire(10, 0x01)) buttons |= dualsense::button::kPs;
    state.buttons = buttons;

    // The pad reports 0x0A as "battery full"; treat anything above 0x10 as
    // charging-capable and keep the raw estimate otherwise.
    const std::uint8_t rawBattery = report[53] & 0x0F;
    state.batteryPercent = dualsense::toBatteryPercent(rawBattery);
    state.chargeState = dualsense::chargeStatus::kDischarging;
}

BeitongInputSource::BeitongInputSource(BeitongDevice& device) : device_(device) {}

platform::PhysicalInputStatus BeitongInputSource::waitForState(dualsense::DualSenseInputState& state,
                                                     std::chrono::milliseconds timeout,
                                                     std::string& error) {
    std::array<std::uint8_t, kDualSenseInputReportLength> raw{};
    std::size_t bytesRead = 0;
    error.clear();

    const auto status = device_.readInputReport(raw, timeout, bytesRead, error);
    if (status == platform::HidReadStatus::Timeout) {
        ++stats_.timeouts;
        return platform::PhysicalInputStatus::Timeout;
    }
    if (status == platform::HidReadStatus::Error) {
        ++stats_.parseFailures;
        if (error.empty()) error = "Reading from the Beitong controller failed.";
        return platform::PhysicalInputStatus::Disconnected;
    }

    ++stats_.reports;
    std::array<std::uint8_t, kDualSenseInputReportLength> translated{};
    const int written = device_.translateInput(raw.data(), bytesRead, translated);
    if (written <= 0) {
        ++stats_.parseFailures;
        error = "The Beitong controller sent an unrecognised input report.";
        return platform::PhysicalInputStatus::Timeout;
    }

    decodeDualSenseInputReport(translated.data(), translated.size(), state);
    ++stats_.mappedReports;
    return platform::PhysicalInputStatus::State;
}

std::string_view BeitongInputSource::backendName() const noexcept {
    return device_.isBluetooth() ? "beitong-hid-bt" : "beitong-hid-usb";
}

bool BeitongInputSource::eventDriven() const noexcept {
    return true;
}

platform::PhysicalInputSourceStats BeitongInputSource::stats() const noexcept {
    return stats_;
}

} // namespace asb::beitong
