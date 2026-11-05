#include "flydigi/FlydigiVendor.h"

#include "platform/PhysicalInputSource.h"

#include <utility>

namespace asb::flydigi {

namespace {

VendorWriteStats toVendorStats(const AsyncWriteStats& stats) noexcept {
    VendorWriteStats converted{};
    converted.writes = stats.writes;
    converted.coalesced = stats.coalesced;
    converted.slowWrites = stats.slowWrites;
    converted.maximumQueueUs = stats.maximumQueueUs;
    converted.maximumWriteUs = stats.maximumWriteUs;
    return converted;
}

VendorWriteStats toVendorStats(const FeedbackWriteStats& stats) noexcept {
    VendorWriteStats converted{};
    converted.writes = stats.writes;
    converted.slowWrites = stats.slowWrites;
    converted.maximumWriteUs = stats.maximumWriteUs;
    return converted;
}

} // namespace

FlydigiVendor::~FlydigiVendor() = default;

std::string_view FlydigiVendor::backendName() const noexcept { return "flydigi"; }

VendorCapabilities FlydigiVendor::capabilities() const noexcept {
    VendorCapabilities caps{};
    const auto& identity = device_.identity();
    // Apex 4 (V1) and Apex 5 (V2) render adaptive triggers; Apex 6 (V3) does not.
    caps.adaptiveTriggers =
        identity.has_value() && identity->supportsAdaptiveTriggers();
    caps.rumble = true;
    // Onboard XInput profiles, RGB config and input-transport control are the
    // Apex 5 surface only. The Apex 4 degraded-interface handling stays in the
    // session and is gated on the same capability.
    caps.profiles = identity.has_value() && identity->isApex5();
    caps.lightbar = caps.profiles;
    caps.pacedWrites = device_.asyncWritePolicy().has_value();
    caps.haptics = identity.has_value() && identity->isApex6();
    caps.gyroTuning = identity.has_value() && identity->isApex4();
    caps.vendorInput = true;
    return caps;
}

const HidDeviceInfo& FlydigiVendor::info() const noexcept { return device_.info(); }

bool FlydigiVendor::hasIdentity() const noexcept { return device_.identity().has_value(); }

std::string FlydigiVendor::describeIdentity() const {
    return device_.identity() ? device_.identity()->describe() : std::string{};
}

std::string FlydigiVendor::connectionName() const {
    return device_.identity()
               ? std::string(device_.identity()->connectionName())
               : std::string{};
}

std::uint8_t FlydigiVendor::batteryPercent() const noexcept {
    return device_.identity() ? device_.identity()->batteryPercent() : 0;
}

std::uint8_t FlydigiVendor::chargeState() const noexcept {
    return device_.identity() ? device_.identity()->chargeState() : 0;
}

unsigned FlydigiVendor::connectionTypeRaw() const noexcept {
    return device_.identity()
               ? static_cast<unsigned>(device_.identity()->connectionTypeRaw())
               : 0U;
}

bool FlydigiVendor::clearAll(std::string& error) { return device_.clearAll(error); }

bool FlydigiVendor::stopRumble(std::string& error) { return device_.stopRumble(error); }

bool FlydigiVendor::requestBatteryRefresh(std::string& error) {
    return device_.requestBatteryRefresh(error);
}

bool FlydigiVendor::startAsyncWrites(std::string& error) {
    return device_.startAsyncWrites(error);
}

void FlydigiVendor::stopAsyncWrites() noexcept { device_.stopAsyncWrites(); }

bool FlydigiVendor::pacedWrites() const noexcept {
    return device_.asyncWritePolicy().has_value();
}

std::uint64_t FlydigiVendor::asyncWriteRetries() const noexcept {
    return device_.asyncWriteRetries();
}

VendorWriteStats FlydigiVendor::asyncWriteStats() const noexcept {
    return toVendorStats(device_.asyncWriteStats());
}

VendorWriteStats FlydigiVendor::triggerWriteStats() const noexcept {
    return toVendorStats(device_.triggerWriteStats());
}

VendorWriteStats FlydigiVendor::rumbleWriteStats() const noexcept {
    return toVendorStats(device_.rumbleWriteStats());
}

bool FlydigiVendor::takeAsyncWriteError(std::string& error) {
    return device_.takeAsyncWriteError(error);
}

std::unique_ptr<platform::PhysicalInputSource> FlydigiVendor::openInputSource(
    std::optional<unsigned int> xinputIndex, std::string& error) {
    return platform::openPhysicalInputSource(device_.info(), xinputIndex, error);
}

Apex5Device* FlydigiVendor::apex() noexcept { return &device_; }

bool FlydigiVendor::queueTriggerRaw(const ForceTriggerCommand& command,
                                    std::string& error) {
    return device_.queueTriggerRaw(command, error);
}

bool FlydigiVendor::queueRumble(std::uint8_t lowFrequencyMotor,
                               std::uint8_t highFrequencyMotor,
                               std::string& error) {
    return device_.queueRumble(lowFrequencyMotor, highFrequencyMotor, error);
}

bool FlydigiVendor::supportsBowEmulation() const noexcept {
    return device_.supportsBowEmulation();
}

std::string_view FlydigiVendor::vendorName() const noexcept { return "flydigi"; }

} // namespace asb::flydigi
