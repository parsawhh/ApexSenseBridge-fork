#pragma once

#include "core/VendorDevice.h"
#include "flydigi/Apex5Device.h"

#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace asb::flydigi {

// Adapts the Flydigi Apex transport to the vendor-neutral session contract.
//
// This class deliberately owns the concrete device and forwards every call
// unchanged: the Flydigi behaviour stays exactly what Apex5Device already
// implements, while the bridge session is written against VendorDevice.
class FlydigiVendor final : public VendorDevice {
public:
    explicit FlydigiVendor(Apex5Device&& device) noexcept : device_(std::move(device)) {}

    FlydigiVendor(const FlydigiVendor&) = delete;
    FlydigiVendor& operator=(const FlydigiVendor&) = delete;
    ~FlydigiVendor() override;

    [[nodiscard]] std::string_view backendName() const noexcept override;
    [[nodiscard]] VendorCapabilities capabilities() const noexcept override;
    [[nodiscard]] const HidDeviceInfo& info() const noexcept override;

    [[nodiscard]] bool hasIdentity() const noexcept override;
    [[nodiscard]] std::string describeIdentity() const override;
    [[nodiscard]] std::string connectionName() const override;
    [[nodiscard]] std::uint8_t batteryPercent() const noexcept override;
    [[nodiscard]] std::uint8_t chargeState() const noexcept override;
    [[nodiscard]] unsigned connectionTypeRaw() const noexcept override;

    bool clearAll(std::string& error) override;
    bool stopRumble(std::string& error) override;
    bool requestBatteryRefresh(std::string& error) override;

    bool startAsyncWrites(std::string& error) override;
    void stopAsyncWrites() noexcept override;
    [[nodiscard]] bool pacedWrites() const noexcept override;
    [[nodiscard]] std::uint64_t asyncWriteRetries() const noexcept override;
    [[nodiscard]] VendorWriteStats asyncWriteStats() const noexcept override;
    [[nodiscard]] VendorWriteStats triggerWriteStats() const noexcept override;
    [[nodiscard]] VendorWriteStats rumbleWriteStats() const noexcept override;
    bool takeAsyncWriteError(std::string& error) override;

    std::unique_ptr<platform::PhysicalInputSource> openInputSource(
        std::optional<unsigned int> xinputIndex,
        std::string& error) override;

    [[nodiscard]] flydigi::Apex5Device* apex() noexcept override;

    // Realtime effects.
    bool queueTriggerRaw(const ForceTriggerCommand& command, std::string& error) override;
    bool queueRumble(std::uint8_t lowFrequencyMotor,
                     std::uint8_t highFrequencyMotor,
                     std::string& error) override;
    [[nodiscard]] bool supportsBowEmulation() const noexcept override;
    [[nodiscard]] std::string_view vendorName() const noexcept override;

    // The concrete device, for call sites that genuinely need Flydigi types.
    [[nodiscard]] Apex5Device& device() noexcept { return device_; }

private:
    Apex5Device device_;
};

} // namespace asb::flydigi
