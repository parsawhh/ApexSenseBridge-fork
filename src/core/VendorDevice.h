#pragma once

#include "core/DeviceInfo.h"
#include "core/TriggerEffect.h"
#include "core/VendorEffects.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace asb::platform {
class PhysicalInputSource;
}

namespace asb::flydigi {
class Apex5Device;
}

namespace asb {

// The session-level contract a physical controller backend must satisfy.
//
// The bridge session owns a VendorDevice instead of a concrete vendor class so
// that every controller family (Flydigi Apex 4/5/6 and Beitong) shares one
// DualSense emulation path. VendorEffects covers the realtime feedback subset;
// the members below cover identity, lifecycle, baselines, telemetry and physical
// input acquisition.
struct VendorCapabilities {
    // The backend can render Sony adaptive-trigger effects.
    bool adaptiveTriggers = true;
    // The backend can render grip rumble.
    bool rumble = true;
    // Onboard RGB control is available (Apex 5 style lightbar sync).
    bool lightbar = false;
    // Onboard XInput profiles and input-transport control are available.
    bool profiles = false;
    // Vendor writes must be paced and coalesced instead of written inline.
    bool pacedWrites = false;
    // Realtime motor-block haptics are available (Apex 6 style).
    bool haptics = false;
    // Host-side gyroscope retuning is meaningful for this device (Apex 4 style).
    bool gyroTuning = false;
    // Physical input is read from the vendor HID interface.
    bool vendorInput = true;
};

// Vendor-neutral write telemetry. Mirrors the per-backend counters the session
// already reports so diagnostics keep the same shape for every controller.
struct VendorWriteStats {
    std::uint64_t writes = 0;
    std::uint64_t coalesced = 0;
    std::uint64_t slowWrites = 0;
    std::uint64_t maximumQueueUs = 0;
    std::uint64_t maximumWriteUs = 0;
};

class VendorDevice : public VendorEffects {
public:
    ~VendorDevice() override = default;

    // Stable identifier for diagnostics: "flydigi" or "beitong".
    [[nodiscard]] virtual std::string_view backendName() const noexcept = 0;
    [[nodiscard]] virtual VendorCapabilities capabilities() const noexcept = 0;
    [[nodiscard]] virtual const HidDeviceInfo& info() const noexcept = 0;

    // Identity. Backends without an identity handshake report false and zeroes.
    [[nodiscard]] virtual bool hasIdentity() const noexcept = 0;
    [[nodiscard]] virtual std::string describeIdentity() const = 0;
    [[nodiscard]] virtual std::string connectionName() const = 0;
    [[nodiscard]] virtual std::uint8_t batteryPercent() const noexcept { return 0; }
    [[nodiscard]] virtual std::uint8_t chargeState() const noexcept { return 0; }
    [[nodiscard]] virtual unsigned connectionTypeRaw() const noexcept { return 0; }

    // Neutral output baselines. A guard restores these on session exit, so they
    // must be safe to call repeatedly.
    virtual bool clearAll(std::string& error) = 0;
    virtual bool stopRumble(std::string& error) = 0;
    virtual bool requestBatteryRefresh(std::string& error) {
        (void)error;
        return true;
    }

    // Realtime feedback writer lifecycle. Backends that write inline may keep
    // the default no-op behaviour.
    virtual bool startAsyncWrites(std::string& error) {
        (void)error;
        return true;
    }
    virtual void stopAsyncWrites() noexcept {}
    [[nodiscard]] virtual bool pacedWrites() const noexcept { return false; }
    [[nodiscard]] virtual std::uint64_t asyncWriteRetries() const noexcept { return 0; }
    [[nodiscard]] virtual VendorWriteStats asyncWriteStats() const noexcept { return {}; }
    [[nodiscard]] virtual VendorWriteStats triggerWriteStats() const noexcept { return {}; }
    [[nodiscard]] virtual VendorWriteStats rumbleWriteStats() const noexcept { return {}; }
    virtual bool takeAsyncWriteError(std::string& error) {
        (void)error;
        return false;
    }

    // Reads the complete physical controller state. Each backend decides whether
    // that is the vendor HID stream, the mapped HID container, or XInput.
    virtual std::unique_ptr<platform::PhysicalInputSource> openInputSource(
        std::optional<unsigned int> xinputIndex,
        std::string& error) = 0;

    // Typed escape hatch for the Apex-only session steps (onboard profiles,
    // input-transport control, RGB config, Apex 6 haptic blocks). Always
    // nullptr for backends that do not implement the Flydigi protocol.
    [[nodiscard]] virtual flydigi::Apex5Device* apex() noexcept { return nullptr; }
};

} // namespace asb
