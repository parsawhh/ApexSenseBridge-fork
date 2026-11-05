#pragma once

#include "beitong/BeitongTriggerTranslator.h"
#include "core/VendorDevice.h"
#include "platform/HidTransport.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace asb::beitong {

struct VendorTransportDeleter {
    void operator()(platform::HidTransport* transport) const noexcept;
};
using VendorTransportPtr = std::unique_ptr<platform::HidTransport, VendorTransportDeleter>;

// Vendor backend for Beitong (Kunpeng) controllers.
//
// Feedback goes out as 9-byte vendor output reports on the private HID
// collection (usage_page 0x00FF, usage 0x0003); physical input comes from the
// pad's XInput endpoint instead. There is no handshake, checksum or sequence
// byte, and the backend never emits the 0x76 rumble command: those
// sub-commands drive hardware the owner reports as damage-prone, so grip
// rumble is reported as unsupported and queueRumble() writes nothing.
class BeitongVendor final : public VendorDevice {
public:
    static constexpr std::uint16_t kVendorId = 0x20BC;
    static constexpr std::uint16_t kVendorIdAlter = 0x20DD;
    static constexpr std::uint16_t kProductWired = 0x5173;
    static constexpr std::uint16_t kProductDongle = 0x5174;
    static constexpr std::uint16_t kVendorUsagePage = 0x00FF;
    static constexpr std::uint16_t kVendorUsage = 0x0003;

    [[nodiscard]] static constexpr bool isSupportedVendor(std::uint16_t vendorId) noexcept {
        return vendorId == kVendorId || vendorId == kVendorIdAlter;
    }
    [[nodiscard]] static constexpr bool isSupportedProduct(std::uint16_t productId) noexcept {
        return productId == kProductWired || productId == kProductDongle;
    }

    BeitongVendor(const BeitongVendor&) = delete;
    BeitongVendor& operator=(const BeitongVendor&) = delete;
    ~BeitongVendor() override;

    // Enumerates supported vendor HID collections, wired interfaces first.
    [[nodiscard]] static std::vector<HidDeviceInfo> findCandidates(std::string& error);
    [[nodiscard]] static std::unique_ptr<BeitongVendor> open(
        const HidDeviceInfo& info, std::string& error);

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

    [[nodiscard]] VendorWriteStats triggerWriteStats() const noexcept override;
    [[nodiscard]] VendorWriteStats rumbleWriteStats() const noexcept override;

    std::unique_ptr<platform::PhysicalInputSource> openInputSource(
        std::optional<unsigned int> xinputIndex,
        std::string& error) override;

    [[nodiscard]] flydigi::Apex5Device* apex() noexcept override { return nullptr; }

    bool queueTriggerRaw(const ForceTriggerCommand& command, std::string& error) override;
    bool queueRumble(std::uint8_t lowFrequencyMotor,
                     std::uint8_t highFrequencyMotor,
                     std::string& error) override;
    [[nodiscard]] bool supportsBowEmulation() const noexcept override;
    [[nodiscard]] std::string_view vendorName() const noexcept override;

private:
    BeitongVendor(VendorTransportPtr transport, HidDeviceInfo info);

    bool isDongle() const noexcept;
    bool writeFrame(const TriggerFrame& frame, std::string& error);

    VendorTransportPtr transport_;
    HidDeviceInfo info_;
    BeitongTriggerTranslator translator_;
    // The virtual DualSense feedback callback and session shutdown may race.
    std::mutex writeMutex_;
    std::atomic<std::uint64_t> writes_{0};
    std::atomic<std::uint64_t> rumbleWrites_{0};
};

} // namespace asb::beitong
