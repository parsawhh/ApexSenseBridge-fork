#pragma once

#include "core/DeviceInfo.h"
#include "core/VendorEffects.h"
#include "dualsense/DualSenseFeedback.h"
#include "platform/HidTransport.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "beitong/beitong_protocol.h"

namespace asb::beitong {

struct TransportDeleter {
    void operator()(platform::HidTransport* transport) const noexcept;
};
using TransportPtr = std::unique_ptr<platform::HidTransport, TransportDeleter>;

// Physical Beitong (Kunpeng) controller: USB or Bluetooth HID.
//
// The controller is a plain HID device, so there is no vendor handshake: the
// reference implementation opens it by VID/PID, writes trigger and rumble
// reports, and reads input reports. All wire formatting and the DualSense
// translation live in beitong_bridge.c, which is a port of the reference
// ds2bt_engine/bidirectional_bridge code.
//
// Threading: the feedback callback and the input loop may run on different
// threads, and the C translation state is not re-entrant, so every entry point
// that touches it holds stateMutex_.
class BeitongDevice : public VendorEffects {
public:
    BeitongDevice() = default;
    explicit BeitongDevice(TransportPtr transport);
    ~BeitongDevice();

    BeitongDevice(const BeitongDevice&) = delete;
    BeitongDevice& operator=(const BeitongDevice&) = delete;
    BeitongDevice(BeitongDevice&&) noexcept;
    BeitongDevice& operator=(BeitongDevice&&) noexcept;

    // Enumerates every supported Beitong pad, most preferred first.
    [[nodiscard]] static std::vector<HidDeviceInfo> findCandidates(std::string& error);
    [[nodiscard]] static std::unique_ptr<BeitongDevice> open(const HidDeviceInfo& info,
                                                             std::string& error);

    [[nodiscard]] bool isOpen() const noexcept;
    [[nodiscard]] const HidDeviceInfo& info() const;
    [[nodiscard]] bool isBluetooth() const noexcept;

    platform::HidReadStatus readInputReport(std::span<std::uint8_t> report,
                                            std::chrono::milliseconds timeout,
                                            std::size_t& bytesRead,
                                            std::string& error);

    // Translates one raw Beitong input report into a 64-byte DualSense report
    // 0x01. Returns the byte count, or 0 when the frame is not a full report.
    int translateInput(const std::uint8_t* data,
                       std::size_t len,
                       std::span<std::uint8_t> out);

    // Faithful DualSense-feedback path. The virtual DualSense hands us the raw
    // effect blocks, which map 1:1 onto the reference translation (rather than
    // the lossy Flydigi TriggerMode round trip).
    bool submitFeedback(const dualsense::DualSenseFeedback& feedback, std::string& error);

    // VendorEffects
    bool queueTriggerRaw(const ForceTriggerCommand& command, std::string& error) override;
    bool queueRumble(std::uint8_t lowFrequencyMotor,
                     std::uint8_t highFrequencyMotor,
                     std::string& error) override;
    [[nodiscard]] bool supportsBowEmulation() const noexcept override;
    [[nodiscard]] std::string_view vendorName() const noexcept override;

    // Tuning knobs mirrored onto the shared translation state.
    void setGains(float forceGain, float vibeGain, float rumbleGain, std::uint8_t minForceFloor);
    void setCurve(int curveType);
    void setDeadzones(std::uint8_t ltStart, std::uint8_t ltEnd,
                      std::uint8_t rtStart, std::uint8_t rtEnd);

    [[nodiscard]] std::uint64_t writes() const noexcept;
    [[nodiscard]] std::uint64_t feedbackReports() const noexcept;
    [[nodiscard]] std::uint64_t inputReports() const noexcept;

private:
    bool writeTriggerPacket(std::uint8_t side,
                            std::uint8_t mode,
                            const std::uint8_t* params,
                            std::size_t paramCount,
                            std::string& error);

    TransportPtr transport_{};
    beitong_bridge_state_t bridge_{};
    mutable std::mutex stateMutex_{};
    std::uint64_t writes_ = 0;
    std::uint64_t feedbackReports_ = 0;
    std::uint64_t inputReports_ = 0;
};

// True when vendorId/productId identify hardware this backend can drive.
[[nodiscard]] bool isSupportedDevice(std::uint16_t vendorId, std::uint16_t productId) noexcept;

} // namespace asb::beitong
