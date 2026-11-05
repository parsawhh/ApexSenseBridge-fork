#pragma once

#include "beitong/BeitongDevice.h"
#include "platform/PhysicalInputSource.h"

#include <cstdint>
#include <string>
#include <string_view>

namespace asb::beitong {

// Presents a Beitong pad as the bridge's physical input source. Input reports
// are translated through the same beitong_bridge.c state the feedback path
// uses, so both directions stay coherent.
class BeitongInputSource final : public platform::PhysicalInputSource {
public:
    explicit BeitongInputSource(BeitongDevice& device);

    platform::PhysicalInputStatus waitForState(dualsense::DualSenseInputState& state,
                                               std::chrono::milliseconds timeout,
                                               std::string& error) override;

    [[nodiscard]] std::string_view backendName() const noexcept override;
    [[nodiscard]] bool eventDriven() const noexcept override;
    [[nodiscard]] platform::PhysicalInputSourceStats stats() const noexcept override;

private:
    BeitongDevice& device_;
    platform::PhysicalInputSourceStats stats_{};
};

// Exposed for tests: maps a translated 64-byte DualSense input report 0x01 onto
// the bridge's device-independent state.
void decodeDualSenseInputReport(const std::uint8_t* report,
                                std::size_t length,
                                dualsense::DualSenseInputState& state);

} // namespace asb::beitong
