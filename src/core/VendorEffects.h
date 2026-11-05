#pragma once

#include "core/TriggerEffect.h"

#include <cstdint>
#include <string>
#include <string_view>

namespace asb {

// The effect-routing subset of a physical controller.
//
// AdaptiveTriggerBridge and RumbleBridge drive hardware through this interface
// instead of a concrete vendor class, so Flydigi and Beitong hardware share one
// feedback path. Implementations must make these calls non-blocking because they
// run synchronously inside the virtual DualSense feedback callback.
class VendorEffects {
public:
    virtual ~VendorEffects() = default;

    // Queues the newest value per side; superseded values are coalesced rather
    // than replayed later.
    virtual bool queueTriggerRaw(const ForceTriggerCommand& command,
                                 std::string& error) = 0;
    virtual bool queueRumble(std::uint8_t lowFrequencyMotor,
                             std::uint8_t highFrequencyMotor,
                             std::string& error) = 0;

    // Bow/galloping emulation retriggers the effect from host code. Hardware
    // with a native equivalent must not be emulated on top of it.
    [[nodiscard]] virtual bool supportsBowEmulation() const noexcept = 0;

    // Purely informational: used in diagnostics and error messages.
    [[nodiscard]] virtual std::string_view vendorName() const noexcept = 0;
};

} // namespace asb
