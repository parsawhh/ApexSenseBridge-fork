#pragma once

#include "core/TriggerEffect.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace asb::beitong {

// A complete 9-byte trigger-effect output report, ready for the HID transport:
//   02 56 <side> <mode> <p0> <p1> <p2> <p3> <p4>
using TriggerFrame = std::array<std::uint8_t, 9>;

// Translates the vendor-neutral FORCEADAPT block into the Beitong trigger
// protocol and stages writes per side.
//
// The panel accepts four behaviours:
//   0 off, 1 race/continuous resistance, 2 shot breakpoint, 3 gun vibration.
// Every mode parameter is an integer percentage 0..100, never a 0..255 byte.
//
// Staging keeps the last frame handed to the transport for each side and drops
// byte-identical repeats.
//
// There is deliberately no time-based dwell: `encode` already quantizes the wall
// to the panel's ~11 discrete positions, so a game streaming slightly varying
// effect values collapses onto identical frames by itself. A dwell would only
// add a window in which the newest frame could be stranded, because writes are
// synchronous and nothing guarantees a further update arrives to flush it.
class BeitongTriggerTranslator {
public:
    // Pure FORCEADAPT translation. Exposed for tests and diagnostics.
    [[nodiscard]] static TriggerFrame encode(const ForceTriggerCommand& command) noexcept;

    // Appends the frame that must be written now to `out` and returns 1, or
    // returns 0 when the encoded frame is identical to the one already written
    // for that side. At most one frame is produced per call.
    std::size_t update(const ForceTriggerCommand& command,
                       std::chrono::steady_clock::time_point now,
                       std::array<TriggerFrame, 2>& out);

    // Forgets all staged state, e.g. after clearAll writes explicit OFF frames.
    void reset() noexcept;

private:
    struct SideState {
        std::optional<TriggerFrame> lastWritten;
        std::chrono::steady_clock::time_point lastWriteAt{};
    };

    std::array<SideState, 2> sides_{};
};

} // namespace asb::beitong
