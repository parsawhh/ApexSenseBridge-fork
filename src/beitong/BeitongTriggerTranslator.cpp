#include "beitong/BeitongTriggerTranslator.h"

#include <algorithm>

namespace asb::beitong {
namespace {

// Beitong vendor trigger report (output report id 0x02, 9 bytes on the wire):
//   02 56 <side> <mode> <p0> <p1> <p2> <p3> <p4>
constexpr std::uint8_t kReportIdOutput = 0x02;
constexpr std::uint8_t kCommandTrigger = 0x56;

constexpr std::uint8_t kSideLeft = 0x00;
constexpr std::uint8_t kSideRight = 0x01;

constexpr std::uint8_t kModeOff = 0x00;
constexpr std::uint8_t kModeRace = 0x01;
constexpr std::uint8_t kModeShotBreak = 0x02;
constexpr std::uint8_t kModeGunVibration = 0x03;

// FORCEADAPT parameter layout (see flydigi/Apex5Protocol.cpp buildForceTrigger,
// buildForceTriggerRaw, and dualsense/AdaptiveTriggerTranslation.cpp, which
// produces the block the bridge hands to VendorEffects::queueTriggerRaw):
//
//   params[0]  start / lower zone boundary in the DualSense 0..192 stroke
//              space (strokeForZone(z) = z * 192 / 10). Native Sony effects
//              that are forwarded unmodified may carry a raw 0..255 position
//              here instead, so treat the field as "0..255-ish".
//   params[1]  Race: resistance/force (0..255). SniperBreak: span/length in the
//              same stroke units as params[0] (not an absolute position).
//              RecoilRattle/Vibration: secondary strength byte, usually 1.
//   params[2]  SniperBreak: break force. RecoilRattle/Vibration: amplitude.
//   params[3]  RecoilRattle/Vibration: vibration frequency byte (Apex 6 reads
//              it as Hz); zero means "unspecified". SniperBreak reserves it.
//   params[4]  Race/SniperBreak input-match flag; unused by the Beitong wire
//              protocol because the hardware has no match-input notion.
//
// The Beitong wall is physically stepped: roughly ten distinct positions over
// full travel. The Sony start stroke is therefore snapped back to the nearest
// zone and encoded as the hardware's own zone percentage (0, 11, 22 ... 100),
// rather than as an arbitrary analog percentage.

constexpr unsigned kSonyZoneCount = 10;
constexpr unsigned kSonyStrokeSpan = 192; // strokeForZone(z) = (z * 192 + 5) / 10.
constexpr unsigned kNativeForceCeiling = 64; // DualSense 1..8 * 8 in AdaptiveTriggerTranslation.cpp.
constexpr unsigned kNativeVibeCeiling = 80;  // DualSense 1..8 * 10 in AdaptiveTriggerTranslation.cpp.

std::uint8_t clampPercent(int value) noexcept {
    return static_cast<std::uint8_t>(std::clamp(value, 0, 100));
}

int scaleRange(int value, int inMin, int inMax, int outMin, int outMax) noexcept {
    if (value <= inMin) return outMin;
    if (value >= inMax) return outMax;
    return outMin + ((value - inMin) * (outMax - outMin) + (inMax - inMin) / 2) /
                        (inMax - inMin);
}

// Recovers the Sony zone index (0..9) from a strokeForZone(z) byte (0..173).
unsigned zoneFromStroke(unsigned stroke) noexcept {
    const unsigned zone = (stroke * kSonyZoneCount + kSonyStrokeSpan / 2U) / kSonyStrokeSpan;
    return (std::min)(zone, kSonyZoneCount - 1U);
}

// Maps a Sony zone (0..9) onto the 0..100 percentage domain used by 0x56 frames.
std::uint8_t zoneToPercent(unsigned zone) noexcept {
    const unsigned clamped = (std::min)(zone, kSonyZoneCount - 1U);
    return static_cast<std::uint8_t>(
        (clamped * 100U + (kSonyZoneCount - 1U) / 2U) / (kSonyZoneCount - 1U));
}

// Scales a FORCEADAPT Race force byte (8..64 from DualSense 0x21/0x22, or up to
// 255 from raw legacy effects) to Beitong's 15..100% range (matching
// beitong_bridge.c:143).
std::uint8_t scaleRaceForce(std::uint8_t raw) noexcept {
    if (raw == 0) return 0;
    if (raw <= kNativeForceCeiling) {
        return clampPercent(scaleRange(raw, 8, static_cast<int>(kNativeForceCeiling), 15, 100));
    }
    return clampPercent(scaleRange(raw, 1, 255, 15, 100));
}

// Scales a FORCEADAPT SniperBreak force byte (8..64 from DualSense 0x25/0x22)
// to Beitong's 25..100% range (matching beitong_bridge.c:151).
std::uint8_t scaleBreakForce(std::uint8_t raw) noexcept {
    if (raw == 0) return 0;
    if (raw <= kNativeForceCeiling) {
        return clampPercent(scaleRange(raw, 8, static_cast<int>(kNativeForceCeiling), 25, 100));
    }
    return clampPercent(scaleRange(raw, 1, 255, 25, 100));
}

// Scales a FORCEADAPT Vibration amplitude byte (10..80 from DualSense
// 0x23/0x26/0x27) to Beitong's 20..100% range (matching beitong_bridge.c:159).
std::uint8_t scaleVibeAmplitude(std::uint8_t raw) noexcept {
    if (raw == 0) return 0;
    if (raw <= kNativeVibeCeiling) {
        return clampPercent(scaleRange(raw, 10, static_cast<int>(kNativeVibeCeiling), 20, 100));
    }
    return clampPercent(scaleRange(raw, 1, 255, 20, 100));
}

std::uint8_t sideCode(TriggerSide side) noexcept {
    return side == TriggerSide::Left ? kSideLeft : kSideRight;
}

std::size_t sideIndex(TriggerSide side) noexcept {
    return side == TriggerSide::Left ? 0U : 1U;
}

} // namespace

TriggerFrame BeitongTriggerTranslator::encode(const ForceTriggerCommand& command) noexcept {
    TriggerFrame frame{};
    frame[0] = kReportIdOutput;
    frame[1] = kCommandTrigger;
    frame[2] = sideCode(command.side);

    switch (command.mode) {
    case TriggerMode::Race: {
        // Mode 1: wall start %, wall force % (15..100%).
        frame[3] = kModeRace;
        const unsigned startZone = zoneFromStroke(command.params[0]);
        frame[4] = zoneToPercent(startZone);
        frame[5] = scaleRaceForce(command.params[1]);
        break;
    }

    case TriggerMode::SniperBreak: {
        // Mode 2: wall start %, snap/click point %, wall force %, return force %, 1.
        // Matches beitong_bridge.c:147-156 and AdaptiveTriggerTranslation.cpp:44-53:
        // params[0] = start stroke, params[1] = span/length stroke, params[2] = force.
        frame[3] = kModeShotBreak;
        const unsigned startZone = zoneFromStroke(command.params[0]);
        const unsigned endStroke = static_cast<unsigned>(command.params[0]) +
                                   static_cast<unsigned>(command.params[1]);
        const unsigned endZone = (std::max)(
            startZone + 1U, (std::min)(zoneFromStroke(endStroke), kSonyZoneCount - 1U));
        const std::uint8_t startPct = zoneToPercent(startZone);
        const std::uint8_t endPct = clampPercent(
            scaleRange(static_cast<int>(endZone), 0, 9,
                       static_cast<int>(startPct) + 10, 100));
        const std::uint8_t forcePct = scaleBreakForce(command.params[2]);
        frame[4] = startPct;
        frame[5] = endPct;
        frame[6] = forcePct;
        frame[7] = static_cast<std::uint8_t>(forcePct / 2U);
        frame[8] = 0x01;
        break;
    }

    case TriggerMode::RecoilRattle:
    case TriggerMode::Vibration: {
        // Mode 3: start %, cadence index (5..100 mapped from 10..240 Hz), amplitude %.
        // Matches beitong_bridge.c:157-164 and AdaptiveTriggerTranslation.cpp:82-127.
        frame[3] = kModeGunVibration;
        const unsigned startZone = zoneFromStroke(command.params[0]);
        frame[4] = zoneToPercent(startZone);
        const std::uint8_t rawHz =
            command.params[3] != 0 ? command.params[3]
                                   : (command.params[1] > 1 ? command.params[1] : 100);
        frame[5] = static_cast<std::uint8_t>(
            std::clamp(scaleRange(rawHz, 10, 240, 5, 100), 5, 100));
        frame[6] = scaleVibeAmplitude(command.params[2]);
        break;
    }

    case TriggerMode::Normal:
    case TriggerMode::Lock:
    default:
        // Mode 0: off. Lock has no native equivalent and must fail safe.
        frame[3] = kModeOff;
        break;
    }

    return frame;
}

std::size_t BeitongTriggerTranslator::update(
    const ForceTriggerCommand& command,
    std::chrono::steady_clock::time_point now,
    std::array<TriggerFrame, 2>& out) {
    auto& side = sides_[sideIndex(command.side)];
    const TriggerFrame frame = encode(command);

    // Identical to what the panel already has: nothing to do. This is what keeps
    // a game that streams a constant effect from re-writing the same bytes.
    if (side.lastWritten && *side.lastWritten == frame) {
        return 0;
    }

    out[0] = frame;
    side.lastWritten = frame;
    side.lastWriteAt = now;
    return 1;
}

void BeitongTriggerTranslator::reset() noexcept {
    sides_ = {};
}

} // namespace asb::beitong
