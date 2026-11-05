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
constexpr unsigned kSonyStrokeSpan = 192; // strokeForZone(10) upper bound.

// Scales a 0..255-ish effect field to whole percent and clamps to 0..100.
std::uint8_t scalePercent(std::uint8_t value) noexcept {
    return static_cast<std::uint8_t>(
        (static_cast<unsigned>(value) * 100U + 127U) / 255U);
}

// Maps a DualSense stroke position back to the nearest Sony zone and then to
// the stepped percentage the Beitong actuator exposes: z * 100 / 9.
std::uint8_t zonePercent(unsigned stroke) noexcept {
    unsigned zone = (stroke * kSonyZoneCount + kSonyStrokeSpan / 2U) / kSonyStrokeSpan;
    if (zone >= kSonyZoneCount) zone = kSonyZoneCount - 1U;
    return static_cast<std::uint8_t>(zone * 100U / (kSonyZoneCount - 1U));
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
    case TriggerMode::Race:
        // Mode 1: wall start, wall force.
        frame[3] = kModeRace;
        frame[4] = zonePercent(command.params[0]);
        frame[5] = scalePercent(command.params[1]);
        break;

    case TriggerMode::SniperBreak: {
        // Mode 2: wall start, snap/click point, wall force, return force, 1.
        // FORCEADAPT stores the snap point as a length relative to the start,
        // so the absolute click position is start + span.
        frame[3] = kModeShotBreak;
        const unsigned snap = static_cast<unsigned>(command.params[0]) +
                              static_cast<unsigned>(command.params[1]);
        frame[4] = zonePercent(command.params[0]);
        frame[5] = zonePercent(snap);
        frame[6] = scalePercent(command.params[2]);
        frame[7] = static_cast<std::uint8_t>(frame[6] / 2U);
        frame[8] = 0x01;
        break;
    }

    case TriggerMode::RecoilRattle:
    case TriggerMode::Vibration: {
        // Mode 3: start, cadence index (5..100, not Hz), amplitude.
        frame[3] = kModeGunVibration;
        frame[4] = zonePercent(command.params[0]);
        const std::uint8_t frequency =
            command.params[3] != 0 ? command.params[3] : command.params[1];
        std::uint8_t cadence = scalePercent(frequency);
        cadence = (std::max)(cadence, std::uint8_t{5});
        frame[5] = cadence;
        frame[6] = scalePercent(command.params[2]);
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
