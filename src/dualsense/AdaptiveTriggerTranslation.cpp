#include "dualsense/AdaptiveTriggerTranslation.h"

#include <algorithm>

namespace asb::dualsense {
namespace {

ForceTriggerCommand make(TriggerSide side, TriggerMode mode,
                         std::array<std::uint8_t, 5> params = {}) {
    return ForceTriggerCommand{side, mode, params};
}

std::uint8_t strokeForZone(unsigned zone) noexcept {
    constexpr unsigned kMaximumStroke = 192;
    return static_cast<std::uint8_t>((zone * kMaximumStroke + 5U) / 10U);
}

std::optional<std::uint8_t> limitedForce(std::uint8_t strength) noexcept {
    constexpr unsigned kLimitedMaximum = 10;
    constexpr unsigned kNativeForceCeiling = 64;
    if (strength > kLimitedMaximum) return std::nullopt;
    return static_cast<std::uint8_t>(
        (strength * kNativeForceCeiling + kLimitedMaximum / 2U) / kLimitedMaximum);
}

struct ZoneRange {
    unsigned first = 10;
    unsigned last = 0;
    unsigned active = 0;
};

ZoneRange scanZones(unsigned mask) noexcept {
    ZoneRange range;
    for (unsigned zone = 0; zone < 10; ++zone) {
        if ((mask & (1U << zone)) == 0) continue;
        range.first = (std::min)(range.first, zone);
        range.last = zone;
        ++range.active;
    }
    return range;
}

// Single-zone masks widen to a one-zone span; sparse masks use first..last.
ForceTriggerCommand makeBreak(TriggerSide side, ZoneRange zones, unsigned strength) {
    if (zones.first == zones.last) {
        zones.last = (std::min)(zones.first + 1U, 9U);
        zones.first = zones.last - 1U;
    }
    const auto start = strokeForZone(zones.first);
    const auto length = static_cast<std::uint8_t>(strokeForZone(zones.last) - start);
    return make(side, TriggerMode::SniperBreak,
                {start, length, static_cast<std::uint8_t>(strength * 8U), 0, 0});
}

ForceTriggerCommand makeBow(TriggerSide side, ZoneRange zones, unsigned drawStrength,
                            unsigned snapForce) {
    constexpr unsigned kLastZone = 9;
    constexpr unsigned kStrongSnap = 7;
    const auto start = strokeForZone(zones.first);
    const auto resistance = static_cast<std::uint8_t>(drawStrength * 8U);
    if (zones.first >= zones.last || zones.last >= kLastZone || snapForce < kStrongSnap) {
        return make(side, TriggerMode::Race, {start, resistance, 0, 0, 0});
    }
    const auto length = static_cast<std::uint8_t>(strokeForZone(zones.last) - start);
    return make(side, TriggerMode::SniperBreak, {start, length, resistance, 0, 0});
}

std::optional<ForceTriggerCommand> translateNativeTrigger(
    TriggerSide side, const std::array<std::uint8_t, 11>& effect) {
    const auto mask = static_cast<unsigned>(effect[1]) |
                      (static_cast<unsigned>(effect[2]) << 8U);
    if ((mask & ~0x03FFU) != 0) {
        // DualSenseX / DualSensitive custom and extended profile fallback:
        // PC mods like Control pass effect[1] as start position and effect[2..3] as force/end,
        // which sets bits above 0x03FF. When effect[3] is non-zero, map it to the corresponding
        // FORCEADAPT mode rather than rejecting it as an invalid Sony bitmask.
        if (effect[3] != 0) {
            const std::uint8_t start = effect[1];
            const std::uint8_t force = effect[3];
            if (effect[0] == 0x25) {
                const std::uint8_t snap = effect[2] > start ? static_cast<std::uint8_t>(effect[2] - start) : 30;
                return make(side, TriggerMode::SniperBreak, {start, snap, force, 0, 0});
            }
            if (effect[0] == 0x26 || effect[0] == 0x23 || effect[0] == 0x27) {
                const std::uint8_t freq = effect[4] != 0 ? effect[4] : (effect[9] != 0 ? effect[9] : 60);
                return make(side, TriggerMode::RecoilRattle, {start, 1, force, freq, 0});
            }
            if (effect[0] == 0x21 || effect[0] == 0x22) {
                return make(side, TriggerMode::Race, {start, force, 0, 0, 0});
            }
        }
        return std::nullopt;
    }
    if (mask == 0) return make(side, TriggerMode::Normal);

    const auto zones = scanZones(mask);
    const auto start = strokeForZone(zones.first);
    switch (effect[0]) {
    case 0x22:
        return makeBow(side, zones, (effect[3] & 0x07U) + 1U, ((effect[3] >> 3U) & 0x07U) + 1U);
    case 0x25:
        return makeBreak(side, zones, (effect[3] & 0x07U) + 1U);
    case 0x23:
        if (effect[4] == 0) return make(side, TriggerMode::Normal);
        return make(side, TriggerMode::RecoilRattle, {start, 1, 60, effect[4], 0});
    case 0x27: {
        if (effect[4] == 0) return make(side, TriggerMode::Normal);
        const auto average = ((effect[3] & 0x07U) + ((effect[3] >> 3U) & 0x07U)) / 2U + 1U;
        return make(side, TriggerMode::RecoilRattle,
                    {start, 1, static_cast<std::uint8_t>(average * 10U), effect[4], 0});
    }
    default: break;
    }

    const auto packed = static_cast<std::uint32_t>(effect[3]) |
                        (static_cast<std::uint32_t>(effect[4]) << 8U) |
                        (static_cast<std::uint32_t>(effect[5]) << 16U) |
                        (static_cast<std::uint32_t>(effect[6]) << 24U);
    std::array<unsigned, 10> strengths{};
    unsigned peakStrength = 0;
    for (unsigned zone = 0; zone < 10; ++zone) {
        if ((mask & (1U << zone)) == 0) continue;
        strengths[zone] = ((packed >> (zone * 3U)) & 0x07U) + 1U;
        peakStrength = (std::max)(peakStrength, strengths[zone]);
    }

    if (effect[0] == 0x21) {
        const auto resistance = static_cast<std::uint8_t>(peakStrength * 8U);
        return make(side, TriggerMode::Race, {start, resistance, 0, 0, 0});
    }

    const auto frequency = effect[9];
    if (frequency == 0) return make(side, TriggerMode::Normal);
    // Start where the effect reaches half its peak and average from there, not the peak.
    unsigned first = zones.first;
    while (strengths[first] * 2U < peakStrength) ++first;
    unsigned sum = 0;
    unsigned count = 0;
    for (unsigned zone = first; zone < 10; ++zone) {
        if (strengths[zone] == 0) continue;
        sum += strengths[zone];
        ++count;
    }
    auto amplitude = (sum * 10U + count / 2U) / count;
    if (frequency < 10) amplitude /= 2U;
    return make(side, TriggerMode::RecoilRattle,
                {strokeForZone(first), 1,
                 static_cast<std::uint8_t>((std::max)(amplitude, 1U)), frequency, 0});
}

unsigned axisForStroke(unsigned stroke) noexcept {
    return (std::min)(255U, stroke * 255U / 192U);
}

} // namespace

std::optional<BowEmulation> describeBowEmulation(
    TriggerSide side, const std::array<std::uint8_t, 11>& effect) {
    constexpr unsigned kLastZone = 9;
    constexpr unsigned kStrongSnap = 7;
    constexpr std::uint8_t kRuptureLength = 40;
    constexpr unsigned kRuptureAxisPercent = 85;
    constexpr unsigned kRearmMargin = 10;
    if (effect[0] != 0x22) return std::nullopt;
    const auto mask = static_cast<unsigned>(effect[1]) |
                      (static_cast<unsigned>(effect[2]) << 8U);
    if (mask == 0 || (mask & ~0x03FFU) != 0) return std::nullopt;
    const auto zones = scanZones(mask);
    const unsigned drawStrength = (effect[3] & 0x07U) + 1U;
    const unsigned snapForce = ((effect[3] >> 3U) & 0x07U) + 1U;
    if (zones.first >= zones.last || zones.last >= kLastZone || snapForce < kStrongSnap) {
        return std::nullopt;
    }
    const auto start = strokeForZone(zones.first);
    const auto end = strokeForZone(zones.last);
    const auto resistance = static_cast<std::uint8_t>(drawStrength * 8U);
    BowEmulation bow;
    bow.draw = make(side, TriggerMode::Race, {start, resistance, 0, 0, 0});
    bow.rupture = make(side, TriggerMode::SniperBreak, {end, kRuptureLength, resistance, 0, 0});
    bow.ruptureAxis = static_cast<std::uint8_t>(axisForStroke(end) * kRuptureAxisPercent / 100U);
    const auto startAxis = axisForStroke(start);
    bow.rearmAxis = static_cast<std::uint8_t>(
        startAxis > 2U * kRearmMargin ? startAxis - kRearmMargin : kRearmMargin);
    return bow;
}

std::optional<ForceTriggerCommand> translateAdaptiveTrigger(
    TriggerSide side, const std::array<std::uint8_t, 11>& effect,
    std::uint8_t) {
    const auto type = effect[0];
    if (type == 1) return make(side, TriggerMode::Race, {effect[1], effect[2], 0, 0, 0});
    if (type == 0x11) {
        const auto force = limitedForce(effect[2]);
        if (!force) return std::nullopt;
        if (*force == 0) return make(side, TriggerMode::Normal);
        return make(side, TriggerMode::Race, {effect[1], *force, 0, 0, 0});
    }
    if (type == 2 || type == 0x12) {
        if (effect[2] < effect[1]) return std::nullopt;
        const auto length = static_cast<std::uint8_t>((std::max)(effect[2] - effect[1], 1));
        if (type == 2) return make(side, TriggerMode::SniperBreak, {effect[1], length, effect[3], 0, 0});
        const auto force = limitedForce(effect[3]);
        if (!force) return std::nullopt;
        if (*force == 0) return make(side, TriggerMode::Normal);
        return make(side, TriggerMode::SniperBreak, {effect[1], length, *force, 0, 0});
    }
    if (type == 5) return make(side, TriggerMode::Normal);
    if (type == 6) return make(side, TriggerMode::RecoilRattle,
                              {effect[3], effect[2], effect[2], effect[1], 0});
    if ((type >= 0x21 && type <= 0x23) || type == 0x25 || type == 0x26 || type == 0x27) {
        return translateNativeTrigger(side, effect);
    }
    return std::nullopt;
}

} // namespace asb::dualsense
