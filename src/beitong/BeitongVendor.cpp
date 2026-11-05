#include "beitong/BeitongVendor.h"

#include "beitong/BeitongHybridInputSource.h"
#include "beitong/BeitongXInputSource.h"
#include "platform/HidTransport.h"
#include "platform/XInputGamepad.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>

namespace asb::beitong {
namespace {

bool isSupportedProduct(std::uint16_t productId) noexcept {
    return BeitongVendor::isSupportedProduct(productId);
}

bool matchesVendorCollection(const HidDeviceInfo& info) noexcept {
    return BeitongVendor::isSupportedVendor(info.vendorId) &&
           isSupportedProduct(info.productId) &&
           info.usagePage == BeitongVendor::kVendorUsagePage &&
           info.usage == BeitongVendor::kVendorUsage;
}

// Minimal UTF-16 -> UTF-8 conversion for the Windows product string. The core
// library builds on non-Windows hosts for tests as well, so this stays portable
// instead of pulling in WideCharToMultiByte.
std::string toUtf8(const std::wstring& text) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t index = 0; index < text.size(); ++index) {
        unsigned code = static_cast<unsigned>(text[index]);
        if (code >= 0xD800U && code <= 0xDBFFU && index + 1 < text.size()) {
            const unsigned low = static_cast<unsigned>(text[index + 1]);
            if (low >= 0xDC00U && low <= 0xDFFFU) {
                code = 0x10000U + ((code - 0xD800U) << 10U) + (low - 0xDC00U);
                ++index;
            }
        }
        if (code < 0x80U) {
            out.push_back(static_cast<char>(code));
        } else if (code < 0x800U) {
            out.push_back(static_cast<char>(0xC0U | (code >> 6U)));
            out.push_back(static_cast<char>(0x80U | (code & 0x3FU)));
        } else if (code < 0x10000U) {
            out.push_back(static_cast<char>(0xE0U | (code >> 12U)));
            out.push_back(static_cast<char>(0x80U | ((code >> 6U) & 0x3FU)));
            out.push_back(static_cast<char>(0x80U | (code & 0x3FU)));
        } else {
            out.push_back(static_cast<char>(0xF0U | (code >> 18U)));
            out.push_back(static_cast<char>(0x80U | ((code >> 12U) & 0x3FU)));
            out.push_back(static_cast<char>(0x80U | ((code >> 6U) & 0x3FU)));
            out.push_back(static_cast<char>(0x80U | (code & 0x3FU)));
        }
    }
    return out;
}

bool matchesGamepadCollection(const HidDeviceInfo& info) noexcept {
    return BeitongVendor::isSupportedVendor(info.vendorId) &&
           isSupportedProduct(info.productId) &&
           info.usagePage == 0x0001U && info.usage == 0x0005U;
}

// Locates the HID gamepad collection that carries the fast stick stream.
bool findGamepadInterface(HidDeviceInfo& found) {
    std::string enumError;
    const auto devices = platform::enumerateHidDevices(enumError);
    for (const auto& device : devices) {
        if (matchesGamepadCollection(device)) {
            found = device;
            return true;
        }
    }
    return false;
}

} // namespace

void VendorTransportDeleter::operator()(platform::HidTransport* transport) const noexcept {
    platform::destroyHidTransport(transport);
}

BeitongVendor::BeitongVendor(VendorTransportPtr transport, HidDeviceInfo info)
    : transport_(std::move(transport)), info_(std::move(info)) {}

BeitongVendor::~BeitongVendor() = default;

std::vector<HidDeviceInfo> BeitongVendor::findCandidates(std::string& error) {
    error.clear();
    auto all = platform::enumerateHidDevices(
        error, platform::HidStringQuery::AllDevices);
    std::vector<HidDeviceInfo> candidates;
    candidates.reserve(all.size());
    for (auto& info : all) {
        if (matchesVendorCollection(info)) {
            candidates.push_back(std::move(info));
        }
    }
    // Deterministic order: the wired collection first, then dongle-backed ones,
    // preserving enumeration order inside each group.
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const HidDeviceInfo& lhs, const HidDeviceInfo& rhs) {
                         return lhs.productId == kProductWired &&
                                rhs.productId != kProductWired;
                     });
    return candidates;
}

std::unique_ptr<BeitongVendor> BeitongVendor::open(const HidDeviceInfo& info,
                                                   std::string& error) {
    if (!matchesVendorCollection(info)) {
        error = "The selected interface is not a Beitong trigger HID collection "
                "(expected VID 0x20BC, PID 0x5173/0x5174, usage page 0x00FF, usage 0x0003).";
        return {};
    }

    error.clear();
    VendorTransportPtr transport(platform::createHidTransport(info, error));
    if (!transport || !transport->isOpen()) {
        if (error.empty()) {
            error = "The Beitong controller could not be opened.";
        }
        return {};
    }
    return std::unique_ptr<BeitongVendor>(
        new BeitongVendor(std::move(transport), info));
}

std::string_view BeitongVendor::backendName() const noexcept { return "beitong"; }

VendorCapabilities BeitongVendor::capabilities() const noexcept {
    VendorCapabilities caps{};
    caps.adaptiveTriggers = true;
    // Grip rumble uses standard XInput (XInputSetState) for the handle motors.
    // The vendor HID 0x76 command (shoulder motors) is never emitted.
    caps.rumble = true;
    caps.lightbar = false;
    caps.profiles = false;
    caps.pacedWrites = false;
    caps.haptics = false;
    caps.gyroTuning = false;
    // Input is the XInput/hybrid endpoint, not this vendor collection.
    caps.vendorInput = false;
    return caps;
}

const HidDeviceInfo& BeitongVendor::info() const noexcept { return info_; }

bool BeitongVendor::hasIdentity() const noexcept {
    return transport_ && transport_->isOpen();
}

bool BeitongVendor::isDongle() const noexcept {
    return info_.productId == kProductDongle;
}

std::string BeitongVendor::describeIdentity() const {
    std::string product = toUtf8(
        info_.product.empty() ? info_.friendlyName : info_.product);
    std::string base = "Beitong Kunpeng 70C";
    if (!product.empty()) {
        base += " (" + product + ")";
    }
    return base + (isDongle() ? " [dongle]" : " [wired]");
}

std::string BeitongVendor::connectionName() const {
    return isDongle() ? "2.4GHz dongle" : "USB";
}

std::uint8_t BeitongVendor::batteryPercent() const noexcept {
    std::string ignored;
    auto pad = platform::openXInputGamepadForDevice(
        info_.vendorId, info_.productId, std::nullopt, ignored);
    std::uint8_t percent = 100;
    std::uint8_t state = 0;
    if (pad && pad->queryBattery(percent, state)) {
        return percent;
    }
    return 100;
}

std::uint8_t BeitongVendor::chargeState() const noexcept {
    std::string ignored;
    auto pad = platform::openXInputGamepadForDevice(
        info_.vendorId, info_.productId, std::nullopt, ignored);
    std::uint8_t percent = 100;
    std::uint8_t state = 0;
    if (pad && pad->queryBattery(percent, state)) {
        return state;
    }
    return 0;
}

unsigned BeitongVendor::connectionTypeRaw() const noexcept { return 0; }

bool BeitongVendor::writeFrame(const TriggerFrame& frame, std::string& error) {
    if (!transport_ || !transport_->isOpen()) {
        error = "The Beitong controller is not open.";
        return false;
    }
    if (!transport_->writeOutputReport(frame, error)) {
        return false;
    }
    writes_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool BeitongVendor::clearAll(std::string& error) {
    error.clear();
    std::lock_guard lock(writeMutex_);
    translator_.reset();

    bool ok = true;
    for (const auto side : {TriggerSide::Left, TriggerSide::Right}) {
        const ForceTriggerCommand off{side, TriggerMode::Normal, {}};
        std::string writeError;
        if (!writeFrame(BeitongTriggerTranslator::encode(off), writeError)) {
            if (ok) error = writeError;
            ok = false;
        }
    }
    translator_.reset();
    return ok;
}

bool BeitongVendor::stopRumble(std::string& error) {
    // SAFETY: never emit 0x76 over HID (shoulder motors). Standard grip motors
    // are stopped through XInputSetState. If the XInput endpoint is asleep or
    // not yet enumerated, treat baseline stop as already quiet.
    error.clear();
    std::string openError;
    auto pad = platform::openXInputGamepadForDevice(
        info_.vendorId, info_.productId, std::nullopt, openError);
    if (!pad) {
        return true;
    }
    return pad->setVibration(0, 0, error);
}

VendorWriteStats BeitongVendor::triggerWriteStats() const noexcept {
    VendorWriteStats stats{};
    stats.writes = writes_.load(std::memory_order_relaxed);
    return stats;
}

VendorWriteStats BeitongVendor::rumbleWriteStats() const noexcept {
    VendorWriteStats stats{};
    stats.writes = rumbleWrites_.load(std::memory_order_relaxed);
    return stats;
}

std::unique_ptr<platform::PhysicalInputSource> BeitongVendor::openInputSource(
    std::optional<unsigned int> xinputIndex,
    std::string& error) {
    // Prefer the hybrid source: the HID gamepad collection carries the stick
    // axes at roughly 2.2 kHz, while XInput supplies the independent LT/RT that
    // the HID report cannot express (it encodes one differential trigger axis).
    // Fall back to plain XInput if that collection is absent, so other BeiTong
    // models still work with slower sticks.
    HidDeviceInfo gamepadInterface{};
    if (findGamepadInterface(gamepadInterface)) {
        std::string hybridError;
        if (auto hybrid = BeitongHybridInputSource::open(
                gamepadInterface, xinputIndex, hybridError)) {
            error.clear();
            return hybrid;
        }
    }
    return BeitongXInputSource::open(info_.vendorId, info_.productId, xinputIndex, error);
}

bool BeitongVendor::queueTriggerRaw(const ForceTriggerCommand& command,
                                    std::string& error) {
    error.clear();
    std::lock_guard lock(writeMutex_);

    std::array<TriggerFrame, 2> frames{};
    const std::size_t count = translator_.update(
        command, std::chrono::steady_clock::now(), frames);
    for (std::size_t index = 0; index < count; ++index) {
        if (!writeFrame(frames[index], error)) {
            return false;
        }
    }
    return true;
}

bool BeitongVendor::queueRumble(std::uint8_t lowFrequencyMotor,
                                std::uint8_t highFrequencyMotor,
                                std::string& error) {
    // SAFETY: never emit 0x76 on the vendor HID channel (shoulder motors).
    // Route standard left/right grip vibration through XInputSetState instead.
    error.clear();
    std::lock_guard lock(writeMutex_);
    auto pad = platform::openXInputGamepadForDevice(
        info_.vendorId, info_.productId, std::nullopt, error);
    if (!pad) {
        return false;
    }
    if (!pad->setVibration(lowFrequencyMotor, highFrequencyMotor, error)) {
        return false;
    }
    rumbleWrites_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool BeitongVendor::supportsBowEmulation() const noexcept {
    // Match Flydigi APEX 5: allow AdaptiveTriggerBridge::updateTriggerPositions
    // to transition DualSense 0x22 Bow effects from draw (Race) to rupture
    // (SniperBreak) at ruptureAxis and re-arm at rearmAxis.
    return true;
}

std::string_view BeitongVendor::vendorName() const noexcept { return "beitong"; }

} // namespace asb::beitong
