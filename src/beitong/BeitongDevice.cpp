#include "beitong/BeitongDevice.h"

#include <algorithm>
#include <array>
#include <cstring>

namespace asb::beitong {
namespace {

// Report framing shared with the reference implementation:
//   02 56 <side> <mode> <params...>   trigger effect (9 bytes on the wire)
//   02 76 01 <sub> <value>            grip rumble (6 bytes on the wire)
constexpr std::uint8_t kReportIdOutput = 0x02;
constexpr std::uint8_t kCommandTrigger = 0x56;
constexpr std::uint8_t kCommandRumble = 0x76;
constexpr std::uint8_t kRumbleGroupA = 0x04;

constexpr std::uint16_t kGamepadUsagePage = 0x0001;
constexpr std::uint16_t kGamepadUsage = 0x0005;

// Preferred order: the pad collections first, then dongle-backed identities.
constexpr std::uint16_t kSupportedProducts[] = {
    BEITONG_PID_KP70C2_PAD,
    BEITONG_PID_KP70C1_PAD,
    BEITONG_PID_KP70C_XINPUT,
    BEITONG_PID_ZEUS_KP70,
    BEITONG_PID_ASURA_KP70,
    BEITONG_PID_KP70_WIRELESS,
    BEITONG_PID_KP70C2_DONGLE,
    BEITONG_PID_KP70C1_DONGLE,
};

constexpr std::uint16_t kDongleProducts[] = {
    BEITONG_PID_KP70_WIRELESS,
    BEITONG_PID_KP70C2_DONGLE,
    BEITONG_PID_KP70C1_DONGLE,
    BEITONG_PID_KP70C_XINPUT,
};

bool isDongleProduct(std::uint16_t productId) noexcept {
    return std::find(std::begin(kDongleProducts), std::end(kDongleProducts), productId) !=
           std::end(kDongleProducts);
}

std::uint8_t clampPercent(int value) noexcept {
    if (value < 0) return 0;
    if (value > 100) return 100;
    return static_cast<std::uint8_t>(value);
}

} // namespace

bool isSupportedDevice(std::uint16_t vendorId, std::uint16_t productId) noexcept {
    if (vendorId != BEITONG_VENDOR_ID && vendorId != BEITONG_ALTER_VENDOR_ID) return false;
    return std::find(std::begin(kSupportedProducts), std::end(kSupportedProducts), productId) !=
           std::end(kSupportedProducts);
}

void TransportDeleter::operator()(platform::HidTransport* transport) const noexcept {
    platform::destroyHidTransport(transport);
}

BeitongDevice::BeitongDevice(TransportPtr transport) : transport_(std::move(transport)) {
    const bool bluetooth = transport_ && isDongleProduct(transport_->info().productId);
    beitong_bridge_init(&bridge_, bluetooth);
}

BeitongDevice::~BeitongDevice() = default;
BeitongDevice::BeitongDevice(BeitongDevice&& other) noexcept
    : transport_(std::move(other.transport_)), bridge_(other.bridge_) {}

BeitongDevice& BeitongDevice::operator=(BeitongDevice&& other) noexcept {
    if (this != &other) {
        transport_ = std::move(other.transport_);
        bridge_ = other.bridge_;
    }
    return *this;
}

std::vector<HidDeviceInfo> BeitongDevice::findCandidates(std::string& error) {
    auto all = platform::enumerateHidDevices(error, platform::HidStringQuery::AllDevices);
    std::vector<HidDeviceInfo> candidates;
    std::copy_if(all.begin(), all.end(), std::back_inserter(candidates), [](const HidDeviceInfo& info) {
        if (!isSupportedDevice(info.vendorId, info.productId)) return false;
        // A Beitong pad exposes several collections; the gamepad collection is
        // the one carrying the input reports we translate.
        return info.usagePage == kGamepadUsagePage && info.usage == kGamepadUsage;
    });

    // Stable order: pad first, then the rest, keeping enumeration order within
    // each group so a user with two pads always gets the same one.
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const HidDeviceInfo& lhs, const HidDeviceInfo& rhs) {
                         return !isDongleProduct(lhs.productId) && isDongleProduct(rhs.productId);
                     });
    return candidates;
}

std::unique_ptr<BeitongDevice> BeitongDevice::open(const HidDeviceInfo& info, std::string& error) {
    TransportPtr transport(platform::createHidTransport(info, error));
    if (!transport || !transport->isOpen()) {
        if (error.empty()) error = "The Beitong controller could not be opened.";
        return nullptr;
    }
    return std::make_unique<BeitongDevice>(std::move(transport));
}

bool BeitongDevice::isOpen() const noexcept {
    return transport_ && transport_->isOpen();
}

const HidDeviceInfo& BeitongDevice::info() const {
    return transport_->info();
}

bool BeitongDevice::isBluetooth() const noexcept {
    return bridge_.is_bluetooth;
}

platform::HidReadStatus BeitongDevice::readInputReport(std::span<std::uint8_t> report,
                                                       std::chrono::milliseconds timeout,
                                                       std::size_t& bytesRead,
                                                       std::string& error) {
    if (!isOpen()) {
        error = "The Beitong controller is not open.";
        return platform::HidReadStatus::Error;
    }
    return transport_->readInputReport(report, timeout, bytesRead, error);
}

int BeitongDevice::translateInput(const std::uint8_t* data,
                                  std::size_t len,
                                  std::span<std::uint8_t> out) {
    std::lock_guard lock(stateMutex_);
    const int written = beitong_translate_beitong_to_game(
        &bridge_, data, len, out.data(), out.size());
    if (written > 0) ++inputReports_;
    return written;
}

bool BeitongDevice::submitFeedback(const dualsense::DualSenseFeedback& feedback,
                                   std::string& error) {
    if (!isOpen()) {
        error = "The Beitong controller is not open.";
        return false;
    }
    if (feedback.kind != dualsense::FeedbackKind::HidOutput) return true;

    beitong_translation_result_t result{};
    {
        std::lock_guard lock(stateMutex_);
        // Rebuild the DualSense common payload the translation expects. The
        // effect blocks are [mode, params x10], which is exactly the wire
        // layout of right/left_trigger_mode + params.
        std::array<std::uint8_t, sizeof(ds_usb_output_report_t)> report{};
        report[0] = kReportIdOutput;
        auto* common = reinterpret_cast<ds_output_common_t*>(report.data() + 1);
        common->valid_flag0 = feedback.enableBits1;
        common->valid_flag1 = feedback.enableBits2;
        common->motor_right = feedback.rumbleRight;
        common->motor_left = feedback.rumbleLeft;
        std::copy(feedback.rightTriggerEffect.begin(), feedback.rightTriggerEffect.end(),
                  reinterpret_cast<std::uint8_t*>(&common->right_trigger_mode));
        std::copy(feedback.leftTriggerEffect.begin(), feedback.leftTriggerEffect.end(),
                  reinterpret_cast<std::uint8_t*>(&common->left_trigger_mode));

        if (beitong_translate_game_to_beitong(&bridge_, report.data(), report.size(), &result) <= 0) {
            return true;
        }
        ++feedbackReports_;
    }

    // Device writes happen outside the state lock: the virtual controller's
    // feedback thread must not serialise behind HID I/O.
    for (std::size_t i = 0; i < result.trigger_count; ++i) {
        if (!transport_->writeOutputReport(
                std::span(result.trigger_pkts[i].data, result.trigger_pkts[i].len), error)) {
            return false;
        }
        ++writes_;
    }
    if (result.rumble_len != 0) {
        if (!transport_->writeOutputReport(
                std::span(result.rumble_pkt.data, result.rumble_pkt.len), error)) {
            return false;
        }
        ++writes_;
    }
    return true;
}

bool BeitongDevice::queueTriggerRaw(const ForceTriggerCommand& command, std::string& error) {
    std::uint8_t mode = BT_TRIGGER_OFF;
    std::array<std::uint8_t, 5> params{};

    // The generic trigger command uses Flydigi mode semantics; the Beitong pad
    // has four native behaviours, so this is a lossy but predictable mapping.
    // The faithful path is submitFeedback(), which keeps the original DualSense
    // effect blocks intact.
    switch (command.mode) {
        case TriggerMode::Normal:
            mode = BT_TRIGGER_OFF;
            break;
        case TriggerMode::Race:
            mode = BT_TRIGGER_RACE_RESISTANCE;
            params[0] = clampPercent(command.params[0]);
            params[1] = clampPercent(command.params[1]);
            break;
        case TriggerMode::RecoilRattle:
            mode = BT_TRIGGER_SHOT_BREAKPOINT;
            params[0] = clampPercent(command.params[0]);
            params[1] = clampPercent(command.params[1]);
            params[2] = clampPercent(command.params[2]);
            break;
        case TriggerMode::SniperBreak:
            mode = BT_TRIGGER_SHOT_BREAKPOINT;
            params[0] = clampPercent(command.params[0]);
            params[1] = clampPercent(command.params[1]);
            params[2] = clampPercent(command.params[4] != 0 ? command.params[4] : command.params[2]);
            break;
        case TriggerMode::Lock:
            mode = BT_TRIGGER_RACE_RESISTANCE;
            params[0] = 100;
            params[1] = 100;
            break;
        case TriggerMode::Vibration:
            mode = BT_TRIGGER_GUN_VIBRATION;
            params[0] = clampPercent(command.params[0]);
            params[1] = clampPercent(command.params[1]);
            params[2] = clampPercent(command.params[2]);
            break;
    }

    const std::uint8_t side =
        command.side == TriggerSide::Left ? BT_TRIGGER_LEFT : BT_TRIGGER_RIGHT;
    return writeTriggerPacket(side, mode, params.data(), params.size(), error);
}

bool BeitongDevice::queueRumble(std::uint8_t lowFrequencyMotor,
                                std::uint8_t highFrequencyMotor,
                                std::string& error) {
    if (!isOpen()) {
        error = "The Beitong controller is not open.";
        return false;
    }

    // The pad has a single grip motor, driven by the stronger of the two
    // requested levels, matching the reference's motor merge.
    const std::uint8_t requested = std::max(lowFrequencyMotor, highFrequencyMotor);
    std::uint8_t level = 0;
    {
        std::lock_guard lock(stateMutex_);
        const int amplified =
            static_cast<int>(requested * bridge_.rumble_gain + 0.5f);
        level = clampPercent(amplified);
        bridge_.motor_left = level;
        bridge_.motor_right = level;
    }

    std::array<std::uint8_t, 6> packet{};
    packet[0] = kReportIdOutput;
    packet[1] = kCommandRumble;
    packet[2] = 0x01;
    packet[3] = kRumbleGroupA;
    packet[4] = level;
    if (!transport_->writeOutputReport(packet, error)) return false;
    ++writes_;
    return true;
}

bool BeitongDevice::supportsBowEmulation() const noexcept {
    // The pad implements its own resistance ramp, so the host must not stack a
    // synthetic bow retrigger on top of a native effect.
    return false;
}

std::string_view BeitongDevice::vendorName() const noexcept {
    return "beitong";
}

void BeitongDevice::setGains(float forceGain, float vibeGain, float rumbleGain,
                             std::uint8_t minForceFloor) {
    std::lock_guard lock(stateMutex_);
    beitong_bridge_set_gains(&bridge_, forceGain, vibeGain, rumbleGain, minForceFloor);
}

void BeitongDevice::setCurve(int curveType) {
    std::lock_guard lock(stateMutex_);
    beitong_bridge_set_curve(&bridge_, curveType);
}

void BeitongDevice::setDeadzones(std::uint8_t ltStart, std::uint8_t ltEnd,
                                 std::uint8_t rtStart, std::uint8_t rtEnd) {
    std::lock_guard lock(stateMutex_);
    beitong_bridge_set_deadzones(&bridge_, ltStart, ltEnd, rtStart, rtEnd);
}

std::uint64_t BeitongDevice::writes() const noexcept {
    std::lock_guard lock(stateMutex_);
    return writes_;
}

std::uint64_t BeitongDevice::feedbackReports() const noexcept {
    std::lock_guard lock(stateMutex_);
    return feedbackReports_;
}

std::uint64_t BeitongDevice::inputReports() const noexcept {
    std::lock_guard lock(stateMutex_);
    return inputReports_;
}

bool BeitongDevice::writeTriggerPacket(std::uint8_t side,
                                       std::uint8_t mode,
                                       const std::uint8_t* params,
                                       std::size_t paramCount,
                                       std::string& error) {
    if (!isOpen()) {
        error = "The Beitong controller is not open.";
        return false;
    }
    std::array<std::uint8_t, 10> packet{};
    packet[0] = kReportIdOutput;
    packet[1] = kCommandTrigger;
    packet[2] = side;
    packet[3] = mode;
    const std::size_t copyCount = std::min<std::size_t>(paramCount, packet.size() - 4);
    if (copyCount != 0) std::copy_n(params, copyCount, packet.begin() + 4);

    // Nine bytes on the wire: the trigger payload ends at params[4]. The pad
    // accepts the trailing byte, but the reference writes 9 and we match it.
    if (!transport_->writeOutputReport(std::span(packet.data(), 9), error)) return false;
    ++writes_;
    return true;
}

} // namespace asb::beitong
