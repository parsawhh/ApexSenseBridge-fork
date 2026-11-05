#pragma once

#include "core/VendorDevice.h"
#include "flydigi/Apex5Device.h"

#include <string>

namespace asb {

// Restores a neutral trigger baseline when the owning session exits.
//
// Construction from either a concrete Flydigi device (device commands, tools)
// or the vendor-neutral session contract (the bridge) keeps one exit path for
// every controller family.
class TriggerResetGuard {
public:
    explicit TriggerResetGuard(flydigi::Apex5Device& device) noexcept : device_(&device) {}
    explicit TriggerResetGuard(VendorDevice& vendor) noexcept : vendor_(&vendor) {}
    ~TriggerResetGuard() noexcept {
        std::string ignored;
        if (device_) {
            device_->clearAll(ignored);
        } else if (vendor_) {
            vendor_->clearAll(ignored);
        }
    }

    TriggerResetGuard(const TriggerResetGuard&) = delete;
    TriggerResetGuard& operator=(const TriggerResetGuard&) = delete;

    void dismiss() noexcept {
        device_ = nullptr;
        vendor_ = nullptr;
    }

private:
    flydigi::Apex5Device* device_ = nullptr;
    VendorDevice* vendor_ = nullptr;
};

} // namespace asb
