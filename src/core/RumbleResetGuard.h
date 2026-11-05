#pragma once

#include "core/VendorDevice.h"
#include "flydigi/Apex5Device.h"

#include <string>

namespace asb {

// Restores a stopped rumble baseline when the owning session exits.
class RumbleResetGuard {
public:
    explicit RumbleResetGuard(flydigi::Apex5Device& device) noexcept : device_(&device) {}
    explicit RumbleResetGuard(VendorDevice& vendor) noexcept : vendor_(&vendor) {}
    ~RumbleResetGuard() noexcept {
        std::string ignored;
        if (device_) {
            device_->stopRumble(ignored);
        } else if (vendor_) {
            vendor_->stopRumble(ignored);
        }
    }

    RumbleResetGuard(const RumbleResetGuard&) = delete;
    RumbleResetGuard& operator=(const RumbleResetGuard&) = delete;

    void dismiss() noexcept {
        device_ = nullptr;
        vendor_ = nullptr;
    }

private:
    flydigi::Apex5Device* device_ = nullptr;
    VendorDevice* vendor_ = nullptr;
};

} // namespace asb
