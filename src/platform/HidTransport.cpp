#include "platform/HidTransport.h"

#include "flydigi/Apex4Protocol.h"
#include "flydigi/Apex5Protocol.h"

namespace asb::platform {

bool queriesHidStrings(HidStringQuery query,
                       std::uint16_t vendorId,
                       std::uint16_t productId) noexcept {
    constexpr std::uint16_t kSonyVendorId = 0x054C;
    constexpr std::uint16_t kBeitongVendorId = 0x20BC;
    constexpr std::uint16_t kBeitongAlterVendorId = 0x20DD;
    return query == HidStringQuery::AllDevices || vendorId == flydigi::kVendorId ||
           vendorId == kSonyVendorId || vendorId == kBeitongVendorId ||
           vendorId == kBeitongAlterVendorId ||
           flydigi::isApex4Product(vendorId, productId);
}

}
