#include "beitong/BeitongDevice.h"
#include "beitong/BeitongInputSource.h"
#include "cli/Commands.h"
#include "dualsense/VirtualDualSense.h"
#include "platform/PhysicalInputSource.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

namespace asb::cli {
namespace {

void printBeitongUsage() {
    std::cout
        << "Usage: ApexSenseBridge beitong-bridge [options]\n"
        << "\n"
        << "Bridges a physical Beitong (Kunpeng) controller to a virtual DualSense,\n"
        << "routing the game's adaptive trigger and rumble output back to the pad.\n"
        << "\n"
        << "Options:\n"
        << "  --list         List detected Beitong controllers and exit.\n"
        << "  --index N      Use the Nth detected controller (default 0).\n"
        << "  --help         Show this help.\n";
}

std::string describe(const HidDeviceInfo& info) {
    std::string text;
    if (!info.product.empty()) {
        text.assign(info.product.begin(), info.product.end());
    } else {
        text = "Beitong controller";
    }
    char ids[32] = {};
    std::snprintf(ids, sizeof(ids), " [%04X:%04X]", info.vendorId, info.productId);
    return text + ids;
}

} // namespace

int commandBeitongBridge(int argc, char** argv) {
    bool listOnly = false;
    int index = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--list") {
            listOnly = true;
        } else if (arg == "--index" && i + 1 < argc) {
            index = std::atoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            printBeitongUsage();
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << '\n';
            printBeitongUsage();
            return 2;
        }
    }

    std::string error;
    const auto candidates = beitong::BeitongDevice::findCandidates(error);
    if (candidates.empty()) {
        std::cerr << "No Beitong controller detected.";
        if (!error.empty()) std::cerr << ' ' << error;
        std::cerr << '\n';
        return 3;
    }
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        std::cout << "  [" << i << "] " << describe(candidates[i]) << '\n';
    }
    if (listOnly) return 0;

    if (index < 0 || static_cast<std::size_t>(index) >= candidates.size()) {
        std::cerr << "Controller index " << index << " is out of range.\n";
        return 2;
    }

    auto device = beitong::BeitongDevice::open(candidates[static_cast<std::size_t>(index)], error);
    if (!device) {
        std::cerr << "Could not open the Beitong controller: " << error << '\n';
        return 3;
    }
    std::cout << "Opened " << describe(device->info()) << '\n';

    auto backend = dualsense::createVirtualDualSense(dualsense::VirtualDualSenseOptions{});
    if (!backend) {
        std::cerr << "Could not create the virtual DualSense backend.\n";
        return 4;
    }

    // The pad's effect translation is driven straight from the raw DualSense
    // effect blocks, so failures here are reported rather than swallowed.
    auto feedbackHandler = [&device](const dualsense::DualSenseFeedback& feedback) {
        std::string writeError;
        if (!device->submitFeedback(feedback, writeError)) {
            std::cerr << "\nBeitong feedback write failed: " << writeError << '\n';
        }
    };

    if (!backend->open(error, feedbackHandler)) {
        std::cerr << "Could not start the virtual DualSense: " << error << '\n';
        return 4;
    }

    beitong::BeitongInputSource input(*device);
    dualsense::DualSenseInputState state{};
    std::cout << "Bridging via " << input.backendName() << ". Press Ctrl+C to stop.\n";

    for (;;) {
        const auto status = input.waitForState(state, std::chrono::milliseconds(100), error);
        if (status == platform::PhysicalInputStatus::State) {
            if (!backend->updateInput(state, error)) {
                std::cerr << "\nVirtual DualSense input update failed: " << error << '\n';
                break;
            }
            continue;
        }
        if (status == platform::PhysicalInputStatus::Disconnected) {
            std::cerr << "\nThe Beitong controller disconnected: " << error << '\n';
            break;
        }
        // Timeouts are the normal idle case: the pad only reports on change.
    }

    backend->close();
    return 0;
}

} // namespace asb::cli
