// Exclude WPI (FRC framework) compatibility layer — required for non-FRC Linux builds
#define Phoenix_No_WPI

#include "ctre/Phoenix.h"                              // Core Phoenix 5 API (TalonSRX, ControlMode, etc.)
#include "ctre/phoenix/platform/Platform.hpp"          // Platform initialization for Linux CAN
#include "ctre/phoenix/unmanaged/Unmanaged.h"          // FeedEnable — required to keep Talons active on Linux
#include <chrono>                                      // std::chrono for timing
#include <iostream>                                    // std::cout for status messages
#include <thread>                                      // std::this_thread::sleep_for

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

int main() {
    // Specify the CAN interface — matches the interface brought up via ip link on the Jetson
    std::string interface = "can0";

    // Instantiate two Talon SRXs — one per side of the robot (front pair or rear pair)
    TalonSRX talon1{1, interface};   // CAN ID 1
    TalonSRX talon2{2, interface};   // CAN ID 2

    // talon1 (left side): not inverted — positive output = forward
    talon1.SetInverted(false);
    // talon2 (right side): inverted — motors are physically mirrored on the chassis,
    // so inverting makes positive output = forward on both sides consistently
    talon2.SetInverted(true);

    // Lambda to run both motors at a given percent output for a given duration
    // Captures talon1 and talon2 by reference so it can command both inside the loop
    auto run = [&](double percent, int seconds) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
            // FeedEnable must be called repeatedly — Phoenix 5 on Linux requires this
            // to keep the Talons out of safety neutral. Timeout arg (100ms) must be
            // longer than the sleep below (10ms) or the Talon will cut out between calls
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

            // Send the same percent output to both motors simultaneously
            talon1.Set(ControlMode::PercentOutput, percent);
            talon2.Set(ControlMode::PercentOutput, percent);

            // 10ms sleep = ~100 Hz loop rate, well within FeedEnable 100ms window
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    // --- Test Sequence ---

    // Step 1: Drive both motors forward at 10% for 15 seconds
    // 10% is just above the minimum working threshold for CIM + Talon SRX
    std::cout << "Forward 15 seconds...\n";
    run(0.10, 15);

    // Step 2: Drive both motors in reverse at 10% for 15 seconds
    // Verifies that negative output produces the expected reverse direction on both sides
    std::cout << "Reverse 15 seconds...\n";
    run(-0.10, 15);

    // Step 3: Hold zero output for 1 second — clean controlled stop
    // Still calls FeedEnable so the Talon receives the stop command properly
    std::cout << "Stopping\n";
    run(0.0, 1);

    std::cout << "Done\n";
    return 0;
}
