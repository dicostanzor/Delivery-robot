// Exclude WPI (FRC framework) compatibility layer — required for non-FRC Linux builds
#define Phoenix_No_WPI

#include "ctre/Phoenix.h"                              // Core Phoenix 5 API (TalonSRX, ControlMode, etc.)
#include "ctre/phoenix/platform/Platform.hpp"          // Platform initialization for Linux CAN
#include "ctre/phoenix/unmanaged/Unmanaged.h"          // FeedEnable — required to keep Talons active on Linux
#include <chrono>                                      // for timing
#include <iostream>                                    // status messages
#include <thread>                                      // sleep

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

int main() {
    // Specify the CAN interface — matches the interface brought up via ip link on the Jetson
    std::string interface = "can0";

    // Instantiate all four Talon SRXs by CAN ID, all bound to can0
    TalonSRX fl{1, interface};   // Front Left
    TalonSRX fr{2, interface};   // Front Right
    TalonSRX rl{3, interface};   // Rear Left
    TalonSRX rr{4, interface};   // Rear Right

    // IDs 1 & 2 (front left, front right): not inverted — positive output = forward
    // IDs 3 & 4 (rear left, rear right): inverted — rear motors are mounted in the
    // opposite orientation on the chassis, so inverting keeps positive = forward consistent
    // across all four wheels for correct mecanum kinematics
    fl.SetInverted(false);
    fr.SetInverted(false);
    rl.SetInverted(true);
    rr.SetInverted(true);

    // Lambda to run all four motors at a given percent output for a given duration.
    // Captures all four Talons by reference so it can command them inside the loop.
    auto run = [&](double percent, int seconds) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
            // FeedEnable must be called repeatedly — Phoenix 5 on Linux requires this
            // to keep Talons out of safety neutral. The 100ms timeout must be longer
            // than the 10ms sleep below or the Talons will cut out between calls
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

            // Send the same percent output to all four motors simultaneously
            fl.Set(ControlMode::PercentOutput, percent);
            fr.Set(ControlMode::PercentOutput, percent);
            rl.Set(ControlMode::PercentOutput, percent);
            rr.Set(ControlMode::PercentOutput, percent);

            // 10ms sleep = ~100 Hz loop rate, well within FeedEnable 100ms window
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    // --- Test Sequence ---

    // Step 1: All four motors forward at 10% for 5 seconds
    // 10% is just above the minimum working threshold for CIM + Talon SRX
    // Short 5s window — enough to confirm all 4 respond without running long unattended
    std::cout << "Forward 5 seconds...\n";
    run(0.10, 5);

    // Step 2: All four motors reverse at 10% for 5 seconds
    // Confirms negative output produces reverse on all four wheels correctly
    std::cout << "Reverse 5 seconds...\n";
    run(-0.10, 5);

    // Step 3: Hold zero output for 1 second — clean controlled stop
    // Continues FeedEnable so Talons receive the stop command rather than timing out
    std::cout << "Stopping\n";
    run(0.0, 1);

    std::cout << "Done\n";
    return 0;
}
