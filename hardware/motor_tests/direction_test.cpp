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

/**
 * Runs a single Talon SRX at a given percent output for a specified duration.
 * 
 * @param talon   Reference to the TalonSRX to command
 * @param percent Percent output: -1.0 (full reverse) to +1.0 (full forward)
 * @param seconds How long to hold this output
 * 
 * FeedEnable is called every loop iteration — Phoenix 5 on Linux requires this
 * to keep the Talon out of safety neutral. The 100ms timeout must exceed the
 * 10ms sleep or the Talon will cut out between calls.
 */
void run_motor(TalonSRX& talon, float percent, int seconds) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
        talon.Set(ControlMode::PercentOutput, percent);
        // 10ms sleep = ~100 Hz loop rate, well within FeedEnable 100ms window
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    // Specify the CAN interface — matches the interface brought up via ip link on the Jetson
    std::string interface = "can0";

    // Instantiate single Talon SRX on CAN ID 1 (Front Left motor)
    // Used alone here to isolate and verify one motor's direction before full 4-motor testing
    TalonSRX talon{1, interface};  // device ID 1

    // --- Direction Test Sequence ---
    // Purpose: confirm that positive output = forward and negative = reverse
    // for this motor's physical mounting orientation.
    // If directions are swapped, toggle SetInverted() and rebuild.

    // Step 1: Forward at 20% for 3 seconds
    // 20% is safely above the 10% minimum threshold and slow enough to observe clearly
    std::cout << "Forward 20% for 3 seconds...\n";
    run_motor(talon, 0.20, 3);

    // Step 2: Coast to a stop for 2 seconds before reversing
    // Prevents abrupt direction change from stressing the gearbox
    std::cout << "Stopping for 2 seconds...\n";
    run_motor(talon, 0.0, 2);

    // Step 3: Reverse at 20% for 3 seconds
    // Negative value drives the motor in the opposite direction
    std::cout << "Reverse 20% for 3 seconds...\n";
    run_motor(talon, -0.20, 3);

    // Step 4: Final stop — hold zero output for 2 seconds for clean shutdown
    std::cout << "Stopping.\n";
    run_motor(talon, 0.0, 2);

    std::cout << "Done\n";
    return 0;
}
