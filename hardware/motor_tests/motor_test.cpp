// Exclude WPI compatibility layer - for non FRC Linux
#define Phoenix_No_WPI

#include "ctre/Phoenix.h"				// Core Phoenix 5 API
#include "ctre/phoenix/platform/Platform.hpp"		// Platform initialization for Linux CAN
#include "ctre/phoenix/unmanaged/Unmanaged.h"		// FeedEnable - required to keep Talons active on linux
#include <chrono>					// Timing
#include <iostream>					// Status messages
#include <thread>					// sleep

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

int main() {
    // On Jetson: onboard mttcan exposed as "can0"
    std::string interface = "can0";
    // Instantiate Talon SRX on CAN ID 1, bound to can0 interface
    TalonSRX talon{1, interface}; // device ID 1

    std::cout << "Spinning at 10% for 10 seconds...\n";

    // Record start time for 30 second run
    auto start = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
	// FeedEnable is required on Linux Phoenix 5 builds - without this call the Talon will enter 
	//neutral mode after 100ms as safety measure. Argument (100) is the timeout before neutral
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

	// Drive motor at 10% output - just above minimum working threshold
	// Range: -1.0 (full reverse) to +1.0 (full forward)
        talon.Set(ControlMode::PercentOutput, 0.10);

	// Sleep 30ms - keeps loop well within the 100ms FeedEnable Window
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    std::cout << "Stopping\n";
    auto stop_start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - stop_start < std::chrono::seconds(1)) {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
        talon.Set(ControlMode::PercentOutput, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    std::cout << "Done\n";
    return 0;
}
