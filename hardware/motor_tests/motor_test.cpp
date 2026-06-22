#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <chrono>
#include <iostream>
#include <thread>

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

int main() {
    std::string interface = "can0";
    TalonSRX talon{1, interface}; // device ID 1

    std::cout << "Spinning at 10% for 10 seconds...\n";
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
        talon.Set(ControlMode::PercentOutput, 0.10);
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
