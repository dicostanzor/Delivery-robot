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

void run_motor(TalonSRX& talon, float percent, int seconds) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
        talon.Set(ControlMode::PercentOutput, percent);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    std::string interface = "can0";
    TalonSRX talon{1, interface}; // device ID 1

    std::cout << "Forward 20% for 3 seconds...\n";
    run_motor(talon, 0.20, 3);

    std::cout << "Stopping for 2 seconds...\n";
    run_motor(talon, 0.0, 2);

    std::cout << "Reverse 20% for 3 seconds...\n";
    run_motor(talon, -0.20, 3);

    std::cout << "Stopping.\n";
    run_motor(talon, 0.0, 2);

    std::cout << "Done\n";
    return 0;
}
