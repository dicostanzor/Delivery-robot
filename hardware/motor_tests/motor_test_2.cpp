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
    TalonSRX talon1{1, interface};
    TalonSRX talon2{2, interface};

    talon1.SetInverted(false);
    talon2.SetInverted(true);

    auto run = [&](double percent, int seconds) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
            talon1.Set(ControlMode::PercentOutput, percent);
            talon2.Set(ControlMode::PercentOutput, percent);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    std::cout << "Forward 15 seconds...\n";
    run(0.10, 15);

    std::cout << "Reverse 15 seconds...\n";
    run(-0.10, 15);

    std::cout << "Stopping\n";
    run(0.0, 1);

    std::cout << "Done\n";
    return 0;
}
