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
    TalonSRX fl{1, interface}; // front left
    TalonSRX fr{2, interface}; // front right
    TalonSRX rl{3, interface}; // rear left
    TalonSRX rr{4, interface}; // rear right

    // 1&2 same direction, 3&4 same direction
    fl.SetInverted(false);
    fr.SetInverted(false);
    rl.SetInverted(true);
    rr.SetInverted(true);

    auto run = [&](double percent, int seconds) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
            fl.Set(ControlMode::PercentOutput, percent);
            fr.Set(ControlMode::PercentOutput, percent);
            rl.Set(ControlMode::PercentOutput, percent);
            rr.Set(ControlMode::PercentOutput, percent);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    std::cout << "Forward 5 seconds...\n";
    run(0.10, 5);

    std::cout << "Reverse 5 seconds...\n";
    run(-0.10, 5);

    std::cout << "Stopping\n";
    run(0.0, 1);

    std::cout << "Done\n";
    return 0;
}
