# Motor Test Binaries

These are Phoenix 5 C++ test programs for the Talon SRX motor controllers.
They require the Phoenix 5 Linux libraries from ~/Phoenix5-Linux-Example/.

## Build

Copy source files into ~/Phoenix5-Linux-Example/, add entries to CMakeLists.txt, then build:

cd ~/Phoenix5-Linux-Example
cmake --build build/

## Run

Always run from ~/Phoenix5-Linux-Example/bin/ as:

cd ~/Phoenix5-Linux-Example/bin
sudo LD_LIBRARY_PATH=. ./[binary_name]

## Programs

| Binary | Description |
|---|---|
| motor_test | Single motor (ID 1), 30s run — used to bring up Phoenix diagnostic server for Tuner X connection |
| motor_test_2 | Two-motor test (IDs 1–2) |
| motor_test_4 | All four motors (IDs 1–4) |
| direction_test | Single motor (ID 1) — runs forward 20%, stops, reverses 20%, stops |

## Notes

- Minimum working percent output: 10%
- IDs 1 & 2: SetInverted(false), IDs 3 & 4: SetInverted(true)
- CAN bus must be up before running: sudo systemctl start can0-setup.service
- If motors don't respond, check for BUS-OFF: ip link show can0
