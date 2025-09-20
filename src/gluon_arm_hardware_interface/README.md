# Gluon Arm Hardware Interface

This package provides the hardware interface for controlling the Gluon robotic arm actuators. It includes a C++ SDK for communicating with the actuators over UDP and a ROS 2 hardware interface implementation.

## Overview

The Gluon Arm Hardware Interface package enables control of the Gluon robotic arm by providing:

1. A C++ SDK for direct actuator control
2. A ROS 2 hardware interface that integrates the arm with the ROS 2 control framework
3. Example programs demonstrating usage

## Main Features

- UDP communication with actuators
- Support for multiple actuators
- Implementation of various control modes (position, speed, current/torque)
- ROS 2 hardware interface integration
- Error handling and diagnostics

## Package Structure

```
gluon_arm_hardware_interface/
├── include/
│   └── gluon_arm_hardware_interface/
│       ├── actuator_ne30_sdk.hpp
│       ├── actuator_ne30_sdk.cpp
│       └── gluon_arm_hardware.hpp
├── src/
│   ├── gluon_arm_hardware_interface.cpp
│   └── test_actuator.cpp
├── CRC_mINTASCA.py
├── package.xml
└── README.md
```

## SDK Functions

The SDK provides functions for:

- Connecting to actuators
- Reading actuator status
- Controlling actuator operation modes
- Setting target positions, speeds, and currents
- Error handling

## Communication Protocol

The actuators communicate using a specific protocol with the following command structure:

```
Frame Header | Address | Command | Data Length | Data | CRC | Frame Tail
```

Where:
- Frame Header: 0xEE
- Address: Actuator ID (0x01-0xFF)
- Command: Command code
- Data Length: Length of data field (2 bytes, big endian)
- Data: Command-specific data
- CRC: 2-byte checksum (big endian)
- Frame Tail: 0xED

### Common Commands

| Function | Command Frame |
|----------|---------------|
| Handshake | `EE 00 44 00 00 ED` |
| Query Actuators | `EE 00 02 00 00 ED` |
| Enable Actuator (ID=0x05) | `EE 05 2A 00 01 01 7E 80 ED` |
| Switch to Current Mode (ID=0x05) | `EE 05 11 00 01 01 7E 80 ED` |
| Set Target Current (ID=0x05) | `EE 05 0A 00 04 00 00 00 00 00 24 ED` |
| Read Current Position (ID=0x05) | `EE 05 06 00 00 ED` |
| Disable Actuator (ID=0x05) | `EE 05 2A 00 01 00 BF 40 ED` |

## Usage

### Building the Package

```bash
cd ~/study/ros/ugv_ddsm_ws
colcon build --packages-select gluon_arm_hardware_interface
```

### Running the Test Program

After building, you can run the test program:

```bash
source install/setup.bash
ros2 run gluon_arm_hardware_interface test_actuator
```

This will:
1. Establish connection with the actuator controller
2. Query available actuators
3. Enable actuator ID 5
4. Switch it to current control mode
5. Set target current to 0A
6. Read the actuator's current position
7. Disable the actuator and exit

## Configuration

The SDK connects to the actuator controller at IP address `192.168.1.30` on port `2000` by default. These can be modified in the SDK source code if needed.

## Dependencies

- ROS 2 (Foxy or later)
- Standard C++ libraries
- CMake build system

## Troubleshooting

If you encounter connection issues:
1. Verify the controller is powered on and connected to the network
2. Check that the IP address and port in the SDK match your controller settings
3. Ensure your computer can reach the controller (try `ping 192.168.1.30`)
4. Confirm that no firewall is blocking UDP traffic on port 2000