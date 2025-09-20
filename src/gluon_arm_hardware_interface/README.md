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

## Building the Package

To build this package, navigate to your ROS 2 workspace and run:

```bash
cd /path/to/your/ros2/workspace
colcon build --packages-select gluon_arm_hardware_interface
```

## Running the Hardware Interface

The hardware interface is designed to work with the `gluon_arm_bringup` package. To run the Gluon arm with real hardware, use:

```bash
ros2 launch gluon_arm_bringup gluon_hardware.launch.py use_mock_hardware:=false hardware_plugin:=gluon_arm_hardware/GluonArmHardwareInterface
```

This command will:
1. Load the real hardware interface plugin
2. Start the controller manager
3. Launch the robot state publisher
4. Start the joint state broadcaster
5. Launch RViz for visualization

## Safety Considerations

For testing purposes, the current implementation puts all actuators in current mode with 0A current, which is a safe state that prevents unexpected movements. This ensures that the arm will not move when first powered on or when the hardware interface is activated.

## Testing the Interface

You can test the hardware interface with a simple joint trajectory command:

```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: ''
  },
  joint_names: ['axis_joint_1', 'axis_joint_2', 'axis_joint_3', 'axis_joint_4', 'axis_joint_5', 'axis_joint_6'],
  points: [
    {
      positions: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}" -1
```

Note that in the current safety configuration, this command will not cause actual movement. To enable position control, the write() function in the hardware interface needs to be modified.

## Monitoring Hardware Components

To check if the hardware interface is properly loaded, use:

```bash
ros2 control list_hardware_components
```

This should show the GluonArmHardwareInterface as an active system component.

To list active controllers:

```bash
ros2 control list_controllers
```

## Viewing Joint States

To monitor the current joint positions published by the hardware interface:

```bash
ros2 topic echo /joint_states
```