# DDSM Mobile Robot Bringup Package

This package provides launch files and configurations for bringing up a 4-wheel differential drive mobile robot using DDSM drivers. It supports both simulation and real hardware operation.

## Features

- 4-wheel differential drive robot support
- ROS 2 Control integration with diff_drive_controller
- Simulation mode using mock hardware components
- Real hardware control via network interface
- RViz visualization for robot state and navigation
- Keyboard teleoperation support

## Package Structure

```
ddsm_mobile_bringup/
├── config/                 # Controller configurations
│   ├── simulation_control.yaml
│   └── rviz_config.rviz
├── launch/                 # Launch files
│   ├── display.launch.py
│   └── simulation.launch.py
├── urdf/                   # Robot description files
│   ├── simulation.urdf.xacro
│   └── ddsm_ros2_control.xacro
├── CMakeLists.txt
└── package.xml
```

## Usage

### Simulation Mode

To run the robot in simulation mode:

```bash
cd ~/study/ros/ugv_ddsm_ws
source install/setup.bash
ros2 launch ddsm_mobile_bringup simulation.launch.py
```

This will start:
- Robot State Publisher
- Controller Manager with mock hardware
- Joint State Broadcaster
- Diff Drive Controller
- RViz visualization

### Real Hardware Mode

To connect to real DDSM hardware, specify the IP address:

```bash
cd ~/study/ros/ugv_ddsm_ws
source install/setup.bash
ros2 launch ddsm_mobile_bringup display.launch.py ip_address:="192.168.1.100"
```

Replace `192.168.1.100` with your DDSM driver's actual IP address.

### Keyboard Teleoperation

In a separate terminal, start the keyboard teleoperation node:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

Keyboard controls:
- `i` - Move forward
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `q`/`z` - Increase/decrease speed
- `w`/`x` - Increase/decrease linear speed only
- `e`/`c` - Increase/decrease angular speed only

## Simulation vs. Real Hardware Configuration

### Simulation Mode

In simulation mode, the robot uses `mock_components/GenericSystem` with the `calculate_dynamics` parameter enabled to simulate joint movements:

```xml
<ros2_control name="DDSMRobot" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
    <param name="calculate_dynamics">true</param>
  </hardware>
  <!-- joint definitions -->
</ros2_control>
```

### Real Hardware Mode

When working with real hardware, a different configuration is used that connects to the actual DDSM driver hardware:

```xml
<ros2_control name="DDSMRobot" type="system">
  <hardware>
    <plugin>ddsm_mobile_hardware/DDSMHardware</plugin>
    <param name="ip_address">192.168.1.10</param>
  </hardware>
  <!-- joint definitions remain the same -->
</ros2_control>
```

**Important Notes for Real Hardware:**
- Do NOT use `calculate_dynamics=true` with real hardware
- The physical motors and encoders provide real feedback
- The actual robot hardware directly controls joint positions and velocities
- Using simulated dynamics with real hardware may cause conflicts or unpredictable behavior

## Simulation Specifics

When running in simulation mode:

1. **Hardware Interface**: Uses `mock_components/GenericSystem` with `calculate_dynamics` parameter enabled
2. **Coordinate Frames**: 
   - Fixed Frame in RViz should be set to `odom`
   - Robot base frame is `base_link`
3. **Controller Configuration**: 
   - Joint State Broadcaster publishes joint states
   - Diff Drive Controller handles differential drive kinematics
4. **TF Tree**: Complete transformation tree from `odom` to all wheel links

## Troubleshooting

### Robot Not Moving in Simulation

If the robot is not moving in RViz during simulation:

1. Check that joint positions are updating:
   ```bash
   ros2 topic echo /joint_states
   ```
   The `position` values should change when velocity commands are sent.

2. Verify controller status:
   ```bash
   ros2 control list_controllers
   ```
   Both `diff_drive_controller` and `joint_state_broadcaster` should be in `active` state.

3. Ensure proper topic mapping for teleoperation:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
   ```

### Real Hardware Issues

1. Ensure the DDSM driver is powered on and connected to the network
2. Verify the IP address is correct and reachable (use `ping`)
3. Check that all motors are properly connected and responding

## Dependencies

- ROS 2 Humble
- ros2_control
- ros2_controllers
- robot_state_publisher
- joint_state_publisher
- rviz2
- xacro
- teleop_twist_keyboard (for teleoperation)

Install dependencies:
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-teleop-twist-keyboard
```

## Building

To build the package:

```bash
cd ~/study/ros/ugv_ddsm_ws
colcon build --packages-select ddsm_mobile_bringup
```