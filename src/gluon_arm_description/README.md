# Gluon Arm Description

This package contains the URDF description and visualization configuration for the Gluon robotic arm. It allows users to visualize the arm in RViz and control its joints using the joint state publisher.

## Package Overview

The Gluon robotic arm is a 6-degree-of-freedom manipulator designed for educational and research purposes. This package provides:

- URDF model of the Gluon arm with accurate kinematic and inertial properties
- Mesh files for realistic 3D visualization
- RViz configuration for easy visualization
- ROS2 control descriptions for simulation and hardware integration
- Launch files for quick setup

## Features

- Complete 3D model with STL meshes for all links
- Properly defined joint limits and dynamics
- Inertial properties for realistic simulation
- ROS2 control interfaces for both simulation and real hardware
- Visualization configuration for RViz

## File Structure

```
gluon_arm_description/
├── config/
│   └── gluon.rviz          # RViz configuration file
├── launch/
│   └── display.launch.py   # Launch file for visualization
├── meshes/                 # STL mesh files for all links
│   ├── base_link.STL
│   ├── 1_Link.STL
│   ├── 2_Link.STL
│   ├── 3_Link.STL
│   ├── 4_Link.STL
│   ├── 5_Link.STL
│   └── 6_Link.STL
├── urdf/
│   ├── gluon.urdf.xacro    # Main URDF with robot description (single entry point for all configurations)
│   ├── gluon_robot.urdf.xacro # Robot physical description only (used by other xacro files)
│   └── gluon_ros2_control.xacro # ROS2 control configuration
└── README.md
```

## URDF Structure Explanation

The URDF files are organized in a modular way to allow easy switching between simulation and real hardware:

1. **gluon_robot.urdf.xacro** - Contains only the physical description of the robot (links and joints)
2. **gluon_ros2_control.xacro** - Contains the ros2_control definitions with conditional hardware selection
3. **gluon.urdf.xacro** - Main entry point that uses arguments to configure hardware (replaces separate sim/hardware files)

This modular approach allows using the same physical description for both simulation and real hardware while easily switching between different control configurations through command-line arguments.

## Dependencies

- ROS2 Humble or later
- robot_state_publisher
- joint_state_publisher_gui
- xacro
- rviz2

## Usage

To visualize the Gluon arm in RViz:

```bash
# Source your ROS2 workspace
source ~/study/ros/ugv_ddsm_ws/install/setup.bash

# Launch the display
ros2 launch gluon_arm_description display.launch.py
```

This will start:
1. robot_state_publisher node to publish the robot's state
2. joint_state_publisher_gui node to control the joints with a GUI
3. RViz2 with a pre-configured view of the arm

In RViz, you can:
- Use the joint sliders in the joint_state_publisher_gui window to move the arm
- View the TF frames to understand the coordinate systems
- Examine the visual and collision geometries

## URDF Usage Examples

The single URDF file supports multiple configurations through command-line arguments:

1. **Simulation mode (default)**:
   ```bash
   xacro gluon.urdf.xacro
   ```

2. **Real hardware mode**:
   ```bash
   xacro gluon.urdf.xacro use_mock_hardware:=false hardware_plugin:=gluon_hardware/GluonHardware
   ```

3. **Custom hardware**:
   ```bash
   xacro gluon.urdf.xacro use_mock_hardware:=false hardware_plugin:=your.hardware.plugin
   ```

## URDF Details

The URDF model includes:

- 6 revolute joints with appropriate limits:
  - axis_joint_1: ±2.44 rad
  - axis_joint_2: ±1.57 rad
  - axis_joint_3: ±2.44 rad
  - axis_joint_4: ±2.44 rad
  - axis_joint_5: -1.22 to 3.66 rad
  - axis_joint_6: ±22.16 rad (large range for continuous rotation)

- Inertial properties for all links
- Visual and collision geometries using STL meshes
- ROS2 control interfaces for integration with ros2_control framework

## ROS2 Control Integration

The package includes configurations for ros2_control:
- Simulation configuration using `mock_components/GenericSystem`
- Joint command and state interfaces for all joints
- Position control interfaces with defined limits

## Customization

To customize the visualization:

1. Modify the URDF file to change kinematic properties
2. Adjust joint limits in the URDF
3. Update the RViz configuration in `config/gluon.rviz`
4. Create new launch files for specific configurations

## Troubleshooting

If you encounter issues:

1. Ensure all dependencies are installed:
   ```bash
   sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-rviz2
   ```

2. Check that the meshes are in the correct location

3. Verify that the xacro files process correctly:
   ```bash
   ros2 run xacro xacro urdf/gluon.urdf.xacro
   ```

## License

This package is provided as part of the UGV DDSM project. Please check the package.xml file for specific licensing information.