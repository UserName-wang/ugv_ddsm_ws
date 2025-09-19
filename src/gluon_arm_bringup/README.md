# Gluon Arm Bringup Package

This package provides launch files and configurations for bringing up the Gluon 6-DOF robotic arm. It supports both simulation and real hardware operation using the ros2_control framework.

## Features

- Complete configuration for 6-DOF Gluon robotic arm
- Joint Trajectory Controller for precise motion control
- Joint State Broadcaster for publishing joint states
- RViz visualization for robot state
- Support for both simulation and real hardware (using mock components)
- Integration with ros2_control framework

## Package Structure

```
gluon_arm_bringup/
├── config/                    # Controller configurations
│   ├── gluon_controllers.yaml # Main controller configuration
│   ├── gluon_controllers_moveit.yaml # MoveIt configuration
│   ├── gluon_hardware_controllers.yaml # Hardware-specific controller configuration
│   └── gluon_simulation_controllers.yaml # Simulation-specific controller configuration
├── launch/                    # Launch files
│   ├── gluon_arm.launch.py    # Main launch file with mode selection
│   ├── gluon_hardware.launch.py # Hardware-specific launch file
│   └── gluon_simulation.launch.py # Simulation-specific launch file
├── README.md                  # This file
└── package.xml
```

## Usage

### Launch the Gluon Arm System in Simulation Mode (Default)

To launch the Gluon arm with visualization in RViz in simulation mode:

```bash
cd ~/study/ros/ugv_ddsm_ws
source install/setup.bash
ros2 launch gluon_arm_bringup gluon_arm.launch.py
```

This will start:
- Robot State Publisher
- Controller Manager with mock hardware
- Joint State Broadcaster
- Joint Trajectory Controller
- RViz visualization

### Launch the Gluon Arm System in Real Hardware Mode

To launch the Gluon arm with real hardware:

```bash
cd ~/study/ros/ugv_ddsm_ws
source install/setup.bash
ros2 launch gluon_arm_bringup gluon_arm.launch.py use_sim:=false
```

### Launch Specific Modes Directly

You can also launch specific modes directly:

Simulation mode:
```bash
ros2 launch gluon_arm_bringup gluon_simulation.launch.py
```

Hardware mode:
```bash
ros2 launch gluon_arm_bringup gluon_hardware.launch.py
```

### Control the Arm Using Command Line

You can control the arm by publishing to the joint trajectory topic:

```bash
# Send a simple trajectory command
ros2 topic pub -1 /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {
    stamp: {
      sec: 0,
      nanosec: 0
    },
    frame_id: ''
  },
  joint_names: ['axis_joint_1', 'axis_joint_2', 'axis_joint_3', 'axis_joint_4', 'axis_joint_5', 'axis_joint_6'],
  points: [
    {
      positions: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3],
      time_from_start: {
        sec: 1,
        nanosec: 0
      }
    }
  ]
}"
```

### Control the Arm Using Actions (Recommended)

The preferred method is to use the action interface:

```bash
# This requires a more complex action call, typically done through a Python script or MoveIt
# The action server is available at: /arm_controller/follow_joint_trajectory
```

### Check Running Controllers

To see the status of running controllers:

```bash
ros2 control list_controllers
```

### View Available Topics

To see all available topics:

```bash
ros2 topic list
```

## Controller Configuration

The main controllers configured in this package are:

1. **joint_state_broadcaster** - Publishes the state of all joints
2. **arm_controller** - Joint trajectory controller for motion control

Controller parameters can be modified in [config/gluon_controllers.yaml](file:///home/panda/study/ros/ugv_ddsm_ws/src/gluon_arm_bringup/config/gluon_controllers.yaml).

## Joint Names

The Gluon arm has 6 joints with the following names:
1. `axis_joint_1` (Base rotation)
2. `axis_joint_2` (Shoulder joint)
3. `axis_joint_3` (Elbow joint)
4. `axis_joint_4` (Wrist rotation)
5. `axis_joint_5` (Wrist bend)
6. `axis_joint_6` (End effector rotation)

## Joint Limits

Each joint has specific limits:
- axis_joint_1: ±2.44 rad
- axis_joint_2: ±1.57 rad
- axis_joint_3: ±2.44 rad
- axis_joint_4: ±2.44 rad
- axis_joint_5: -1.22 to 3.66 rad
- axis_joint_6: ±22.16 rad (continuous rotation)

## Dependencies

- ROS 2 Humble
- ros2_control
- ros2_controllers
- robot_state_publisher
- rviz2
- xacro
- gluon_arm_description

## Troubleshooting

### Controller Not Loading

If the arm_controller fails to load, check:
1. That all joint names in the controller configuration match those in the URDF
2. That parameter types are correct (e.g., floats vs integers)
3. That required command and state interfaces are properly defined

### Robot Not Moving

If the robot is not moving:
1. Check that the controller is in the `active` state:
   ```bash
   ros2 control list_controllers
   ```
2. Verify that the trajectory messages are being published to the correct topic
3. Check RViz for visualization updates

### Issues with Joint States

If joint states are not being published:
1. Ensure the joint_state_broadcaster is loaded and active
2. Check that the hardware interfaces are properly defined in the URDF