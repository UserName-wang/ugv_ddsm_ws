from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="gluon.urdf.xacro",
            description="URDF description file with the robot.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="mock_components/GenericSystem",
            description="Hardware plugin to use.",
        )
    )

    # Initialize Arguments
    description_file = LaunchConfiguration("description_file")
    hardware_plugin = LaunchConfiguration("hardware_plugin")

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("gluon_arm_description"), "urdf", description_file]
                ),
                " ",
                "use_mock_hardware:=false",
                " ",
                "hardware_plugin:=",
                hardware_plugin
            ]
        ),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("gluon_arm_description"), "config", "gluon.rviz"]
    )
    ros_control_config = PathJoinSubstitution(
        [FindPackageShare("gluon_arm_bringup"), "config", "gluon_hardware_controllers.yaml"]
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros_control_config],
        output='screen'
    )
    
    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Spawn arm controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)