#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def load_yaml(package_name, file_path):
    """Load a YAML file from a package"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Failed to load YAML file {absolute_file_path}: {e}")
        return None

def generate_launch_description():
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("my-robot-urdf", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    
    # Load servo parameters
    servo_yaml = load_yaml("arm_moveit_config", "config/servo_parameters.yaml")
    if servo_yaml is None:
        raise RuntimeError("Cannot load servo parameters")
    
    # Extract servo parameters with correct namespace
    servo_params = {"moveit_servo": servo_yaml["servo"]["ros__parameters"]}
    
    # Get MoveIt configurations
    robot_description = moveit_config.robot_description
    robot_description_semantic = moveit_config.robot_description_semantic
    robot_description_kinematics = moveit_config.robot_description_kinematics
    planning_pipelines = moveit_config.planning_pipelines
    
    # ROS2 control configuration
    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )
    
    # Verify files exist
    if not os.path.exists(ros2_controllers_path):
        raise FileNotFoundError(f"ROS2 controllers config not found: {ros2_controllers_path}")
    
    nodes = [
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        
        # Controller Manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, ros2_controllers_path],
            output="screen",
            condition=IfCondition(LaunchConfiguration('start_controller_manager')),
        ),
        
        # Spawn Joint State Broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        
        # Spawn Arm Controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        
        # MoveIt Move Group
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                planning_pipelines,
                {"use_sim_time": False},
            ],
        ),
        
        # Servo Node (critical - fix for robot_description issue)
        Node(
            package="moveit_servo",
            executable="servo_node",
            output="screen",
            parameters=[
                # Servo-specific parameters first
                servo_params,
                # Then MoveIt parameters
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                # Ensure robot_description is available globally
                {"robot_description": robot_description['robot_description']},
            ],
            # Add remappings if needed
            remappings=[
                ('/servo_node/delta_twist_cmds', '/servo_node/delta_twist_cmds'),
                ('/servo_node/delta_joint_cmds', '/servo_node/delta_joint_cmds'),
                ('/arm_controller/joint_trajectory', '/arm_controller/joint_trajectory'),
            ]
        ),
    ]

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'start_controller_manager',
            default_value='true',
            description='Start the controller manager'
        ),
        *nodes
    ])