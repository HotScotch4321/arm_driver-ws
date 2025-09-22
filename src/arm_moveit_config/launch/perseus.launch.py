#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Build MoveIt configuration with OMPL planning pipeline
    moveit_config = (
        MoveItConfigsBuilder("my-robot-urdf", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    
    # Load servo parameters
    servo_params = {
        "moveit_servo": load_yaml("arm_moveit_config", "config/servo_parameters.yaml")
    }
    
    # Robot description
    robot_description = moveit_config.robot_description
    
    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,  # This provides OMPL configuration
            planning_scene_monitor_parameters,
            servo_params,
        ],
    )

    # Servo node for real-time control
    servo_node = ComposableNode(
        package="moveit_servo",
        plugin="moveit_servo::ServoNode",
        name="servo_node",
        parameters=[
            servo_params,
            robot_description,
            moveit_config.robot_description_semantic,
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Controller manager
    controller_manager_node = Node(
        package="controller_manager", 
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(get_package_share_directory("arm_moveit_config"), "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )

    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Arm controller
    arm_controller = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Container for servo components
    container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[servo_node],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager_node, 
        joint_state_broadcaster,
        arm_controller,
        move_group_node,
        container,
    ])