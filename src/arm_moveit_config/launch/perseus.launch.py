#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

def generate_launch_description():
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("my-robot-urdf", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    
    # Launch servo as standalone node or component
    launch_as_standalone_node = LaunchConfiguration("launch_as_standalone_node", default="true")
    
    # Robot description
    robot_description = moveit_config.robot_description
    
    # Load servo parameters using proper method
    servo_params = {
        "moveit_servo": ParameterBuilder("arm_moveit_config")
        .yaml("config/servo_parameters.yaml")
        .to_dict()
    }

    # Controller manager
    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager", 
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    # Spawn controllers
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "300"],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "300"],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # Servo as standalone node
    servo_node_standalone = Node(
        package="moveit_servo",
        executable="servo_node",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(launch_as_standalone_node),
    )

    # Servo as composable node (alternative)
    servo_container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        output="screen",
        condition=UnlessCondition(launch_as_standalone_node),
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,
        arm_controller,
        move_group_node,
        servo_node_standalone,  # Used when launch_as_standalone_node=true
        servo_container,        # Used when launch_as_standalone_node=false
    ])