#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my-robot-urdf", package_name="arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    
    servo_params = {
        "moveit_servo": ParameterBuilder("arm_moveit_config")
        .yaml("config/servo_parameters.yaml")
        .to_dict()
    }
    
    nodes = [
        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit_config.robot_description],
            output="screen",
        ),
        
        # Controller manager  
        Node(
            package="controller_manager", 
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                os.path.join(get_package_share_directory("arm_moveit_config"), "config", "ros2_controllers.yaml"),
            ],
            output="screen",
        ),
        
        # Spawn controllers
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["arm_controller"], output="screen"),
        
        # Move group
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            output="screen",
        ),
        
        Node(
            package="moveit_servo",
            executable="servo_node",
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(nodes)