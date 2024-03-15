#!/usr/bin/env python

from launch import LaunchDescription
from launch.launch_context import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    urdf_cfg_pth = os.path.join(get_package_share_directory("description"),
                                "urdf",
                                "main.urdf.xacro")

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters=[{
            "robot_description": os.popen("xacro " + urdf_cfg_pth).read()
        }]
    )

    rviz_cfg_pth = os.path.join(get_package_share_directory("bringup"),
                                "config",
                                "config.rviz")

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg_pth]
    )

    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description","-entity", "robot"]
    )

    gazebo_launch_world_pth = os.path.join(get_package_share_directory("bringup"),
                                           "worlds",
                                           "objects.world")

    gazebo_launch_file_pth = os.path.join(get_package_share_directory("gazebo_ros"),
                                          "launch",
                                          "gazebo.launch.py")

    gazebo_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file_pth),
        launch_arguments = {"world" : gazebo_launch_world_pth}.items())
    
    slam_toolbox_launch_file_pth = os.path.join(get_package_share_directory("slam_toolbox"),
                                                "launch",
                                                "online_async_launch.py")

    slam_param_pth = os.path.join(get_package_share_directory("bringup"),
                                  "config",
                                  "mapper_params_online_async.yaml")

    slam_toolbox_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file_pth),
        launch_arguments = {"slam_params_file" : slam_param_pth,
                            "use_sim_time" : "true"}.items())

    navigation_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'use_sim_time:=true'],
        output='screen'
    )

    return LaunchDescription([robot_state_publisher_node,
                              rviz2_node,
                              gazebo_spawn_entity_node,
                              gazebo_launch_desc,
                              slam_toolbox_launch_desc,
                              navigation_launch,
                              ])

if __name__ == "__main__":
    generate_launch_description()
