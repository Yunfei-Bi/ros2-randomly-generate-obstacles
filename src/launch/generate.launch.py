#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 获取Gazebo和当前包的路径
    gazebo_ros_pkg_path = get_package_share_directory('gazebo_ros')
    src_pkg_path = get_package_share_directory('src')
    
    # 启动Gazebo（使用默认世界）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_path, 'launch', 'gazebo.launch.py')
        )
    )
    
    # 启动障碍物生成节点
    obstacle_node = Node(
        package='src',
        executable='generate',
        name='obstacle_manager',
        output='screen'
    )
    
    # 按顺序启动Gazebo和障碍物节点（不使用事件处理器，直接启动）
    return LaunchDescription([
        gazebo,
        obstacle_node,
    ])