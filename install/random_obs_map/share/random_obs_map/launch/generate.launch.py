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
    random_obs_map_pkg_path = get_package_share_directory('random_obs_map')

    # 指定你自己的.world文件路径
    world_file = os.path.join(random_obs_map_pkg_path, 'worlds', 'myworld.world')  # 替换为你的.world文件名
    print(f"World file path: {world_file}")  # 打印路径确认
    
    # 检查文件是否存在
    if not os.path.isfile(world_file):
        print(f"World file {world_file} does not exist!")
        return LaunchDescription([])
    
    # 启动Gazebo并指定使用你自己的.world文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_path, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # 启动障碍物生成节点
    obstacle_node = Node(
        package='random_obs_map',
        executable='generate',
        name='obstacle_manager',
        output='screen'
    )
    
    # 按顺序启动Gazebo和障碍物节点（不使用事件处理器，直接启动）
    return LaunchDescription([
        gazebo,
        obstacle_node,
    ])