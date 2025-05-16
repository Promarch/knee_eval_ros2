from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np

def generate_launch_description():

    # Set directory of the parameter file
    pkg_dir = get_package_share_directory('tf_broadcaster')
    PublishStatic_param_path = (os.path.join(pkg_dir, "config", "PublishStatic_param.yaml"))

    # Default path for the marker coordinates
    default_marker_path = os.path.join(pkg_dir, 'config', 'marker_coordinates.yaml')

    return LaunchDescription([

        Node(
            package='tf_broadcaster',
            executable='PublishRefFrame',
            name='PublishRefFrame',
            output='screen',
            parameters=[{"marker_file_path":default_marker_path}]
        ), 

        Node(
            package="tf_broadcaster", 
            executable="PublishStaticTf", 
            name="PublishStaticTf", 
            output="screen",
            parameters=[PublishStatic_param_path]
        )
    ])
