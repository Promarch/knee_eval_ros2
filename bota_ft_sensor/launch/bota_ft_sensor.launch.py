from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Get the filepath of the parameters file
    ft_sensor_param_path = os.path.join(get_package_share_directory("bota_ft_sensor"), 'config', 'ft_sensor_param.yaml')
    return LaunchDescription([
        
        Node(
            package='bota_ft_sensor',
            executable='bota_ft_sensor_node',
            name='bota_ft_sensor',
            output='screen',
            parameters=[ft_sensor_param_path]
        )
    ])
