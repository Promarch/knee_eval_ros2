from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('tf_broadcaster')

    # Default path for the marker coordinates
    default_marker_path = os.path.join(pkg_share, 'config', 'marker_coordinates.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'marker_frame', 
            default_value='tibia_body',
            description='Name of the marker screwed on the tibia'
        ),

        DeclareLaunchArgument(
            'target_frame',
            default_value='tibia_ref',
            description='Reference values for the individual marker points'
        ),

        DeclareLaunchArgument(
            'marker_file_path',
            default_value=default_marker_path,
            description='Path to the marker coordinates'
        ),

        Node(
            package='tf_broadcaster',
            executable='AddFrameSensor',
            name='AddFrameSensor',
            output='screen',
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame'), 
                'marker_frame': LaunchConfiguration('marker_frame'),
                'marker_file_path': LaunchConfiguration('marker_file_path')
            }]
        )
    ])
