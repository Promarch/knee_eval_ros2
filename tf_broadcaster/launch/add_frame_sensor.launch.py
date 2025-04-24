from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np

def generate_launch_description():

    pkg_share = get_package_share_directory('tf_broadcaster')

    # Default path for the marker coordinates
    default_marker_path = os.path.join(pkg_share, 'config', 'marker_coordinates.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'marker_frame', 
            default_value='femur_body',
            description='Name of the marker screwed on the tibia'
        ),

        DeclareLaunchArgument(
            'target_frame',
            default_value='femur_ref',
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
        ), 

        DeclareLaunchArgument(
            "SensorInTibia_RPY", 
            default_value="0.053, -0.023, 0.076, 3.1415, 1.5708, 0", 
            description= "Transformation from tibia to sensor, described in tibia CoSy"
        ), 

        DeclareLaunchArgument(
            "KneeInTibia_RPY", 
            default_value="-0.150, 0, 0.155, 0, 0, 0", 
            description= "Transformation from tibia to knee, described in tibia CoSy"
        ),

        Node(
            package="tf_broadcaster", 
            executable="AddStaticTf", 
            name="AddStaticTf", 
            output="screen",
            parameters=[{
                "SensorInTibia_RPY": LaunchConfiguration("SensorInTibia_RPY"), 
                "KneeInTibia_RPY": LaunchConfiguration("KneeInTibia_RPY")
            }]
        )
    ])
