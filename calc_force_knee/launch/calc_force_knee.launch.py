from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(): 
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument(
            "source_frame", 
            default_value = "sensor_ref",
            description = "Frame in which the forces are recorded"
        ),
        DeclareLaunchArgument(
            "target_frame", 
            default_value = "knee_ref", 
            description = "Frame in which the forces should be transformed"
        ),

        # Launch Node
        Node(
            package = "calc_force_knee", 
            executable = "CalcForceKnee", 
            name = "CalcForceKnee", 
            output = "screen", 
            parameters = [{
                "source_frame": LaunchConfiguration("source_frame"), 
                "target_frame": LaunchConfiguration("target_frame")
            }]
        )
    ])