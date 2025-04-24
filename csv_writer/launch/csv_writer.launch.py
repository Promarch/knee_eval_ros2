from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(): 
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument(
            "path_csv_file", 
            default_value = "data.config", 
            description = "Path to the config file"
        ),
        DeclareLaunchArgument(
            "source_frame", 
            default_value = "map",
            description = "Base Frame in which all coordinates are displayed"
        ),
        DeclareLaunchArgument(
            "tibia_frame", 
            default_value = "tibia_ref", 
            description = "Frame of the tibia"
        ),
        DeclareLaunchArgument(
            "femur_frame", 
            default_value = "femur_ref", 
            description = "Frame of the femur"
        ),
        DeclareLaunchArgument(
            "knee_frame", 
            default_value = "knee_ref", 
            description = "Frame of the knee"
        ),

        # Launch Node
        Node(
            package = "csv_writer", 
            executable = "CsvWriter", 
            name = "CsvWriter", 
            output = "screen", 
            parameters = [{
                "path_csv_file": LaunchConfiguration("path_csv_file"), 
                "source_frame": LaunchConfiguration("source_frame"), 
                "tibia_frame": LaunchConfiguration("tibia_frame"), 
                "femur_frame": LaunchConfiguration("femur_frame"), 
                "knee_frame": LaunchConfiguration("knee_frame")
            }]
        )
    ])