import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(): 

    # Get the filepath of the parameters file
    calc_force_param_path = os.path.join(get_package_share_directory("calc_force_knee"), 'config', 'calc_force_param.yaml')
    return LaunchDescription([

        # Launch Node
        Node(
            package = "calc_force_knee", 
            executable = "CalcForceKnee", 
            name = "CalcForceKnee", 
            output = "screen", 
            parameters = [calc_force_param_path]
        )
    ])