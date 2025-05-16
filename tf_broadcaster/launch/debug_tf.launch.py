from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Set directory of the parameter file
    pkg_dir = get_package_share_directory('tf_broadcaster')
    PublishDebug_param_path = (os.path.join(pkg_dir, "config", "PublishDebug_param.yaml"))

    return LaunchDescription([

        Node(
            package='tf_broadcaster',
            executable='PublishDebug',
            name='PublishDebug',
            output='screen',
            parameters=[PublishDebug_param_path]
        )
    ])
