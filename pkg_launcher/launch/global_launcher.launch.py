from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directories for each package
    ft_sensor_dir = get_package_share_directory('bota_ft_sensor')
    optitrack_dir = get_package_share_directory('mocap_optitrack_driver')
    tf_broadcaster_dir = get_package_share_directory('tf_broadcaster')
    calc_force_knee_dir = get_package_share_directory('calc_force_knee')
    csv_writer_dir = get_package_share_directory('csv_writer')

    # First stage: Launch bota_ft_sensor and optitrack2
    bota_ft_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ft_sensor_dir, 'launch', 'bota_ft_sensor.launch.py')
        )
    )

    optitrack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(optitrack_dir, 'launch', 'optitrack2.launch.py')
        )
    )

    # Second stage: Launch add_frame_sensor after mocap_optitrack_driver_node starts
    add_frame_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tf_broadcaster_dir, 'launch', 'tf_broadcaster.launch.py')
        )
    )

    # Wait 3 seconds to ensure optitrack node is up and running
    add_frame_timer = TimerAction(
        period=3.0,
        actions=[add_frame_sensor_launch]
    )

    # Third stage: Launch calc_force_knee and csv_writer after add_frame_sensor
    # Adding a timer to ensure all the previous nodes have time to initialize
    calc_force_knee_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(calc_force_knee_dir, 'launch', 'calc_force_knee.launch.py')
        )
    )

    csv_writer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(csv_writer_dir, 'launch', 'csv_writer.launch.py')
        )
    )

    calc_force_timer = TimerAction(
        period=5.0,  # 5 second delay to ensure previous nodes are ready
        actions=[calc_force_knee_launch]
    )

    csv_timer = TimerAction(
        period=5.5,  # 3.5 second delay to ensure previous nodes are ready
        actions=[csv_writer_launch]
    )

    # Create and return launch description
    return LaunchDescription([
        bota_ft_sensor_launch,
        optitrack_launch,
        add_frame_timer,
        calc_force_timer,
        csv_timer
    ])
