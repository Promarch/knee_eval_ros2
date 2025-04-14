from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the Bota FT sensor'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='ft_sensor_link',
            description='TF frame ID for the FT sensor measurements'
        ),
        
        DeclareLaunchArgument(
            'publish_intervall',
            default_value='10.0',
            description='Intervall between two readings (ms)'
        ),

        DeclareLaunchArgument(
            'zero_samples',
            default_value='100',
            description='Amount of samples taken to zero out the sensor'
        ),

        DeclareLaunchArgument(
            'zero_timeout_ms',
            default_value='5000',
            description='Maximum allowed time to gather the defined amount of samples (ms)'
        ),
        
        Node(
            package='bota_ft_sensor',
            executable='bota_ft_sensor_node',
            name='bota_ft_sensor',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate')
            }]
        )
    ])
