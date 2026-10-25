"""Opt-in raw-image capture shared by the camera driving launches."""
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CAPTURE_ARGUMENT_NAMES = ('capture', 'capture_dir', 'capture_interval', 'capture_start_enabled')


def capture_arguments():
    return [
        DeclareLaunchArgument('capture', default_value='false', description='Enable R/C raw-camera capture controls'),
        DeclareLaunchArgument('capture_dir', default_value='~/autodrive_dataset/autonomous_drive'),
        DeclareLaunchArgument('capture_interval', default_value='0.20', description='Seconds between automatic JPEG captures'),
        DeclareLaunchArgument('capture_start_enabled', default_value='false', description='Start with auto capture ON; saving still requires accepted nonzero motion'),
    ]


def capture_node(image_topic, remappings=None):
    return Node(
        condition=IfCondition(LaunchConfiguration('capture')),
        package='manual_drive_pkg', executable='autonomous_capture_node',
        name='autonomous_capture_node', output='screen',
        remappings=remappings or [],
        parameters=[{
            'image_topic': image_topic,
            'output_dir': LaunchConfiguration('capture_dir'),
            'auto_capture_interval': ParameterValue(LaunchConfiguration('capture_interval'), value_type=float),
            'start_enabled': ParameterValue(LaunchConfiguration('capture_start_enabled'), value_type=bool),
        }],
    )
