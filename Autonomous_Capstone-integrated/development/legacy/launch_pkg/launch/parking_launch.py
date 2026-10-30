from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_position = LaunchConfiguration("start_position")
    obstacle_layout = LaunchConfiguration("obstacle_layout")
    mirror_direction = LaunchConfiguration("mirror_direction")
    park_speed = LaunchConfiguration("park_speed")
    out_speed = LaunchConfiguration("out_speed")
    steer_max = LaunchConfiguration("steer_max")
    reverse_turn_time = LaunchConfiguration("reverse_turn_time")
    reverse_counter_time = LaunchConfiguration("reverse_counter_time")
    reverse_straight_time = LaunchConfiguration("reverse_straight_time")
    stop_time = LaunchConfiguration("stop_time")
    forward_turn_time = LaunchConfiguration("forward_turn_time")
    forward_straight_time = LaunchConfiguration("forward_straight_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_position",
            default_value="1",
            description="Parking mission start position: 1, 2, 3, or 4.",
        ),
        DeclareLaunchArgument(
            "obstacle_layout",
            default_value="1",
            description="Parking obstacle layout: 1 or 2.",
        ),
        DeclareLaunchArgument(
            "mirror_direction",
            default_value="0",
            description="0: auto by start_position, 1: normal, -1: mirrored.",
        ),
        DeclareLaunchArgument("park_speed", default_value="90"),
        DeclareLaunchArgument("out_speed", default_value="100"),
        DeclareLaunchArgument("steer_max", default_value="6"),
        DeclareLaunchArgument("reverse_turn_time", default_value="3.35"),
        DeclareLaunchArgument("reverse_counter_time", default_value="5.15"),
        DeclareLaunchArgument("reverse_straight_time", default_value="7.75"),
        DeclareLaunchArgument("stop_time", default_value="4.0"),
        DeclareLaunchArgument("forward_turn_time", default_value="6.20"),
        DeclareLaunchArgument("forward_straight_time", default_value="1.35"),

        Node(
            package="decision_making_pkg",
            executable="parking_mission_node",
            name="parking_mission_node",
            output="screen",
            parameters=[{
                "start_position": ParameterValue(start_position, value_type=int),
                "obstacle_layout": ParameterValue(obstacle_layout, value_type=int),
                "mirror_direction": ParameterValue(mirror_direction, value_type=int),
                "park_speed": ParameterValue(park_speed, value_type=int),
                "out_speed": ParameterValue(out_speed, value_type=int),
                "steer_max": ParameterValue(steer_max, value_type=int),
                "reverse_turn_time": ParameterValue(
                    reverse_turn_time, value_type=float),
                "reverse_counter_time": ParameterValue(
                    reverse_counter_time, value_type=float),
                "reverse_straight_time": ParameterValue(
                    reverse_straight_time, value_type=float),
                "stop_time": ParameterValue(stop_time, value_type=float),
                "forward_turn_time": ParameterValue(
                    forward_turn_time, value_type=float),
                "forward_straight_time": ParameterValue(
                    forward_straight_time, value_type=float),
            }],
        ),

        Node(
            package="serial_communication_pkg",
            executable="serial_sender_node",
            name="serial_sender_node",
            output="screen",
        ),
    ])
