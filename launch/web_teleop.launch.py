from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params", description="parameter YAML file"),
        Node(package="failsafe_teleop", executable="web_teleop", name="web_teleop",
             parameters=[LaunchConfiguration("params")], output="screen"),
    ])
