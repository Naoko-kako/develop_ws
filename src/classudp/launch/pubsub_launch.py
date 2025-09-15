from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='classudp',
            executable='catch_publisher',
        ),
        Node(
            package='classudp',
            executable='catch_subscriber',
        ),
        Node(
            package='classudp',
            executable='my_turtle_controller',
        ),
        ])