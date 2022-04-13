from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_cartographer',
            namespace='auto_nav1',
            executable='cartographer.launch.py',
            name='auto_nav',
            output='screen',
        ),
        Node(
            package='auto_nav',
            namespace='auto_nav2',
            executable='map2base',
            name='auto_nav',
        ),
        Node(
            package='auto_nav',
            namespace='auto_nav3',
            executable='wall_follow',
            name='auto_nav',
        ),
    ])