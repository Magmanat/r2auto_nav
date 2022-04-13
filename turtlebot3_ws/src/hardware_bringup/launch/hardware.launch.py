from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_bringup',
            namespace='',
            executable='nfc_pub',
            name='nfc_chip'
        ),
        Node(
            package='hardware_bringup',
            namespace='',
            executable='button_pub',
            name='keyboard_switch'
        ),
        Node(
            package='hardware_bringup',
            namespace='',
            executable='thermal_pub',
            name='thermal_cam'
        ),
        Node(
            package='hardware_bringup',
            namespace='',
            executable='firing',
            name='firing_system'
        ),
    ])
