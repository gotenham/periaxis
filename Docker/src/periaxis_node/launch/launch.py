from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='periaxis_node',
            executable='DMXL_read_write_node',
            name='DMXL_read_write_node',
            output='screen'
        ),
        Node(
            package='periaxis_node',
            executable='kinematic_solver',
            name='kinematic_solver',
            output='screen'
        ),
        Node(
            package='periaxis_node',
            executable='gait_controller',
            name='gait_controller',
            output='screen'
        )
        
    ])