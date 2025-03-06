import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from xacro import process_file

def generate_launch_description():
    # Package path
    pkg_path = get_package_share_directory('periaxis_node')
    
    # Process/convert robot model description from XACRO to URDF
    xacro_file = os.path.join(pkg_path, 'urdf', 'periaxis_robot.xacro')
    robot_description = process_file(xacro_file).toxml()
    
    # Robot State Publisher Node (interprets robot model from converted .xacro and generates transforms for rviz2 based on received jointStates messages from kinematic_solver)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Periaxis C++ Nodes
    DMXL_read_write = Node(
        package='periaxis_node',
        executable='DMXL_read_write_node',
        name='DMXL_read_write_node',
        output='screen'
    )
    kinematic_solver = Node(
        package='periaxis_node',
        executable='kinematic_solver',
        name='kinematic_solver',
        output='screen'
    )
    gait_controller = Node(
        package='periaxis_node',
        executable='gait_controller',
        name='gait_controller',
        output='screen'
    )
    
    # Launch RViz2 Node from config file (renders robot visualisation based on robot_state_publisher)
    #rviz_config = os.path.join(pkg_path, 'config', 'rviz2_config.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        #arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        DMXL_read_write,
        kinematic_solver,
        gait_controller,
        rviz2
    ])