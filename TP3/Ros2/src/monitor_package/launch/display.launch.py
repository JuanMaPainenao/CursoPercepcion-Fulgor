import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('monitor_package')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot_2dof.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # 1) URDF -> arbol de TF a partir de /joint_states.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # 2) Puente hardware-in-the-loop: sensores del ESP32 -> /joint_states.
        Node(
            package='monitor_package',
            executable='sensor_bridge',
            output='screen',
        ),

        # 3) Visualizador.
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])