import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    package_share = get_package_share_directory('ur5e_toolheads')

    xacro_file = os.path.join(
        package_share,
        'urdf',
        'ur5e_with_probe.urdf.xacro'
    )

    robot_description = {
        'robot_description': Command([
            'xacro ',
            xacro_file
        ])
    }

    return LaunchDescription([

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[robot_description],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[robot_description],
            arguments=['-f', 'world'],
            output='screen'
        ),
    ])