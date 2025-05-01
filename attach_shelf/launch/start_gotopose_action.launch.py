from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('attach_shelf'),
        'rviz',
        'launch_part.rviz'
    )

    return LaunchDescription([
        Node(
            package='attach_shelf',
            executable='go_to_pose_action',
            name='go_to_pose_action'
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_path],
        #     output='screen'
        # )
    ])
