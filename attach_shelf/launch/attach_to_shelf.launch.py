from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    package_description = "attach_shelf"

    # args that can be set from the command line or a default will be used
    # TextSubstitution(text="0.0") W ill only evaluate that in execution time.

    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value=TextSubstitution(text="0.3")
    )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="-90.0"
    )
    rviz_config_file_name_arg = DeclareLaunchArgument(
        "rviz_config_file_name", default_value=TextSubstitution(text="launch_part.rviz")
    )
    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value=TextSubstitution(text="false")
    )

    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')

    rviz_config_file_name_f = LaunchConfiguration('rviz_config_file_name')
    final_approach_f = LaunchConfiguration('final_approach')

    # include another launch file
    # use items because you need to pass a list with a key-value structure
    # [(key1,value_x),(key2,value_y),...]

    start_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_description),
                'launch',
                'start_rviz_with_arguments.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz_config_file_name': rviz_config_file_name_f}.items()
    )

    pre_approach_node = Node(
        package='attach_shelf',
        executable='pre_approach_node',
        name='pre_approach',
        parameters=[
            {'obstacle': LaunchConfiguration('obstacle')},
            {'degrees': LaunchConfiguration('degrees')}
        ]
    )

    tf_node = Node(
        package='attach_shelf',
        executable='laser_to_cart_tf_publisher',
        name='laser_to_cart_tf_publisher',
        # parameters=[
        #     {'obstacle': LaunchConfiguration('obstacle')},
        #     {'degrees': LaunchConfiguration('degrees')}
        # ]
    )
    
    cart_frame_chase_node = Node(
        package='attach_shelf',
        executable='robot_chase',
        name='robot_chase',
                parameters=[
            {'final_approach': LaunchConfiguration('final_approach')}
        ]
    )
    
    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        rviz_config_file_name_arg,
        final_approach_arg,
        start_rviz_launch,
        pre_approach_node,
        tf_node,
        cart_frame_chase_node
    ])