from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # RViz config argument
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_components'),
            'rviz_config',
            'launch_part.rviz'
        ]),
        description='Path to the RViz config file'
    )

    # Container with composable components
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_components',
                plugin='my_components::PreApproach',
                name='pre_approach'
            ),
            ComposableNode(
                package='my_components',
                plugin='my_components::AttachServer',
                name='attach_server'
            ),
        ],
        output='screen'
    )

    # Standalone executable node
    tf_publisher_node = Node(
        package='attach_shelf',
        executable='laser_to_cart_tf_publisher',
        name='laser_to_cart_tf_publisher',
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        rviz_config_arg,
        container,
        tf_publisher_node,
        rviz_node
    ])