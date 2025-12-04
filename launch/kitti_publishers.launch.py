from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare('ros2_kitti_publishers').find('ros2_kitti_publishers')
    
    # Default config file path
    default_config_file = os.path.join(pkg_share, 'config', 'kitti_publishers_params.yaml')
    
    # Launch arguments
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_base_path',
        default_value='',
        description='Base path to KITTI dataset directory (REQUIRED)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='ROS2 frame ID for all published messages'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the YAML configuration file'
    )
    
    # Rviz2 config file path
    default_rviz_file = os.path.join(pkg_share, 'config', 'kitti_publishers.rviz')
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_file,
        description='Path to RViz2 configuration file'
    )
    
    # KITTI Publishers Node
    kitti_publishers_node = Node(
        package='ros2_kitti_publishers',
        executable='kitti_publishers',
        name='kitti_publishers',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'dataset_base_path': LaunchConfiguration('dataset_base_path'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
        output='screen'
    )
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        dataset_path_arg,
        frame_id_arg,
        publish_rate_arg,
        config_file_arg,
        use_rviz_arg,
        rviz_config_arg,
        kitti_publishers_node,
        rviz_node,
    ])

