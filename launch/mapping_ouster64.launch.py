import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('s_faster_lio')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2'
    )

    mapping_node = Node(
        package='s_faster_lio',
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ouster64.yaml'),
            {
                'feature_extract_enable': False,
                'point_filter_num': 4,
                'max_iteration': 3,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_dir, 'rviz_cfg', 'loam_livox.rviz')],
    )

    return LaunchDescription([
        rviz_arg,
        mapping_node,
        rviz_node,
    ])
