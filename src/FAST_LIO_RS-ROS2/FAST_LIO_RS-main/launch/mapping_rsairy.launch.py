import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    # visual_tf = LaunchConfiguration('visual_tf') # 新增坐标变换

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='FAST-LIO config directory'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file'
    )
    # 新增坐标变换
    # declare_visual_tf_cmd = DeclareLaunchArgument(
    #     'visual_tf', default_value='true',
    #     description='Publish camera_init_lidar -> camera_init transform for RViz'
    # )
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, 'rsairy.yaml']),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )
    # 新增坐标变换
    # visual_tf_node = Node(
    #     package='fast_lio',
    #     executable='camera_init_visual_tf',
    #     name='camera_init_visual_tf',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'parent_frame': 'camera_init_lidar',
    #         'child_frame': 'camera_init',
    #         'roll': 3.141592653589793,
    #         'pitch': 0.0,
    #         'yaw': -1.570796326794897,
    #     }],
    #     condition=IfCondition(visual_tf),
    #     output='screen'
    # )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    # ld.add_action(declare_visual_tf_cmd) # 新增坐标变换
    ld.add_action(fast_lio_node)
    # ld.add_action(visual_tf_node) # 新增坐标变换
    ld.add_action(rviz_node)

    return ld
