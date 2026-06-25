from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    bag_file = LaunchConfiguration('bag_file')
    transforms_from_csv = LaunchConfiguration('transforms_from_csv')
    csv_file = LaunchConfiguration('csv_file')
    output_pointcloud_path = LaunchConfiguration('output_pointcloud_path')
    output_calibration_path = LaunchConfiguration('output_calibration_path')

    return LaunchDescription([
        DeclareLaunchArgument('bag_file', default_value=''),
        DeclareLaunchArgument('transforms_from_csv', default_value='false'),
        DeclareLaunchArgument('csv_file', default_value=''),
        DeclareLaunchArgument(
            'output_pointcloud_path',
            default_value='/tmp/lidar_align_points.ply'),
        DeclareLaunchArgument(
            'output_calibration_path',
            default_value='/tmp/lidar_align_calibration.txt'),
        Node(
            package='lidar_align',
            executable='lidar_align_node',
            name='lidar_align',
            output='screen',
            parameters=[{
                'input_bag_path': bag_file,
                'input_csv_path': csv_file,
                'output_pointcloud_path': output_pointcloud_path,
                'output_calibration_path': output_calibration_path,
                'transforms_from_csv': ParameterValue(
                    transforms_from_csv, value_type=bool),
            }],
        ),
    ])
