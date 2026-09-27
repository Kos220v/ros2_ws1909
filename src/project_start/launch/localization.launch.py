"""Counter/IMU local odometry plus a GNSS EKF; unique owners of dynamic TF."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration('localization_params')
    return LaunchDescription([
        DeclareLaunchArgument('localization_params', default_value=os.path.join(
            get_package_share_directory('project_start'), 'config', 'localization.yaml')),
        Node(package='project_start', executable='counter_odometry', name='counter_odometry',
             parameters=[params], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_global',
             parameters=[params], remappings=[('odometry/filtered', '/odometry/global')],
             output='screen'),
        Node(package='robot_localization', executable='navsat_transform_node',
             name='navsat_transform', parameters=[params], output='screen',
             remappings=[('imu', '/imu/data'), ('gps/fix', '/gps/fix'),
                         ('odometry/filtered', '/odometry/global'),
                         ('odometry/gps', '/odometry/gps')]),
    ])
