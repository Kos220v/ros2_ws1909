"""Standalone BNO086 I2C bringup. Do not also run hardware start.launch.py."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('bno086_imu')
    args = [DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'imu.yaml')),
            DeclareLaunchArgument('gpio_chip', default_value='auto'),
            DeclareLaunchArgument('rst_gpio', default_value='17'),
            DeclareLaunchArgument('int_gpio', default_value='27'),
            DeclareLaunchArgument('i2c_bus', default_value='1'),
            DeclareLaunchArgument('i2c_address', default_value='75'),
            DeclareLaunchArgument('declination_deg', default_value='0.0')]
    node = Node(package='bno086_imu', executable='imu_node', name='bno086_imu', namespace='imu',
                output='screen', respawn=False,
                parameters=[LaunchConfiguration('params_file'), {
                    'gpio_chip': ParameterValue(LaunchConfiguration('gpio_chip'), value_type=str),
                    'rst_gpio': ParameterValue(LaunchConfiguration('rst_gpio'), value_type=int),
                    'int_gpio': ParameterValue(LaunchConfiguration('int_gpio'), value_type=int),
                    'i2c_bus': ParameterValue(LaunchConfiguration('i2c_bus'), value_type=int),
                    'i2c_address': ParameterValue(LaunchConfiguration('i2c_address'), value_type=int),
                    'declination_deg': ParameterValue(LaunchConfiguration('declination_deg'), value_type=float),
                }])
    return LaunchDescription(args + [node])
