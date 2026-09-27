"""Mapless Nav2. Hardware and localization must be launched separately."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('project_start')
    params = LaunchConfiguration('nav2_params')
    specs = [('planner_server', 'nav2_planner', 'planner_server'),
             ('controller_server', 'nav2_controller', 'controller_server'),
             ('behavior_server', 'nav2_behaviors', 'behavior_server'),
             ('bt_navigator', 'nav2_bt_navigator', 'bt_navigator'),
             ('velocity_smoother', 'nav2_velocity_smoother', 'velocity_smoother'),
             ('collision_monitor', 'nav2_collision_monitor', 'collision_monitor')]
    actions = [DeclareLaunchArgument('nav2_params', default_value=os.path.join(
        share, 'config', 'nav2.yaml'))]
    for name, package, executable in specs:
        extra = {'use_sim_time': False, 'enable_stamped_cmd_vel': False}
        if name == 'bt_navigator':
            extra['default_nav_to_pose_bt_xml'] = os.path.join(
                share, 'behavior_trees', 'gps_navigation.xml')
        remaps = []
        if name in ('controller_server', 'behavior_server', 'velocity_smoother'):
            remaps = [('cmd_vel', '/cmd_vel/nav2_raw'),
                      ('cmd_vel_smoothed', '/cmd_vel/smoothed')]
        actions.append(Node(package=package, executable=executable, name=name,
                            output='screen', parameters=[params, extra], remappings=remaps))
    actions += [
        Node(package='project_start', executable='navigation_guard', name='navigation_guard',
             output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[{
                 'autostart': True, 'node_names': [s[0] for s in specs], 'bond_timeout': 4.0}]),
    ]
    return LaunchDescription(actions)
