# -*- coding: utf-8 -*-
"""Запуск ноды kolesa_control с параметрами.

Отредактируйте значения под свой робот и запустите:
    ros2 launch kolesa_control kolesa_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kolesa_control",
            executable="kolesa_control",
            name="kolesa_control",
            output="screen",
            parameters=[{
                # Порты (Raspberry Pi: uart3/uart4)
                "left_port": "/dev/ttyAMA3",
                "right_port": "/dev/ttyAMA4",
                "baud": 115200,

                # Колея, м — только для раскладки cmd_vel по бортам
                "wheel_separation": 0.48,

                # Калибровка одометрии VESC (README.md, «Калибровка одометрии»)
                "tacho_counts_per_revolution": 2157.0,
                "distance_per_revolution": 2.011,
                "odometry_scale": 1.0,
                "left_odometry_scale": 1.0,
                "right_odometry_scale": 1.0,

                # Направления
                "invert_left": False,
                "invert_right": True,
                "encoder_invert_left": False,
                "encoder_invert_right": True,
                "invert_angular": False,

                # Скважность VESC (управление разомкнутое — см. README.md)
                "duty_min": 0.03,
                "duty_max": 0.6,
                "max_linear_velocity": 1.0,
                "max_angular_velocity": 1.0,

                # Тайминги
                "control_rate": 50.0,
                "telemetry_rate": 20.0,
                "cmd_timeout": 0.5,
                "telemetry_stale_timeout": 0.5,
                "telemetry_pair_max_skew": 0.10,
                "tacho_jump_margin": 3.0,
                "min_tacho_jump_threshold": 10.0,

                # Публикации
                "publish_odom": True,          # /odom/vesc — скорость для robot_localization
                "odom_topic": "odom/vesc",
                "publish_joint_states": True,
                "publish_diagnostics": True,
            }],
        ),
    ])
