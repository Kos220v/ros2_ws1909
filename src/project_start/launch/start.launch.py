#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
start.launch.py — слой ЖЕЛЕЗА гусеничного робота (Raspberry Pi 5, Ubuntu 24.04,
ROS 2 Jazzy).

Распределение UART:
    uart0  /dev/ttyAMA0  — приёмник ELRS (пульт)
    I2C1  /dev/i2c-1    — BNO086 Qwiic (0x4B), GPIO2/GPIO3
    uart2  /dev/ttyAMA2  — GPS (NMEA)
    uart3  /dev/ttyAMA3  — VESC левый  (kolesa_control)
    uart4  /dev/ttyAMA4  — VESC правый (kolesa_control)
    Лидар                — USB (/dev/ttyUSB0)

Запускает:
    elrs_receiver        пульт ELRS       -> /cmd_vel/manual, /control_mode
    kolesa_control       2×VESC (FS75100) <- /cmd_vel, -> /odom/vesc (скорость)
    bno086_imu           BNO086 I2C        -> /imu/data (кватернион ENU, гироскоп)
    Локализация запускается отдельно: localization.launch.py (counter_odometry + GPS EKF).
    nmea_navsat_driver   GNSS             -> /gps/fix
    robot_state_publisher  URDF           -> статические TF base_link -> датчики
    cmd_switcher         приоритеты       -> /cmd_vel
    relay_reliable       /scan -> /scan_reliable
    ydlidar              лидар (USB)      -> /scan (с задержкой lidar_delay)

Аргументы:
    lidar_delay   задержка старта лидара, с (мотор вибрирует, IMU должна
                  успеть откалибровать гироскоп стоя)
    use_gps       запускать драйвер GNSS (false — стенд/помещение)
    gps_port, lidar_port, imu_i2c_bus, imu_i2c_address   переопределение устройств
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _first_existing(*paths):
    """Возвращает первый существующий путь или None."""
    for p in paths:
        if p and os.path.exists(p):
            return p
    return None


def launch_setup(context, *args, **kwargs):
    project_start_share = get_package_share_directory('project_start')
    imu_share = get_package_share_directory('bno086_imu')

    # ---------------------------------------------------------------- порты
    # Жёстко зафиксированные UART на Raspberry Pi 5 (см. config.txt / оверлеи)
    ELRS_PORT  = '/dev/ttyAMA0'   # uart0
    GPS_PORT   = '/dev/ttyAMA2'   # uart2
    VESC_LEFT  = '/dev/ttyAMA3'   # uart3
    VESC_RIGHT = '/dev/ttyAMA4'   # uart4
    # Лидар — USB-адаптер
    LIDAR_PORT = '/dev/ttyUSB0'

    # Возможность переопределения через аргументы launch
    lidar_port = LaunchConfiguration('lidar_port').perform(context) or \
        _first_existing(LIDAR_PORT, '/dev/ttyUSB1')
    gps_port = LaunchConfiguration('gps_port').perform(context) or \
        _first_existing(GPS_PORT)

    use_gps = LaunchConfiguration('use_gps').perform(context).lower() in ('1', 'true', 'yes')
    lidar_delay = float(LaunchConfiguration('lidar_delay').perform(context))

    if lidar_port is None:
        raise RuntimeError('Лидар не найден: нет /dev/ttyUSB0 '
                           '(задайте lidar_port:=...)')
    if use_gps and gps_port is None:
        raise RuntimeError('GNSS не найден: нет /dev/ttyAMA2 '
                           '(задайте gps_port:=... или use_gps:=false)')

    # ------------------------------------------------------------ пульт ELRS
    elrs_node = Node(
        package='elrs_receiver',
        executable='elrs_node',
        name='elrs_receiver',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'port': ELRS_PORT,
            'baudrate': 420000,
            'deadzone': 0.02,
            'throttle_channel': 1,
            'steering_channel': 0,
            'invert_throttle': False,
            'invert_steering': False,
            'channel_min': 172,
            'channel_center': 992,
            'channel_max': 1811,
        }],
    )

    # ------------------------------------------------------------------ GNSS
    gps_nodes = []
    if use_gps:
        gps_nodes.append(Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_navsat_driver',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                'port': gps_port,
                'baud': int(LaunchConfiguration('gps_baud').perform(context)),
                'frame_id': 'gps_link',
                'useRMC': False,
            }],
            remappings=[
                ('fix', 'gps/fix'),
                ('vel', 'gps/vel'),
                ('time_reference', 'gps/time_reference'),
                ('heading', 'gps/heading'),
            ],
        ))

    # --------------------------------------------------------------- приводы
    kolesa_control_node = Node(
        package='kolesa_control',
        executable='kolesa_control',
        name='kolesa_control',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'left_port': VESC_LEFT,      # /dev/ttyAMA3
            'right_port': VESC_RIGHT,    # /dev/ttyAMA4
            'baud': 115200,
            'wheel_separation': 0.48,
            # Калибровка одометрии VESC (см. kolesa_control/README.md)
            'tacho_counts_per_revolution': 2157.0,
            'distance_per_revolution': 2.011,
            'left_odometry_scale': 1.0,
            'right_odometry_scale': 1.0,
            'odometry_scale': 1.15,   # замер: рулетка 14.12 м / одометрия 12.28 м
            'invert_left': False,
            'invert_right': True,
            'encoder_invert_left': False,
            'encoder_invert_right': True,
            'invert_angular': False,
            'max_linear_velocity': 1.0,
            'max_angular_velocity': 1.0,
            'duty_min': 0.03,
            'duty_max': 0.6,
            'publish_odom': True,
            'odom_topic': 'odom/vesc',
            'publish_joint_states': True,
            'publish_diagnostics': True,
        }],
    )

    # BNO086: обязательные RST/INT; без respawn, reset меняет состояние курса.
    # ------------------------------------------------------- BNO086 по I2C
    imu_params = os.path.join(imu_share, 'config', 'imu.yaml')
    imu_node = Node(
        package='bno086_imu', executable='imu_node', name='bno086_imu',
        namespace='imu', output='screen', respawn=False,
        parameters=[imu_params, {
            'gpio_chip': ParameterValue(LaunchConfiguration('imu_gpio_chip'), value_type=str),
            'rst_gpio': ParameterValue(LaunchConfiguration('imu_rst_gpio'), value_type=int),
            'int_gpio': ParameterValue(LaunchConfiguration('imu_int_gpio'), value_type=int),
            'i2c_bus': ParameterValue(LaunchConfiguration('imu_i2c_bus'), value_type=int),
            'i2c_address': ParameterValue(LaunchConfiguration('imu_i2c_address'), value_type=int),
            'declination_deg': ParameterValue(LaunchConfiguration('declination_deg'), value_type=float),
        }],
    )

    # ------------------------------------------------------------- TF из URDF
    urdf_file = os.path.join(
        get_package_share_directory('tracked_robot_description'),
        'urdf', 'tracked_robot.urdf.xacro'
    )
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ------------------------------------------------- приоритеты команд
    cmd_mux_node = Node(
        package='cmd_switcher',
        executable='cmd_mux_node',
        name='cmd_switcher',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # ------------------------------------------------------------------ лидар
    ydlidar_params = os.path.join(project_start_share, 'params',
                                  'ydlidar_params.yaml')

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            ydlidar_params,
            {'port': lidar_port},      # /dev/ttyUSB0
        ],
    )
    lidar_delayed = TimerAction(period=lidar_delay, actions=[ydlidar_node])

    # /scan: RELIABLE-копия для Nav2 (костмапы), Wi-Fi и rosbridge
    relay_node = Node(
        package='relay_reliable',
        executable='relay_node',
        name='relay_reliable',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return [
        elrs_node,
        imu_node,
        kolesa_control_node,
        *gps_nodes,
        robot_state_publisher_node,
        cmd_mux_node,
        relay_node,
        lidar_delayed,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_delay', default_value='10.0',
                              description='Задержка старта лидара, с'),
        DeclareLaunchArgument('use_gps', default_value='true',
                              description='Запускать драйвер GNSS'),
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyAMA2',
                              description='Порт GNSS (по умолчанию /dev/ttyAMA2)'),
        DeclareLaunchArgument('gps_baud', default_value='115200'),
        DeclareLaunchArgument('imu_gpio_chip', default_value='auto'),
        DeclareLaunchArgument('imu_rst_gpio', default_value='17'),
        DeclareLaunchArgument('imu_int_gpio', default_value='27'),
        DeclareLaunchArgument('imu_i2c_bus', default_value='1',
                              description='Номер шины /dev/i2c-N для BNO086'),
        DeclareLaunchArgument('imu_i2c_address', default_value='75',
                              description='Адрес BNO086: 75=0x4B, 74=0x4A'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                              description='Порт лидара (USB, по умолчанию /dev/ttyUSB0)'),
        DeclareLaunchArgument('declination_deg', default_value='0.0',
                              description='Магнитное склонение, град (+ восточное)'),
        OpaqueFunction(function=launch_setup),
    ])
