from glob import glob
from setuptools import setup

setup(
    name='bno086_imu', version='0.1.0', packages=['bno086_imu'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/bno086_imu']),
        ('share/bno086_imu', ['package.xml', 'README.md']),
        ('share/bno086_imu/launch', glob('launch/*.py')),
        ('share/bno086_imu/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='admin', maintainer_email='admin@example.com', license='Apache-2.0',
    description='BNO086 SHTP over Linux I2C with fresh, quality-gated ROS IMU data',
    entry_points={'console_scripts': ['imu_node = bno086_imu.node:main']},
)
