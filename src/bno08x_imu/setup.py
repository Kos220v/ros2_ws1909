from glob import glob
from setuptools import setup

setup(
    name='bno08x_imu', version='0.2.1', packages=['bno08x_imu'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/bno08x_imu']),
        ('share/bno08x_imu', ['package.xml', 'README.md']),
        ('share/bno08x_imu/launch', glob('launch/*.py')),
        ('share/bno08x_imu/config', glob('config/*.yaml') + glob('config/*.txt')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='admin', maintainer_email='admin@example.com', license='Apache-2.0',
    description='BNO085 SHTP over Linux I2C with fresh, quality-gated ROS IMU data',
    entry_points={'console_scripts': [
        'imu_node = bno08x_imu.node:main',
        'bno08x_diagnose = bno08x_imu.diagnose:main',
    ]},
)
