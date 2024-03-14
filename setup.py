import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'comms_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/run_logger.launch.py']),
        ('share/' + package_name, ['launch/run_network_tables.launch.py']),
        ('share/' + package_name, ['launch/run_detect.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alistair',
    maintainer_email='alistair@keiller.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'log_subscriber = comms_node.log_pose_subscriber:main',
            'network_tables_subscriber = comms_node.network_tables_subscriber:main',
        ],
    },
)
