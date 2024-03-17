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
        ('share/' + package_name, ['launch/run_all.launch.py']),
        ('share/' + package_name, ['launch/run_vslam.launch.py']),
        ('share/' + package_name, ['launch/run_detect.launch.py']),
        ('share/' + package_name, ['launch/run_apriltag.launch.py']),
    ],
    install_requires=['setuptools', 'pynetworktables'],
    zip_safe=True,
    maintainer='alistair',
    maintainer_email='alistair@keiller.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'integrated = comms_node.integrated:main',
            'apriltag_subscriber = comms_node.apriltag_subscriber:main',
            'detect_subscriber = comms_node.detect_subscriber:main',
            'vslam_subscriber = comms_node.vslam_subscriber:main',
            'save_map = comms_node.save_map:main',
        ],
    },
)
