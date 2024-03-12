from setuptools import find_packages, setup

package_name = 'comms_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'log_pose = comms_node.log_pose_client:main',
            'network_tables = comms_node.network_tables_pose_client:main',
        ],
    },
)
