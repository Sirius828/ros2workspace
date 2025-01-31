from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'arm_control'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='sirius@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = arm_control.joint_state_publisher:main',
            'target_pose_publisher = arm_control.target_pose_publisher:main',
            'moveit_ik_node = arm_control.moveit_ik_node:main',
            'target_pose_client = arm_control.target_pose_client:main',
            'arm_actuator = arm_control.arm_actuator:main',
        ],
    },
)