from setuptools import setup
import os
from glob import glob

package_name = 'dummy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安装 urdf 文件夹
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        # 安装 meshes 文件夹（包括 visual 和 collision）
        ('share/' + package_name + '/meshes/visual', glob('meshes/visual/*.STL')),
        ('share/' + package_name + '/meshes/collision', glob('meshes/collision/*.STL')),
        # 安装到 ROS 2 包索引
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sirius',
    maintainer_email='sirius@todo.todo',
    description='Dummy package for URDF using ament_python',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)