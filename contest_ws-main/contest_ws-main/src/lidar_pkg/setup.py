# src/lidar_pkg/setup.py

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 포함
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hun',
    maintainer_email='hun@todo.todo',
    description='Lidar processing package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    # <<< ❗ 여기가 가장 중요합니다 ❗ >>>
    entry_points={
        'console_scripts': [
            'lidar_publisher = lidar_pkg.lidar_publisher:main',
            'rear_parking_fsm = lidar_pkg.rear_parking_fsm:main',
        ],
    },
)
