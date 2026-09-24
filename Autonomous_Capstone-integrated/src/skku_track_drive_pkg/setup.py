from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'skku_track_drive_pkg'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='skku', maintainer_email='noreply@example.com',
    description='ROS2 wrapper for 2026_skku_autodrive-final track driving pipeline', license='MIT',
    entry_points={'console_scripts': [
        'track_controller_node = skku_track_drive_pkg.track_controller_node:main',
        'track_tuner_node = skku_track_drive_pkg.track_tuner_node:main',
    ]},
)
