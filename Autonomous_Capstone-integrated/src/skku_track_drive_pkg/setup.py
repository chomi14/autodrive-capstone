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
        (os.path.join('share', package_name, 'models/mission'), glob('models/mission/*.pt')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='skku', maintainer_email='noreply@example.com',
    description='ROS2 wrapper for 2026_skku_autodrive-final track driving pipeline', license='MIT',
    entry_points={'console_scripts': [
        'track_controller_node = skku_track_drive_pkg.track_controller_node:main',
        'track_tuner_node = skku_track_drive_pkg.track_tuner_node:main',
        'tuning_recorder_node = skku_track_drive_pkg.tuning_recorder_node:main',
        'tuning_bag_replay_node = skku_track_drive_pkg.tuning_bag_replay_node:main',
        'mission_controller_node = skku_track_drive_pkg.mission_controller_node:main',
        'parking_controller_node = skku_track_drive_pkg.parking_controller_node:main',
        'parking_calibration_controller_node = skku_track_drive_pkg.calibration_controller_node:controller_main',
        'parking_calibration_tuner_node = skku_track_drive_pkg.calibration_controller_node:tuner_main',
        'mission_tuner_node = skku_track_drive_pkg.mode_tuner_node:mission_main',
        'perpendicular_tuner_node = skku_track_drive_pkg.mode_tuner_node:perpendicular_main',
        'parallel_tuner_node = skku_track_drive_pkg.mode_tuner_node:parallel_main',
    ]},
)
