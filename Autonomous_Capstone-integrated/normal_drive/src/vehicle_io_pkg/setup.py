from setuptools import find_packages, setup

package_name = 'vehicle_io_pkg'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skku',
    maintainer_email='noreply@example.com',
    description='Arduino serial bridge, startup steering calibration and drive arm gate',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_sender_node_v2 = vehicle_io_pkg.serial_sender_node:main',
            'drive_arm_node = vehicle_io_pkg.drive_arm_node:main',
        ]
    },
)
