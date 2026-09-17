from setuptools import find_packages, setup
package_name = 'sensor_bringup_pkg'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='skku', maintainer_email='noreply@example.com',
    description='Camera and RPLidar bringup nodes for SKKU autonomous vehicle', license='MIT',
    entry_points={'console_scripts': [
        'camera_publisher_node = sensor_bringup_pkg.camera_publisher_node:main',
        'lidar_publisher_node_v2 = sensor_bringup_pkg.lidar_publisher_node:main',
    ]},
)
