from setuptools import find_packages, setup

package_name = 'control_pkg'

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
    maintainer='sg',
    maintainer_email='sg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_controller = control_pkg.keyboard_controller:main',
            'control = control_pkg.control:main',
            'parking_control = control_pkg.parking_control:main',
            'motion_command_guard = control_pkg.motion_command_guard:main',
        ],
    },
)
