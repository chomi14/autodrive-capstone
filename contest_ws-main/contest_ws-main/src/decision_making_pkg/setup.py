from setuptools import find_packages, setup

package_name = 'decision_making_pkg'

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
            'motion = decision_making_pkg.motion:main',
            'motion_mission = decision_making_pkg.motion_mission:main',
            'motion_parking = decision_making_pkg.motion_parking:main'
        ],
    },
)
