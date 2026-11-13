from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'vehicle_bringup_pkg'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='skku', maintainer_email='noreply@example.com', description='Vehicle launch files', license='MIT',
)
