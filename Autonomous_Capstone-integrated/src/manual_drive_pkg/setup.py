from setuptools import find_packages, setup
package_name = 'manual_drive_pkg'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='skku', maintainer_email='noreply@example.com', description='WASD manual drive and labeled image capture', license='MIT',
    entry_points={'console_scripts': ['manual_drive_capture_node = manual_drive_pkg.manual_drive_capture_node:main']},
)
