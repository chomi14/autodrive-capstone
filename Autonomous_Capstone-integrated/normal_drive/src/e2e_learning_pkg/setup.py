from setuptools import setup,find_packages
setup(name='e2e_learning_pkg',version='0.1.0',packages=find_packages(),
    package_data={'e2e':['static/*','rosdefs/*.msg']},
    data_files=[('share/ament_index/resource_index/packages',['resource/e2e_learning_pkg']),('share/e2e_learning_pkg',['package.xml'])],
    install_requires=['setuptools'],entry_points={'console_scripts':['e2e=e2e.cli:main','e2e_shadow=e2e.shadow:main']})
