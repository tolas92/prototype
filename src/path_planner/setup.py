import os
from glob import glob
from setuptools import setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
         (os.path.join('share',package_name,'config'),glob('config/*.yaml'))
    ],
    install_requires=['setuptools','slam_toolbox','navigation2','nav2_lifecycle_manager'],
    zip_safe=True,
    maintainer='tolasing',
    maintainer_email='tolasingganesh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
    'navigation_node=path_planner.test:main'
        ],
    },
)