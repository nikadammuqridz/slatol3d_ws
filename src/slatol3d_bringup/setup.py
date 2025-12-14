import os
from glob import glob
from setuptools import setup

package_name = 'slatol3d_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=[
        # Install basic package metadata
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # FIX: Install ALL python launch files to the launch subdirectory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),

        # Install ALL world files from the worlds directory
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Launch files for 3DOF SLATOL Gazebo Fortress simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_control = slatol3d_bringup.test_control:main',
            'slatol_ui = slatol3d_bringup.slatol_ui:main',
            'slatol_planner = slatol3d_bringup.slatol_planner:main',
        ],
    },
)