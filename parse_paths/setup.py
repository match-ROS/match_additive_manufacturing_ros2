from glob import glob
import os

from setuptools import setup

package_name = 'parse_paths'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosmatch',
    maintainer_email='rosmatch@todo.todo',
    description='Path publisher utilities for additive manufacturing robot tests.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_path_generator = parse_paths.test_path_generator:main',
            'publish_sideways_arm_test_path = parse_paths.publish_sideways_arm_test_path:main',
            'publish_front_side_arm_base_paths = parse_paths.publish_front_side_arm_base_paths:main',
            'publish_robotnik_base_arm_paths = parse_paths.publish_robotnik_base_arm_paths:main',
        ],
    },
)
