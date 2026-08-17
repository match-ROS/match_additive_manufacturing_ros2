from glob import glob
import os

from setuptools import setup

package_name = 'move_to_path_idx'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosmatch',
    maintainer_email='rosmatch@todo.todo',
    description='Move a mobile base to a selected path index using an external robot pose.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_path_idx = move_to_path_idx.move_to_path_idx:main',
            'move_to_pose = move_to_path_idx.move_to_pose:main',
            'move_ur_to_path_idx = move_to_path_idx.move_ur_to_path_idx:main',
            'move_ur_ik_to_path_idx = move_to_path_idx.move_ur_ik_to_path_idx:main',
        ],
    },
)
