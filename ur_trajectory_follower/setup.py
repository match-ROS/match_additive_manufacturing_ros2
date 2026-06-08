from glob import glob
import os

from setuptools import setup

package_name = 'ur_trajectory_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosmatch',
    maintainer_email='rosmatch@todo.todo',
    description='UR path direction controller (ROS 2 port).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_direction_controller = ur_trajectory_follower.ur_path_direction_controller:main',
            'combine_twists = ur_trajectory_follower.combine_twists:main',
            'current_pose_from_tf = ur_trajectory_follower.current_pose_from_tf:main',
            'increment_path_index = ur_trajectory_follower.increment_path_index:main',
            'pid_twist_controller = ur_trajectory_follower.pid_twist_controller:main',
            'publish_joint_pose_once = ur_trajectory_follower.publish_joint_pose_once:main',
            'publish_sideways_test_path = ur_trajectory_follower.publish_sideways_test_path:main',
            'ur_orientation_controller = ur_trajectory_follower.ur_path_orientation_controller:main',
        ],
    },
)
