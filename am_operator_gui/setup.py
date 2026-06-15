from glob import glob
import os

from setuptools import setup

package_name = 'am_operator_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosmatch',
    maintainer_email='rosmatch@todo.todo',
    description='PyQt5 operator GUI for additive manufacturing ROS 2 demos.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'am_operator_gui = am_operator_gui.main:main',
            'controller_switch_guard = am_operator_gui.controller_switch_guard:main',
            'pose_stamped_adapter = am_operator_gui.pose_stamped_adapter:main',
        ],
    },
)
