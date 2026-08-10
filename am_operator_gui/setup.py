from glob import glob
import os

from setuptools import setup

package_name = 'am_operator_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={
        package_name: ['web/templates/*', 'web/static/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'img'), glob('img/*')),
        (os.path.join('share', package_name, 'web', 'templates'), glob('am_operator_gui/web/templates/*')),
        (os.path.join('share', package_name, 'web', 'static'), glob('am_operator_gui/web/static/*')),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'jinja2'],
    zip_safe=True,
    maintainer='rosmatch',
    maintainer_email='rosmatch@todo.todo',
    description='PyQt5 operator GUI for additive manufacturing ROS 2 demos.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'am_operator_gui = am_operator_gui.main:main',
            'am_operator_web = am_operator_gui.web_main:main',
            'controller_switch_guard = am_operator_gui.controller_switch_guard:main',
            'external_base_reference = am_operator_gui.external_base_reference:main',
            'mur_arm_readiness = am_operator_gui.mur_arm_readiness:main',
            'odometry_robot_pose = am_operator_gui.odometry_robot_pose:main',
            'pose_stamped_adapter = am_operator_gui.pose_stamped_adapter:main',
            'vicon_tcp_robot_pose_backup = am_operator_gui.vicon_tcp_robot_pose_backup:main',
            'vicon_ee_static_tf = am_operator_gui.vicon_ee_static_tf:main',
        ],
    },
)
