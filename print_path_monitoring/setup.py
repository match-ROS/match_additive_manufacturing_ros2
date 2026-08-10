from glob import glob
import os

from setuptools import setup

package_name = 'print_path_monitoring'

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
    description='Monitoring-only diagnostics for print path and nozzle pose tracking.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nozzle_pose_monitor = print_path_monitoring.nozzle_pose_monitor:main',
            'trajectory_accuracy_monitor = print_path_monitoring.trajectory_accuracy_monitor:main',
            'trajectory_accuracy_report = print_path_monitoring.accuracy_report:main',
            'contour_profile_monitor = print_path_monitoring.contour_profile_monitor:main',
            'contour_correction = print_path_monitoring.contour_correction:main',
        ],
    },
)
