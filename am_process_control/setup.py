from setuptools import find_packages, setup

package_name = 'am_process_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'process_safety_node = am_process_control.process_safety_node:main',
        'flow_serial_bridge = am_process_control.flow_serial_bridge:main',
        'dynamixel_valve_adapter = am_process_control.dynamixel_valve_adapter:main',
    ]},
)
