import os
from setuptools import find_packages, setup
from glob import glob
package_name = 'rover_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tora',
    maintainer_email='tiger.tora1210@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover = rover_controller.rover:main',
            'odom_calc = rover_controller.odom_calc:main',
            'wheel_ctl_serial_master= rover_controller.wheel_ctl_serial_master:main',
            'teleop_joy_control= rover_controller.teleop_joy_control:main',
        ],
    },
)
