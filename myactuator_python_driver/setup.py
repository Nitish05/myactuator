from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'myactuator_python_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rich',
        'PyQt6>=6.4.0',
        'qt-material>=2.14',
    ],
    zip_safe=True,
    maintainer='Nitish',
    maintainer_email='nitish@example.com',
    description='Python ROS2 driver for MyActuator RMD motors with TUI setup and recording/playback',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node = myactuator_python_driver.driver_node:main',
            'recorder_tui = myactuator_python_driver.recorder_tui:main',
            'setup_tui = myactuator_python_driver.setup_tui:main',
            'motor_studio = myactuator_python_driver.studio.main:main',
            'calibrator_cli = myactuator_python_driver.calibrator.main:main',
        ],
    },
)
