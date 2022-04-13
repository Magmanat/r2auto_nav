from setuptools import setup
import os
from glob import glob

package_name = 'hardware_bringup'
submodules = 'hardware_bringup/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_run = hardware_bringup.servorun:main',
            'nfc_pub = hardware_bringup.NFC_pub:main',
            'button_pub = hardware_bringup.buttondetect:main',
            'thermal_pub = hardware_bringup.thermal_publisher:main',
            'motor_run = hardware_bringup.motor:main',
            'firing = hardware_bringup.firing_listener:main',
        ],
    },
)
