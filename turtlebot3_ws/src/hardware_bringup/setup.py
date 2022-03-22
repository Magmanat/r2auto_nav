from setuptools import setup

package_name = 'hardware_bringup'
submodules = 'hardware_bringup/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
