from setuptools import setup
from glob import glob
import os
package_name = 'auto_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prince',
    maintainer_email='princemjp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2mover = auto_nav.r2mover:main',
            'r2moverotate = auto_nav.r2moverotate:main',
            'r2scanner = auto_nav.r2scanner:main',
            'r2occupancy = auto_nav.r2occupancy:main',
            'r2occupancy2 = auto_nav.r2occupancy2:main',
            'r2auto_nav = auto_nav.r2auto_nav:main',
            'r2auto_nav2 = auto_nav.r2auto_nav2:main',
            'r2auto_nav3_gazebo = auto_nav.r2auto_nav3_gazebo:main',
            'r2auto_nav3_real = auto_nav.r2auto_nav3_real:main',
            'map2base = auto_nav.map2base:main',
            'snake = auto_nav.r2_snake:main',
            'wall_follow = auto_nav.r2_wall_follow:main',
            'thermal_listener = auto_nav.thermal_listener:main',
            'thermal_interp = auto_nav.thermal_interp:main',
            'targeter = auto_nav.r2targeting1:main',
            'factory_test = auto_nav.r2_factory_test:main',
        ],
    },
)
