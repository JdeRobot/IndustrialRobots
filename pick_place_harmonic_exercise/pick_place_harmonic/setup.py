import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pick_place_harmonic'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JdeRobot',
    maintainer_email='diego.martin.martin@gmail.com',
    description='PICK and PLACE exercise for JdeRobot Robotics Academy (ROS2 Humble and Gazebo Harmonic)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_pick_place = pick_place_harmonic.test_pick_place:main'
        ],
    },
)
