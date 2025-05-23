import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pick_place_exercise'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all world files.
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        # Include all urdf files.
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JdeRobot',
    maintainer_email='diego.martin.martin@gmail.com',
    description='PICK and PLACE exercise for JdeRobot Robotics Academy (ROS2 Humble and Gazebo Classic)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_pick = pick_place_exercise.simple_pick:main'
        ],
    },
)
