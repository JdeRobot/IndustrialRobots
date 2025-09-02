from setuptools import find_packages, setup
from glob import glob

package_name = 'machine_vision_exercise'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')), 
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shu',
    maintainer_email='shu.xiao@students.iaac.net',
    description='Vision exercise',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # expose: ros2 run machine_vision_exercise my_algorithm
            'my_algorithm = machine_vision_exercise.my_algorithm:main',
        ],
    },
)
