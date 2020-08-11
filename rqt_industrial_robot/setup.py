from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['rqt_industrial_robot'],
    package_dir={'': 'src'}
    # scripts=['scripts/rqt_kinematics', 'scripts/rqt_vacuum_gripper']
)

setup(**d)