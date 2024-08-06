from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['trajectory_planner_rqt'],
    package_dir={'': 'src'}
)

setup(**d)
