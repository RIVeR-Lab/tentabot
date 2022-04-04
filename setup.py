## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['tentabot_heuristic', 'tentabot_drl', 'tentabot_il'],
    package_dir={'': 'scripts'},
)

setup(**setup_args)