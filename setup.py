## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['robofleet_client'],
    package_dir={'': 'src'},
    scripts=['scripts/generate/generate_plugin_pkg.py'],
    requires=['rosmsg', 'rospkg']
)

setup(**setup_args)