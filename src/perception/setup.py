# DO NOT USE
# python setup.py install

from distutils.core import setup

setup(
    version='0.0.1',
    scripts=['scripts/inference.py'],
    packages=['perception_lidar'],
    package_dir={'': 'src'}
)
