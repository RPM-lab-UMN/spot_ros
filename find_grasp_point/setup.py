from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["find_grasp_point_utils", "DINO"],
    package_dir={"": "scripts"}
)

setup(**d)
