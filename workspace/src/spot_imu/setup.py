from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
     packages=["spot_imu"],
     scripts=[
          "scripts/enable_streaming",
     ],
     package_dir={"": "src"}
)

setup(**d)
