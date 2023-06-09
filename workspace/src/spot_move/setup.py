from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
     packages=["spot_move"],
     scripts=[
          "scripts/enable_movement",
          "scripts/test/main"
     ],
     package_dir={"": "src"}
)

setup(**d)
