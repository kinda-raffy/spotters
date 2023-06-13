from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
     packages=["spot_move"],
     scripts=[
          "scripts/enable_movement",
          "scripts/__test_main"
     ],
     package_dir={"": "src"}
)

setup(**d)
