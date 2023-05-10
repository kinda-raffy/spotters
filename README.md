## Requirements

- Ubuntu 20.04
- ROS Noetic
- [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
- Python 3.8.10 and Pip
  - `python3` is assumed to be this version

## Installation

1. `cd` then `git clone https://github.com/kinda-raffy/spotters`
2. Create a virtual environment `cd spotters; python3 -m virtualenv --python=/usr/bin/python3 spot_venv`
3. `source .setup.bash`
4. Activate the pyvenv `spot_env`
5. Install requriements `pip install -r requirements.txt`
6. Populate base submodules `git submodule init; git submodule update`
7. Populate spot_ros submodules `cd workspace/src/spot_ros; git submodule init; git submodule update`
8. Install spot_ros dependencies `rosdep install --from-paths spot_driver spot_msgs spot_viz spot_description spot_cam --ignore-src -y`
9. Build workspace packages `cd ../..; catkin build`

## Guidelines

- Do not push to main.
- Categorise branches via `/` (`nav/localisation`, `nav/init_graph`)
- Submodules have been defaulted to `dev` in their respective repositories. To update dependencies in `spotters`, `git submodule update --remote` will replace the current commit with the latest on `dev`.
- To work on submodules, create an approapiate branch and merge with `dev` to publish changes. 
