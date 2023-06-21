# Installation

## Ubuntu Setup

Install a minimal installation of Ubuntu 20.04.06.

### Prepare Ubuntu

```bash
sudo apt update; sudo apt upgrade
sudo apt install ranger htop python3-pip git gnome-tweaks neovim wget
sudo pip3 install virtualenv
```

### Install ROS and build tools

Install [ROS Noetic Full](http://wiki.ros.org/noetic/Installation/Ubuntu).

```bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh; chmod +x ./ros_install_noetic.sh; ./ros_install_noetic.sh
source ~/.bashrc
```

Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).

## Spotters Installation

```bash
cd; git clone https://github.com/kinda-raffy/spotters                   # spotters must be in ~/.
cd spotters; python3 -m virtualenv --python=/usr/bin/python3 spot_venv  # Create virtual environment.
echo "source ~/spotters/.setup.bash" >> ~/.bashrc; source ~/.bashrc     # Source spotter commands.
spot_env                                                                # Activate spot's virtual environment.
git submodule init; git submodule update                                # Populate base submodules.
wspt; cd src/spot_ros; git submodule init; git submodule update         # Populate spot_ros submodules.
# Install ROS dependencies.
rosdep install --from-paths spot_driver spot_msgs spot_viz spot_description spot_cam --ignore-src -y
pip install -e spot_wrapper/                                            # Install spot SDK wrapper.
wspt; cd ..; pip install -r requirements.txt                            # Install requirements.

# --- Install required VSLAM libraries.
sudo apt install libeigen3-dev                                          # Install Eigen3.
cd; mkdir -p dev/tools; cd dev/tools                                    # Download Pangolin.
git clone https://github.com/stevenlovegrove/Pangolin.git pangolin
cd pangolin; mkdir build; cd build                                      # Build Pangolin.
cmake ..; make
sudo make install                                                       # Install Pangolin.
python3 -c "import cv2; print(cv2.__version__)"                         # Verify 4.2.0 OpenCV version.
# Download ROS package dependencies.
sudo apt install ros-noetic-hector-trajectory-server ros-noetic-octomap-server ros-noetic-usb-cam
sudo apt install v4l-utils v4l2loopback-dkms gstreamer1.0-tools         # Download camera streaming utilities.

# --- Perform a clean build.
cbuildspt
```

### Errors and known issues

#### Python packages (sip, PyKDL) not found during `pip install -r requirements.txt`

Some required python packages are not supported by pip or are defective.
Currently known packages include PyKDL and sip. In most cases, apt's own version
of these packages will work perfectly.

To resolve this, install the precompiled python packages using apt:

```txt
apt install python3-<python package>
```

Then copy over the packages from your base interpreter into your virtual environment.

```txt
cp /usr/lib/python3/dist-packages/<python package>* /<venv location>/lib/python<version>/site-packages/
```

Doing this is not ideal as those packages will not be tracked by any package manager. A better option would be
to create a symlink between the files instead of performing a hard copy. **Note down any packages you copy**.

#### Failed to build SLAM

SLAM requires dependencies installed in spot's virtual environment.
Before building workspace packages, always make sure this environment is activated through the `spot_venv` command.

#### Failed to detect successful installation of [transforms3d]

Manually install this package using pip. Makes sure spot's virtual environment is activated.

```bash
pip install transforms3d
```

#### Build error ~ c++: fatal error: Killed signal terminated program cc1plus

The build process is resource intensive. To resolve this, allocate more resources to your computer.
Our build configuration uses 7 CPUs and 11 GiB Memory.

#### I get GCC warnings but the build succeeds

This is something we will look into if we have time. Some dependencies use deprecated modules
which might cause this error. For now, ignore these warnings.