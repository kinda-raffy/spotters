# Spot SLAM

## Installation

```bash
source ~/spotters/.setup.bash
# Activate virtual environment.
spot_env
# Install Eigen3.
sudo apt install libeigen3-dev
# Install Pangolin.
cd; mkdir -p dev/tools; cd dev/tools
git clone https://github.com/stevenlovegrove/Pangolin.git pangolin; cd pangolin
mkdir build; cd build
cmake ..; make
sudo make install
# Install OpenCV 4.2
pip install -r ~/spotters/requirements.txt
# Verify correct OpenCV version.
python3 -c "import cv2; print(cv2.__version__)"
# Download octomap and a path tracker.
sudo apt install ros-noetic-hector-trajectory-server ros-noetic-octomap-server
# Dowload camera streaming utilities.
sudo apt install v4l-utils v4l2loopback-dkms gstreamer1.0-tools
# Perform a clean build.
cbuildspt
```

## Enable camera streaming

```bash
# Create dummy virtual webcam.
sudo modprobe v4l2loopback
# Find the /dev/video[number] of the dummy camera.
v4l2-ctl --list-devices
# Define a streaming pipeline to convert deocoded data to the dummy device.
gst-launch-1.0 souphttpsrc location=http://[ip]:[port]/video ! jpegdec ! videoconvert ! v4l2sink device=/dev/video[number]
```

### Using Docker

The provided [Dockerfile](Dockerfile) sets up an image based a ROS noetic environment.

To access a USB device (such as a camera) inside docker container perform:

``` bash
docker run --network host --privileged -v /dev:/dev -it [image_name]
```

> **_NOTE:_**  `--network host` is recommended to listen to rostopics outside the container

---

### Credit

1. [Orb SLAM3 authors](https://github.com/UZ-SLAMLab/ORB_SLAM3)
2. [Orb SLAM3 ROS authors](https://github.com/thien94/orb_slam3_ros)
