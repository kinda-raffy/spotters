# ORB-SLAM3-ROS

A ROS implementation of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 that focuses on the ROS part.

## Prerequisites

### Virtual environment

Always activate spot_env via `spot_env` when developing for Spot.

### Eigen3

```bash
sudo apt install libeigen3-dev
```

### Pangolin

```bash
cd; mkdir -p dev/tools; cd dev/tools
git clone https://github.com/stevenlovegrove/Pangolin.git pangolin; cd pangolin
mkdir build; cd build
cmake ..; make
sudo make install
```

### OpenCV

Verify OpenCV `4.2` installation.
Confirm this through the following command:

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

### Hector Trajectory Server

Visualises the camera frame trajectory in relation to the frame of origin.

```bash
sudo apt install ros-noetic-hector-trajectory-server
```

### Octomap

```bash
sudo apt install ros-noetic-octomap-server
```

### Camera Streamer

```bash
sudo apt install v4l-utils v4l2loopback-dkms gstreamer1.0-tools
```

## Installation

```bash
cbuildspt
```

## Camera stream

```bash
sudo modprobe v4l2loopback  # Create dummy virtual webcam.
v4l2-ctl --list-devices     # Find the /dev/video[number] of the dummy camera.
gst-launch-1.0 souphttpsrc location=http://[ip]:[port]/video ! jpegdec ! videoconvert ! v4l2sink device=/dev/video[number]
gst-launch-1.0 souphttpsrc location=http://192.168.20.87:4747/video ! jpegdec ! videoconvert ! v4l2sink device=/dev/video0
```

## Rosbag examples

The following are some examples of running the spot_slam package over standardised datasets.

### Mono mode with [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)'s [`eee_01.bag`](https://researchdata.ntu.edu.sg/api/access/datafile/68133)

```
# In one terminal:
roslaunch orb_slam3_ros ntuviral_mono.launch
# In another terminal:
rosbag play eee_01.bag -s 50 # The UAV starts moving at t~50s
```

### Mono-inertial mode with [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)'s [`MH_01_easy.bag`]( http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag)

```
# In one terminal:
roslaunch orb_slam3_ros euroc_mono_inertial.launch
# In another terminal:
rosbag play MH_01_easy.bag
```

### RGB-D mode with [TUM](http://vision.in.tum.de/data/datasets/rgbd-dataset/download)'s [`rgbd_dataset_freiburg1_xyz.bag`](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag)

```
# In one terminal:
roslaunch orb_slam3_ros tum_rgbd.launch
# In another terminal:
rosbag play rgbd_dataset_freiburg1_xyz.bag
```

- **Note**: change `TUMX.yaml` to `TUM1.yaml`,`TUM2.yaml` or `TUM3.yaml` for freiburg1, freiburg2 and freiburg3 sequences respectively.

### RGB-D-Inertial mode with [VINS-RGBD](https://github.com/STAR-Center/VINS-RGBD)'s [`Normal.bag`](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag)

- Download the bag files, for example [Normal.bag](https://star-center.shanghaitech.edu.cn/seafile/d/0ea45d1878914077ade5/).
- Decompress the bag, run `rosbag decompress Normal.bag`.
- Change the calibration params in `RealSense_D435i.yaml` if necessary.

```
# In one terminal:
roslaunch orb_slam3_ros rs_d435i_rgbd_inertial.launch.launch
# In another terminal:
rosbag play Normal.bag
```

### Save and load map

The map file will have `.osa` extension, and is located in the `ROS_HOME` folder (`~/.ros/` by default).

#### Load map

- Set the name of the map file to be loaded with `System.LoadAtlasFromFile` param in the settings file (`.yaml`).
- If the map file is not available, `System.LoadAtlasFromFile` param should be commented out otherwise there will be error.

#### Save map

- **Option 1**: If `System.SaveAtlasToFile` is set in the settings file, the map file will be automatically saved when you kill the ros node.
- **Option 2**: You can also call the following ros service at the end of the session

```bash
rosservice call /orb_slam3/save_map [file_name]
```

## 4. ROS topics, params and services

### Subscribed topics

- `/camera/image_raw` for Mono(-Inertial) node
- `/camera/left/image_raw` for Stereo(-Inertial) node
- `/camera/right/image_raw` for Stereo(-Inertial) node
- `/imu` for Mono/Stereo/RGBD-Inertial node
- `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw` for RGBD node

### Published topics

- `/orb_slam3/camera_pose`, left camera pose in world frame, published at camera rate
- `/orb_slam3/body_odom`, imu-body odometry in world frame, published at camera rate
- `/orb_slam3/tracking_image`, processed image from the left camera with key points and status text
- `/orb_slam3/tracked_points`, all key points contained in the sliding window
- `/orb_slam3/all_points`, all key points in the map
- `/orb_slam3/kf_markers`, markers for all keyframes' positions
- `/tf`, with camera and imu-body poses in world frame

### Params

- `voc_file`: path to vocabulary file required by ORB-SLAM3
- `settings_file`: path to settings file required by ORB-SLAM3
- `enable_pangolin`: enable/disable ORB-SLAM3's Pangolin viewer and interface. (`true` by default)

### Services

- `rosservice call /orb_slam3/save_map [file_name]`: save the map as `[file_name].osa` in `ROS_HOME` folder.
- `rosservice call /orb_slam3/save_traj [file_name]`: save the estimated trajectory of camera and keyframes as `[file_name]_cam_traj.txt` and  `[file_name]_kf_traj.txt` in `ROS_HOME` folder.

### Docker

Provided [Dockerfile](Dockerfile) sets up an image based a ROS noetic environment including RealSense SDK

To access a USB device (such as RealSense camera) inside docker container use:

``` bash
docker run --network host --privileged -v /dev:/dev -it [image_name]
```

> **_NOTE:_**  `--network host` is recommended to listen to rostopics outside the container

---

### Credit

1. [Orb Slam3 authors](https://github.com/UZ-SLAMLab/ORB_SLAM3)
2. [Orb Slam3 ROS authors](https://github.com/thien94/orb_slam3_ros)
